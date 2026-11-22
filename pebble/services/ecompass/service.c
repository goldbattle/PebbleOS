/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/mag.h>
#include <pbl/kernel/assert.h>
#include <pbl/kernel/malloc.h>
#include <pbl/kernel/syscall.h>
#include <pbl/kernel/work.h>
#include <pbl/logging/logging.h>
#include <pbl/services/ecompass.h>
#include <pbl/services/event_service.h>
#include <pbl/util/trig.h>

#include <applib/accel_service.h>
#include <applib/compass_service.h>
#include <applib/compass_service_private.h>
#include <syscall/syscall.h>

PBL_LOG_MODULE_DEFINE(service_ecompass, CONFIG_SERVICE_ECOMPASS_LOG_LEVEL);

// Duration (in minutes) to run high-frequency sampling during compass calibration.
// Defaults to 2 minutes, but some platforms (e.g., Asterix) require longer.
#ifdef CONFIG_BOARD_ASTERIX
#define ECOMPASS_CALIBRATION_FAST_MINUTES 6
#else
#define ECOMPASS_CALIBRATION_FAST_MINUTES 2
#endif

#define ECOMPASS_SAMPLING_INTERVAL_US       200000U
#define ECOMPASS_CALIB_SAMPLING_INTERVAL_US 50000U

#define VALID_CORR_MARKER       0x5644
#define BITS_PER_CORRECTION_VAL 16
#define CORRECTION_VAL_MASK     ((1 << BITS_PER_CORRECTION_VAL) - 1)

static CompassStatus s_current_cal_status = CompassStatusDataInvalid;
static int16_t s_active_corr[3] = {};

static bool s_service_init = false;
static bool s_saved_corr_present = false;
static int16_t s_saved_corr[3] = {};

static int32_t s_last_heading = -1; // the last heading we found

static MagData s_mag_sample;

//////////////////////////////////////////////////////////////////////////////////
// Raw magnetometer subscribers (mag_data_service_subscribe)

typedef struct {
  // What the task asked for (sys_ecompass_mag_request)
  uint32_t rate_hz;
  uint32_t per_update;
  // Active from the task's event service subscription until its unsubscription or exit. All of
  // the following is guarded by the magnetometer device lock.
  MagSample *ring;  // null when not active; capacity 2 * per_update
  uint16_t cap;
  uint16_t head;
  uint16_t count;
  uint32_t dropped;
  bool notified;  // a PEBBLE_MAG_SAMPLE_EVENT is on its way and the batch has not been fetched
} MagSubscriber;

static MagSubscriber s_mag_subs[NumPebbleTask];
// Watch time of the last sample that was passed on to the compass heading code
static uint64_t s_last_compass_sample_us;

//////////////////////////////////////////////////////////////////////////////////
// Calibration state variables
static void prv_calibration_time_expired_cb(struct pbl_work *work);

static bool s_high_freq_calib_active = false;
static bool s_calib_run = false;
static struct pbl_wallclock_work s_cb_info;

//////////////////////////////////////////////////////////////////////////////////
// Compass subscription state variables

static uint8_t s_compass_subscribers_count = 0;
static bool s_compass_subscribers[NumPebbleTask] = {};

//////////////////////////////////////////////////////////////////////////////////
// Accel service state variables

static AccelServiceState *s_accel_session = nullptr;
static bool s_charger_plugged = false;
static AccelRawData s_accel_data = {};

//////////////////////////////////////////////////////////////////////////////////
// Private calibration handlers

static void prv_get_roll_and_pitch(AccelRawData *d, int32_t *rollp, int32_t *pitchp) {
  if ((d->x == 0) && (d->y == 0) && (d->z == 0)) {
    *rollp = *pitchp = 0;
    return;
  }

  int32_t roll = atan2_lookup(d->y, d->z);

  int32_t act_ang = (roll * 360) / TRIG_MAX_ANGLE;
  if (act_ang > 180) {
    roll = roll - TRIG_MAX_ANGLE;
  }

  int32_t pitch =
      atan2_lookup(-d->x, (d->y * sin_lookup(roll) + d->z * cos_lookup(roll)) / TRIG_MAX_RATIO);

  // solution repeats every 180 degrees
  if (pitch > (TRIG_MAX_ANGLE / 4)) { // > 90 degrees
    if (pitch < ((270 * TRIG_MAX_ANGLE) / 360)) {
      pitch -= (TRIG_MAX_ANGLE / 2);
    } else {
      pitch -= TRIG_MAX_ANGLE;
    }
  }

  *rollp = roll;
  *pitchp = pitch;
}

static int32_t prv_correct_for_roll_and_pitch(AccelRawData *accel_data, MagData *mag_data,
                                              int32_t roll, int32_t pitch) {
  int32_t mx = mag_data->x - s_active_corr[0];
  int32_t my = mag_data->y - s_active_corr[1];
  int32_t mz = mag_data->z - s_active_corr[2];

  int32_t mx_rot, my_rot;

  // per freescale AN4249, roll is unstable close to vertical but pitch is ok
  int32_t corr = 0;
  if (PBL_TRIGANGLE_TO_DEG(pitch) > 82) {
    pitch = TRIG_MAX_ANGLE / 4;
    roll = 0;
  } else if (accel_data->z < 0) {
    // the watch has been flipped over. If someone is viewing the watch
    // at a pitch > 90 degrees, this means the heading will rotate around
    // on them (since technically, the 'front' of the watch is pointing
    // at them.) Flip the heading back around in this case.
    corr = TRIG_MAX_ANGLE / 2;
  }

  mx_rot = (mx * cos_lookup(pitch)) / TRIG_MAX_RATIO;
  mx_rot += (((my * sin_lookup(pitch)) / TRIG_MAX_RATIO) * sin_lookup(roll)) / TRIG_MAX_RATIO;
  mx_rot += (((mz * sin_lookup(pitch)) / TRIG_MAX_RATIO) * cos_lookup(roll)) / TRIG_MAX_RATIO;

  my_rot = mz * sin_lookup(roll) - my * cos_lookup(roll);
  my_rot /= TRIG_MAX_RATIO;

  int32_t heading = (atan2_lookup(-my_rot, mx_rot) + corr) % TRIG_MAX_ANGLE;
  return (heading);
}

//////////////////////////////////////////////////////////////////////////////////
// Private handlers for compass service

// The magnetometer runs at the fastest rate any user wants, or not at all
static void prv_update_mag_rate(void) {
  uint32_t interval_us = 0;
  if (s_compass_subscribers_count > 0) {
    interval_us = s_high_freq_calib_active ? ECOMPASS_CALIB_SAMPLING_INTERVAL_US
                                           : ECOMPASS_SAMPLING_INTERVAL_US;
  }

  pbl_mag_lock(MAG);
  for (int i = 0; i < NumPebbleTask; i++) {
    if (s_mag_subs[i].ring != nullptr) {
      // Rounded up so that the driver, which rounds the rate up, lands on rate_hz
      const uint32_t mag_us = (1000000U + s_mag_subs[i].rate_hz - 1U) / s_mag_subs[i].rate_hz;
      if (interval_us == 0 || mag_us < interval_us) {
        interval_us = mag_us;
      }
    }
  }
  const int ret = pbl_mag_set_sampling_interval(MAG, interval_us, nullptr);
  pbl_mag_unlock(MAG);

  if (ret != 0) {
    PBL_LOG_WRN("Could not set magnetometer interval %" PRIu32, interval_us);
  }
}

static void prv_calibration_time_expired_cb(struct pbl_work *work) {
  PBL_LOG_DBG(
      "Calibration time expired, complete, or app exit, "
      "dropping back to low frequency");

  s_high_freq_calib_active = false;
  prv_update_mag_rate();
  pbl_wallclock_work_stop(&s_cb_info);
}

static void prv_accel_for_compass_handler(AccelRawData *d, uint32_t num_samples,
                                          uint64_t timestamp) {
  int32_t x = 0, y = 0, z = 0;

  // 1st order butterworth filter with a cutoff freq of 0.02Fs
  static int32_t xp = 0, yp = 0, zp = 0;
  static int32_t xr = 0, yr = 0, zr = 0;

  for (uint32_t i = 0; i < num_samples; i++) {
    xr = (305 * d[i].x + 305 * xp + 9391 * xr) / 10000;
    yr = (305 * d[i].y + 305 * yp + 9391 * yr) / 10000;
    zr = (305 * d[i].z + 305 * zp + 9391 * zr) / 10000;
    xp = d[i].x;
    yp = d[i].y;
    zp = d[i].z;
    x += xr;
    y += yr;
    z += zr;
  }

  x /= (int)num_samples;
  y /= (int)num_samples;
  z /= (int)num_samples;

  // TODO: could this ever be called in the middle of ecompass_service_handle?
  // map accel data to NED coordinate system
  s_accel_data.x = y;
  s_accel_data.y = x;
  s_accel_data.z = -z;
}

static void prv_compass_data_service_stop(PebbleTask task) {
  if (s_compass_subscribers[task]) {
    s_compass_subscribers[task] = false;

    if (--s_compass_subscribers_count == 0) {
      // If this was the last subscribed process, then stop the compass
      // service
      if (s_high_freq_calib_active) {
        prv_calibration_time_expired_cb(nullptr);
        s_calib_run = false;
      }
      accel_session_data_unsubscribe(s_accel_session);
      accel_session_delete(s_accel_session);
      s_accel_session = nullptr;
    }
    prv_update_mag_rate();
  }
  PBL_LOG_DBG("subscribers %" PRIu8, s_compass_subscribers_count);
}

static void prv_compass_data_service_start(PebbleTask task) {
  prv_compass_data_service_stop(task);
  s_compass_subscribers[task] = true;
  if (++s_compass_subscribers_count == 1) {
    // If this is the first subscriber to the compass service, start sampling
    PBL_ASSERTN(s_accel_session == nullptr);

    s_accel_session = accel_session_create();
    accel_session_raw_data_subscribe(s_accel_session, ACCEL_SAMPLING_25HZ, 5,
                                     prv_accel_for_compass_handler);
  }
  prv_update_mag_rate();
  PBL_LOG_DBG("subscribers %" PRIu8, s_compass_subscribers_count);
}

static void prv_post_event_work(void *data) {
  PebbleEvent e = {
    .type = PEBBLE_ECOMPASS_SERVICE_EVENT,
  };
  event_put(&e);
}

static void prv_post_mag_notification_work(void *data) {
  PebbleEvent e = {.type = PEBBLE_MAG_SAMPLE_EVENT};
  event_put(&e);
}

static void prv_mag_data_service_stop(PebbleTask task) {
  MagSubscriber *sub = &s_mag_subs[task];

  pbl_mag_lock(MAG);
  pbl_free(sub->ring);
  sub->ring = nullptr;
  pbl_mag_unlock(MAG);

  prv_update_mag_rate();
}

static void prv_mag_data_service_start(PebbleTask task) {
  MagSubscriber *sub = &s_mag_subs[task];

  prv_mag_data_service_stop(task);
  if (sub->rate_hz == 0 || sub->per_update == 0) {
    return;
  }

  MagSample *ring = pbl_zalloc(2U * sub->per_update * sizeof(MagSample));
  if (ring == nullptr) {
    PBL_LOG_ERR("No memory for magnetometer samples");
    return;
  }

  pbl_mag_lock(MAG);
  sub->cap = 2U * sub->per_update;
  sub->head = 0;
  sub->count = 0;
  sub->dropped = 0;
  sub->notified = false;
  sub->ring = ring;
  pbl_mag_unlock(MAG);

  prv_update_mag_rate();
}

static void prv_mag_sample_cb(const struct pbl_mag *mag, const MagData *sample, void *user_data) {
  s_mag_sample = *sample;

  bool notify = false;
  const MagSample ms = {
    .x = sample->x,
    .y = sample->y,
    .z = sample->z,
    .temp_c100 = sample->temp_c100,
    .calibrated = (s_current_cal_status == CompassStatusCalibrated),
    .timestamp_us = sample->timestamp_us,
  };
  for (int i = 0; i < NumPebbleTask; i++) {
    MagSubscriber *sub = &s_mag_subs[i];
    if (sub->ring == nullptr) {
      continue;
    }
    if (sub->count == sub->cap) {
      sub->dropped++;  // the task is behind: drop rather than block
      continue;
    }
    sub->ring[(sub->head + sub->count) % sub->cap] = ms;
    sub->count++;
    if (sub->count >= sub->per_update && !sub->notified) {
      sub->notified = true;
      notify = true;
    }
  }
  if (notify) {
    pbl_sys_work_call_high(prv_post_mag_notification_work, nullptr);
  }

  // The magnetometer may run faster than the compass needs because of raw subscribers: feed the
  // heading code at the compass rate only (half a sensor period of slack for jitter).
  // event_put() may block on KernelMain, which takes the device lock: post outside of it
  if (s_compass_subscribers_count > 0) {
    const uint32_t compass_us = s_high_freq_calib_active ? ECOMPASS_CALIB_SAMPLING_INTERVAL_US
                                                         : ECOMPASS_SAMPLING_INTERVAL_US;
    const uint32_t slack_us = pbl_mag_get_sampling_interval(MAG) / 2U;
    if (sample->timestamp_us - s_last_compass_sample_us + slack_us >= compass_us) {
      s_last_compass_sample_us = sample->timestamp_us;
      pbl_sys_work_call_high(prv_post_event_work, nullptr);
    }
  }
}

static const struct pbl_mag_listener s_mag_listener = {
  .sample = prv_mag_sample_cb,
};

//////////////////////////////////////////////////////////////////////////////////
// Public API

void ecompass_handle_battery_state_change_event(PreciseBatteryChargeState new_state) {
  if (new_state.is_plugged) {
    s_charger_plugged = true;
    s_current_cal_status = CompassStatusDataInvalid;
    memset(s_active_corr, 0x00, sizeof(s_active_corr));
    s_saved_corr_present = false;
  } else if (s_charger_plugged) {
    // we have unplugged the charger, initiate recalibration
    s_charger_plugged = false;
    s_calib_run = false; // trigger a rerun of fast compass calibration
    PBL_LOG_DBG("Restarting calibration after charge event");
  }
}

void ecompass_service_init(void) {
  s_service_init = true;
  pbl_wallclock_work_init(&s_cb_info, prv_calibration_time_expired_cb);

  (void)pbl_mag_set_listener(MAG, &s_mag_listener);

  event_service_init(PEBBLE_COMPASS_DATA_EVENT, &prv_compass_data_service_start,
                     &prv_compass_data_service_stop);
  event_service_init(PEBBLE_MAG_SAMPLE_EVENT, &prv_mag_data_service_start,
                     &prv_mag_data_service_stop);
}

void ecompass_service_handle(void) {
  static int samples_collected = 0;

  // Samples are reported for any magnetometer user
  if (s_compass_subscribers_count == 0) {
    return;
  }

  MagData mag_data;
  pbl_mag_lock(MAG);
  mag_data = s_mag_sample;
  pbl_mag_unlock(MAG);

  // industry standard for heading coordinates uses NED convention (check out
  // Freescale's AN4248 or ST's AN3192 as examples). Therefore, we map pebbles
  // coordinate system (ENU) to NED in this service module
  int16_t my_orig = mag_data.y;
  mag_data.y = mag_data.x;
  mag_data.x = my_orig;
  mag_data.z = -mag_data.z;

  samples_collected++;

  // don't perform any calibration if the charger is plugged in
  if (!s_charger_plugged && (s_current_cal_status != CompassStatusCalibrated)) {
    // if we haven't tried to calibrate yet, run at a higher sampling rate
    // for a short duration in order to finish calibration more quickly
    if (!s_calib_run) {
      ecomp_corr_reset();
      s_high_freq_calib_active = true;
      prv_update_mag_rate();
      pbl_wallclock_work_start(&pbl_sys_workq_high, &s_cb_info, 60,
                               ECOMPASS_CALIBRATION_FAST_MINUTES);
      s_calib_run = true;
      samples_collected = 0;
      return; // if we are switching to high freq mode, don't use 1st sample
    }

    if (samples_collected < 5) {
      return; // wait a few samples for ramp up error to stabilize
    }

    int16_t new_corr[3];
    MagCalStatus cal_status = ecomp_corr_add_raw_mag_sample(
        (int16_t *)&mag_data, (s_saved_corr_present) ? s_saved_corr : nullptr, new_corr);

    if (cal_status != MagCalStatusNoSolution) {
      PBL_LOG_INFO("%s : %d %d %d (type = %d)", "Mag Corr", (int)new_corr[0], (int)new_corr[1],
                   (int)new_corr[2], (int)cal_status);
    }

    bool locked_sol = (cal_status == MagCalStatusNewLockedSolutionAvail);
    if (locked_sol || ((cal_status == MagCalStatusNewSolutionAvail) && !s_saved_corr_present)) {
      s_current_cal_status = CompassStatusCalibrating;
      for (int i = 0; i < 3; i++) {
        if ((s_active_corr[i] == 0) || locked_sol) {
          s_active_corr[i] = new_corr[i];
          continue;
        }

        // smooth out noise from solutions while we wait for a locked set
        const int alpha = 30; // greater alpha leads to less smoothing
        new_corr[i] = ((new_corr[i] - s_active_corr[i]) * alpha) / 100;
        s_active_corr[i] += new_corr[i];
      }
    }

    if (s_high_freq_calib_active && (cal_status == MagCalStatusNoSolution) &&
        ((samples_collected % 4) != 0)) {
      return; // only bubble up every 4th sample to app land with high samp rate
    }

    if ((cal_status == MagCalStatusNewLockedSolutionAvail) ||
        (cal_status == MagCalStatusSavedSampleMatch)) {
      if (s_high_freq_calib_active) {
        prv_calibration_time_expired_cb(nullptr);
      }
      s_current_cal_status = CompassStatusCalibrated;
      for (int i = 0; i < 3; i++) {
        s_active_corr[i] = new_corr[i];
      }
      s_saved_corr_present = true;
    }
  }

  // get the most recent accel readings
  AccelRawData accel_data = s_accel_data;
  int32_t roll = 0, pitch = 0;
  prv_get_roll_and_pitch(&accel_data, &roll, &pitch);

  PebbleEvent e = {
    .type = PEBBLE_COMPASS_DATA_EVENT,
    .compass_data = {
      .magnetic_heading = prv_correct_for_roll_and_pitch(&accel_data, &mag_data, roll, pitch),
      .calib_status = s_current_cal_status
    }
  };

  s_last_heading = e.compass_data.magnetic_heading;
  event_put(&e);
}

//////////////////////////////////////////////////////////////////////////////////
// System call handlers

PBL_SYSCALL(uint32_t, sys_ecompass_mag_request, (VAL, uint32_t, rate_hz),
            (VAL, uint32_t, samples_per_update)) {
  MagSubscriber *sub = &s_mag_subs[pebble_task_get_current()];

  if (rate_hz == 0 || samples_per_update == 0 || !pbl_device_is_ready(&MAG->dev)) {
    sub->rate_hz = 0;
    return 0;
  }

  sub->rate_hz = PBL_CLAMP(rate_hz, 1U, MAG_RATE_HZ_MAX);
  sub->per_update = PBL_CLAMP(samples_per_update, 1U, MAG_SAMPLES_PER_UPDATE_MAX);
  return sub->rate_hz;
}

PBL_SYSCALL(uint32_t, sys_ecompass_mag_get_samples, (VAL, uint32_t, max),
            (OUT_BUF, MagSample *, samples, max * sizeof(MagSample))) {
  MagSubscriber *sub = &s_mag_subs[pebble_task_get_current()];

  if (max > MAG_SAMPLES_PER_UPDATE_MAX) {
    pbl_syscall_fail();
  }

  pbl_mag_lock(MAG);
  uint32_t n = 0;
  if (sub->ring != nullptr) {
    n = PBL_MIN(max, (uint32_t)sub->count);
    for (uint32_t i = 0; i < n; i++) {
      samples[i] = sub->ring[(sub->head + i) % sub->cap];
    }
    sub->head = (sub->head + n) % sub->cap;
    sub->count -= n;
    if (sub->count < sub->per_update) {
      sub->notified = false;
    }
    if (sub->dropped != 0) {
      PBL_LOG_WRN("Magnetometer subscriber too slow, dropped %" PRIu32 " samples", sub->dropped);
      sub->dropped = 0;
    }
  }
  pbl_mag_unlock(MAG);
  return n;
}

PBL_SYSCALL(bool, sys_ecompass_service_subscribed) {
  PebbleTask task = pebble_task_get_current();
  return s_compass_subscribers[task];
}

PBL_SYSCALL(void, sys_ecompass_get_last_heading, (OUT, CompassHeadingData *, data)) {
  *data = (CompassHeadingData){
    .magnetic_heading = s_last_heading,
    .true_heading = s_last_heading,
    .compass_status = s_current_cal_status,
    .is_declination_valid = false
  };
}
