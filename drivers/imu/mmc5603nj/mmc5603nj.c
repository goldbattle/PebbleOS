/* SPDX-FileCopyrightText: 2025 Matthew Wardrop */
/* SPDX-FileCopyrightText: 2025 Bob Wei */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <pbl/devicetree.h>
#include <pbl/drivers/i2c.h>
#include <pbl/drivers/imu/mmc5603nj/mmc5603nj.h>
#include <pbl/drivers/imu/mmc5603nj/registers.h>
#include <pbl/drivers/mag.h>
#include <pbl/kernel/delay.h>
#include <pbl/kernel/time.h>
#include <pbl/logging/logging.h>
#include <pbl/services/imu/units.h>
#include <pbl/util/byteorder.h>
#include <pbl/util/math.h>
#include <pbl/util/misc.h>

PBL_LOG_MODULE_DECLARE(driver_mag, CONFIG_DRIVER_IMU_LOG_LEVEL);

#define MMC5603NJ_RATE_HZ_MAX 255U

// The sensor free-runs and has no data-ready timestamp, so it is polled at 4x its output rate. A
// sample found ready was completed at some point since the previous poll, so it is stamped at the
// middle of that window: the stamp is within +-poll_period/2 of the real measurement time (plus
// the small, fixed I2C read latency). Polling period floor is the 1 ms timer resolution.
#define MMC5603NJ_POLL_OVERSAMPLE 4U

static uint32_t s_poll_period_us;

// Temperature (TOUT, 8 bit: -75 C + 0.8 C/LSB, per datasheet) is measured about once a second by
// setting TAKE_MEAS_T while continuous mode keeps running, then collected on a later poll when
// STATUS1 reports it done. NOTE: not verified on hardware whether the part accepts TM_T in
// continuous mode; if it never reports done the temperature simply stays unavailable (INT16_MIN).
#define MMC5603NJ_TEMP_C100_UNAVAILABLE INT16_MIN

static int16_t s_temp_c100 = MMC5603NJ_TEMP_C100_UNAVAILABLE;
static bool s_temp_pending;
static uint32_t s_temp_polls;  // polls since the last request (or since a request was dropped)

static const struct pbl_mmc5603nj *s_dev;

static int prv_read(uint8_t reg, uint8_t *data, uint32_t len) {
  int ret;

  ret = pbl_i2c_burst_read(&s_dev->i2c, reg, data, len);
  if (ret != 0) {
    PBL_LOG_ERR("Could not read register 0x%02x (%d)", reg, ret);
  }

  return ret;
}

static int prv_write(uint8_t reg, uint8_t val) {
  int ret;

  ret = pbl_i2c_reg_write_byte(&s_dev->i2c, reg, val);
  if (ret != 0) {
    PBL_LOG_ERR("Could not write register 0x%02x (%d)", reg, ret);
  }

  return ret;
}

static int prv_is_data_ready(bool *ready) {
  uint8_t status;
  int ret;

  ret = prv_read(MMC5603NJ_REG_STATUS1, &status, 1);
  if (ret != 0) {
    return ret;
  }

  *ready = (status & MMC5603NJ_STATUS1_MEAS_M_DONE_MASK) != 0U;

  return 0;
}

static int16_t prv_get_axis(const int16_t raw[3], IMUCoordinateAxis axis) {
  int16_t val = s_dev->axis_dir[axis] * raw[s_dev->axis_map[axis]];

  if (s_dev->state->rotated && (axis == AXIS_X || axis == AXIS_Y)) {
    val = -val;
  }

  // 1024 counts per gauss in 16-bit mode
  return (int16_t)(((int32_t)val * 1000) / 1024);
}

static int prv_read_sample(MagData *sample) {
  uint8_t data[6];
  int16_t raw[3];
  int ret;

  ret = prv_read(MMC5603NJ_REG_XOUT0, data, sizeof(data));
  if (ret != 0) {
    return ret;
  }

  // Unsigned, offset by 2^15
  for (uint8_t i = 0U; i < 3U; i++) {
    raw[i] = (int16_t)(pbl_get_be16(&data[2U * i]) - (1U << 15));
  }

  sample->x = prv_get_axis(raw, AXIS_X);
  sample->y = prv_get_axis(raw, AXIS_Y);
  sample->z = prv_get_axis(raw, AXIS_Z);
  sample->temp_c100 = s_temp_c100;
  sample->timestamp_us = pbl_realtime_us();

  return 0;
}

static int prv_reset(void) {
  int ret;

  ret = prv_write(MMC5603NJ_REG_CTRL1,
                  PBL_FIELD_PREP(MMC5603NJ_CTRL1_BANDWIDTH_MASK, MMC5603NJ_CTRL1_BANDWIDTH_6ms6) |
                      MMC5603NJ_CTRL1_SW_RESET);
  if (ret != 0) {
    return ret;
  }

  pbl_sleep_ms(MMC5603NJ_SW_RESET_DELAY_MS);

  ret = prv_write(MMC5603NJ_REG_CTRL0, MMC5603NJ_CTRL0_DO_SET);
  if (ret != 0) {
    return ret;
  }

  pbl_sleep_ms(MMC5603NJ_SET_DELAY_MS);

  ret = prv_write(MMC5603NJ_REG_CTRL0, MMC5603NJ_CTRL0_DO_RESET);
  if (ret != 0) {
    return ret;
  }

  pbl_sleep_ms(MMC5603NJ_SET_DELAY_MS);

  return 0;
}

static int prv_set_rate_hw(uint8_t rate_hz) {
  MagData discard;
  bool ready;
  int ret;

  s_temp_pending = false;
  s_temp_polls = 0U;

  // Leave continuous mode
  ret = prv_write(MMC5603NJ_REG_CTRL2, 0x00);
  if (ret != 0) {
    return ret;
  }

  // Clear any pending data-ready flag
  if (prv_is_data_ready(&ready) == 0 && ready) {
    (void)prv_read_sample(&discard);
  }

  if (rate_hz == 0U) {
    return 0;
  }

  ret = prv_write(MMC5603NJ_REG_ODR, rate_hz);
  if (ret != 0) {
    return ret;
  }

  // Recompute the measurement period
  ret = prv_write(MMC5603NJ_REG_CTRL0, MMC5603NJ_CTRL0_AUTO_SR_EN | MMC5603NJ_CTRL0_CMM_FREQ_EN);
  if (ret != 0) {
    return ret;
  }

  return prv_write(MMC5603NJ_REG_CTRL2, PBL_FIELD_PREP(MMC5603NJ_CTRL2_AUTOSET_PRD_MASK,
                                                       MMC5603NJ_CTRL2_AUTOSET_PRD_100) |
                                            MMC5603NJ_CTRL2_PRD_SET_EN | MMC5603NJ_CTRL2_CMM_EN);
}

static void prv_temperature_step(void) {
  const uint32_t polls_per_second = 1000000U / s_poll_period_us;
  uint8_t status;
  uint8_t raw;

  s_temp_polls++;

  if (s_temp_pending) {
    if (prv_read(MMC5603NJ_REG_STATUS1, &status, 1) == 0 &&
        (status & MMC5603NJ_STATUS1_MEAS_T_DONE_MASK) != 0U) {
      if (prv_read(MMC5603NJ_REG_TOUT, &raw, 1) == 0) {
        s_temp_c100 = (int16_t)((int32_t)raw * 80 - 7500);
      }
      (void)prv_write(MMC5603NJ_REG_STATUS1, MMC5603NJ_STATUS1_MEAS_T_DONE_MASK);
      s_temp_pending = false;
    } else if (s_temp_polls >= 2U * polls_per_second) {
      s_temp_pending = false;  // gave up on this one, try again
    }
    return;
  }

  if (s_temp_polls >= polls_per_second) {
    // Same value as set up by prv_set_rate_hw(), plus the self-clearing temperature trigger
    if (prv_write(MMC5603NJ_REG_CTRL0, MMC5603NJ_CTRL0_AUTO_SR_EN | MMC5603NJ_CTRL0_CMM_FREQ_EN |
                                           MMC5603NJ_CTRL0_TAKE_MEAS_T) == 0) {
      s_temp_pending = true;
    }
    s_temp_polls = 0U;
  }
}

static uint32_t prv_poll_period_ms(uint8_t rate_hz) {
  return PBL_MAX(1U, 1000U / rate_hz / MMC5603NJ_POLL_OVERSAMPLE);
}

static pbl_tick_t prv_poll_period_ticks(uint8_t rate_hz) {
  return pbl_ms_to_ticks(prv_poll_period_ms(rate_hz));
}

static void prv_poll_fn(struct pbl_work *work) {
  struct pbl_mmc5603nj_state *state = s_dev->state;
  MagData sample;
  bool ready;
  int ret;

  pbl_mag_lock(&s_dev->mag);

  if (state->rate_hz == 0U) {
    goto out;
  }

  prv_temperature_step();

  ret = prv_is_data_ready(&ready);
  if (ret == 0 && !ready) {
    goto out;
  }

  if (ret == 0) {
    ret = prv_read_sample(&sample);
  }

  if (ret != 0) {
    PBL_LOG_WRN("Read failed (%d), restarting sampling", ret);
    if (prv_set_rate_hw(0U) == 0) {
      (void)prv_set_rate_hw(state->rate_hz);
    }
    goto out;
  }

  // prv_read_sample() stamped the read time; the measurement finished within the last poll period
  sample.timestamp_us -= s_poll_period_us / 2U;
  pbl_mag_report_sample(&s_dev->mag, &sample);

out:
  // Rate changes hold the lock too
  if (state->rate_hz != 0U && !pbl_delayed_work_is_pending(&state->poll_timer)) {
    pbl_delayed_work_schedule_next(&state->poll_timer, prv_poll_period_ticks(state->rate_hz));
  }

  pbl_mag_unlock(&s_dev->mag);
}

static int prv_init(const struct pbl_mag *mag) {
  uint8_t whoami;
  int ret;

  s_dev = PBL_CONTAINER_OF(mag, const struct pbl_mmc5603nj, mag);
  pbl_delayed_work_init(&s_dev->state->poll_timer, prv_poll_fn);

  ret = prv_read(MMC5603NJ_REG_WHO_AM_I, &whoami, 1);
  if (ret != 0) {
    return -ENODEV;
  }

  if (whoami != MMC5603NJ_WHO_AM_I_VALUE) {
    PBL_LOG_ERR("Unexpected id: 0x%02x!=0x%02x", whoami, MMC5603NJ_WHO_AM_I_VALUE);
    return -ENODEV;
  }

  ret = prv_reset();
  if (ret != 0) {
    PBL_LOG_ERR("Could not reset (%d)", ret);
    return ret;
  }

  return 0;
}

static int prv_set_rotated(const struct pbl_mag *mag, bool rotated) {
  s_dev->state->rotated = rotated;

  return 0;
}

static int prv_set_sampling_interval(const struct pbl_mag *mag, uint32_t interval_us,
                                     uint32_t *actual_us) {
  struct pbl_mmc5603nj_state *state = s_dev->state;
  uint8_t rate_hz = 0U;
  int ret;

  if (interval_us > 0U) {
    rate_hz =
        (uint8_t)PBL_CLAMP(PBL_DIV_ROUND_UP(1000000U, interval_us), 1U, MMC5603NJ_RATE_HZ_MAX);
  }

  if (rate_hz != state->rate_hz) {
    PBL_LOG_DBG("Setting sample rate to %u Hz", rate_hz);

    ret = prv_set_rate_hw(rate_hz);
    if (ret != 0) {
      return ret;
    }

    state->rate_hz = rate_hz;

    if (rate_hz == 0U) {
      pbl_delayed_work_cancel(&state->poll_timer);
    } else {
      s_poll_period_us = prv_poll_period_ms(rate_hz) * 1000U;
      pbl_delayed_work_reschedule(&pbl_sys_workq_high, &state->poll_timer,
                                  PBL_TICKS(prv_poll_period_ticks(rate_hz)));
    }
  }

  *actual_us = (rate_hz == 0U) ? 0U : 1000000U / rate_hz;

  return 0;
}

static int prv_peek(const struct pbl_mag *mag, MagData *sample) {
  if (s_dev->state->rate_hz == 0U) {
    return -ENODATA;
  }

  return prv_read_sample(sample);
}

const struct pbl_mag_ops pbl_mmc5603nj_ops = {
  .init = prv_init,
  .set_rotated = prv_set_rotated,
  .set_sampling_interval = prv_set_sampling_interval,
  .peek = prv_peek,
};

#include PBL_DT_INSTANCES(memsic_mmc5603)
