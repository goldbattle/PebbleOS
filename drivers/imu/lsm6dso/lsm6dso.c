/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <inttypes.h>

#include <pbl/analytics/analytics.h>
#include <pbl/devicetree.h>
#include <pbl/drivers/accel.h>
#include <pbl/drivers/gpio.h>
#include <pbl/drivers/i2c.h>
#include <pbl/drivers/imu/lsm6dso/lsm6dso.h>
#include <pbl/kernel/delay.h>
#include <pbl/kernel/time.h>
#include <pbl/kernel/work.h>
#include <pbl/logging/logging.h>
#include <pbl/services/imu/units.h>
#include <pbl/util/bits.h>
#include <pbl/util/byteorder.h>
#include <pbl/util/math.h>
#include <pbl/util/misc.h>
#include <pbl/util/size.h>

PBL_LOG_MODULE_DECLARE(driver_accel, CONFIG_DRIVER_IMU_LOG_LEVEL);

// The single instance, for interrupt and timer contexts
static const struct pbl_lsm6dso *s_dev;

// Implementation notes:
//
// - The gyroscope is off unless requested (set_config). It then runs at the accelerometer ODR in
//   high-performance mode (ULP off), batched in the FIFO with the accelerometer.
// - The full-scale range is CONFIG_ACCEL_LSM6DSO_SCALE_MG unless set_config requests another. The
//   wake-up threshold is in units of the range, so like with ODR changes, shake sensitivity is
//   lower while a larger range is in use.
// - Every FIFO time slot carries the sensor's own timestamp (TIMESTAMP_EN, DEC_TS_BATCH=1). Each
//   FIFO read also reads the live timestamp counter next to the system uptime, so every sample's
//   time comes from the sensor's own clock, mapped onto uptime by a continuous line fit over those
//   reads (pbl/kernel/clocksync.h). The fit starts from the factory trim (INTERNAL_FREQ_FINE).
// - Peeking returns the last FIFO sample when sampling is active, otherwise a
//   single-shot measurement is performed.
// - Ultra-low-power mode (XL_ULP_EN) is always used (minimum power). It is
//   enabled once at init while the accelerometer is powered down (as required
//   by the datasheet) and never toggled afterwards. The gyroscope is never
//   enabled, so ULP is allowed.
// - ODR is limited to the [12.5, 208] Hz range (the ULP ODR range).
// - Shake detection uses 12.5Hz when no active sampling is ongoing.
// - Wake-up duration absolute time depends on the ODR, a parameter that can
//   be changed depending on the sampling interval configuration. Value is NOT
//   adjusted automatically when ODR changes, so it is possible to notice
//   sensitivity changes when changing sampling interval.
// - INT1 mixes level (FIFO threshold) and latched (wake-up) sources on a
//   rising-edge interrupt, so a pad latched HIGH yields no further edges: shake
//   events die and the pending interrupt blocks deep sleep. The work handler
//   re-checks the pad after servicing; a watchdog recovers the stream (FIFO
//   cadence while sampling, fixed-rate pad poll in shake-only mode).

// Time to wait after reset (us)
#define LSM6DSO_RESET_TIME_US 5

// DRDY polling parameters for single-shot peeks
#define LSM6DSO_DRDY_POLL_DELAY_MS   (5)   /* ms between data-ready polls */
#define LSM6DSO_DRDY_POLL_TIMEOUT_MS (100) /* max wait (~5x 20ms at 52Hz ODR) */

// FIFO threshold periods without a FIFO read before the stream is considered
// stalled (>1x to tolerate scheduling jitter), and threshold lower bound
#define LSM6DSO_STALL_MARGIN 2U
#define LSM6DSO_STALL_MIN_MS 1000U

// Max buffer-sized reads when draining the FIFO
#define LSM6DSO_DRAIN_CHUNKS_MAX 4U

// INT1 watchdog poll rate in shake-only mode (no FIFO cadence to track)
#define LSM6DSO_SHAKE_WDT_PERIOD_S 5U

// Consecutive stuck-high watchdog passes before a full stream recovery
#define LSM6DSO_SHAKE_STUCK_PASSES_MAX 3U

// Scale range for 16-bit two's complement samples
#define LSM6DSO_S16_SCALE_RANGE PBL_BIT(15)

// FIFO tag byte: TAG_SENSOR in bits 7:3, TAG_CNT (time slot counter) in bits 2:1
#define LSM6DSO_FIFO_TAG_SENSOR(tag) ((uint8_t)((tag) >> 3U))
#define LSM6DSO_FIFO_TAG_CNT(tag)    ((uint8_t)(((tag) >> 1U) & 0x3U))
#define LSM6DSO_FIFO_TAG_GY_NC       0x01U
#define LSM6DSO_FIFO_TAG_XL_NC       0x02U
#define LSM6DSO_FIFO_TAG_TS          0x04U

// Nominal timestamp tick: 25 us; trimmed tick = 25 us / (1 + 0.0015 * INTERNAL_FREQ_FINE)
#define LSM6DSO_TS_TICK_US_NOMINAL 25.0
// Memory of the sensor-clock fit: long enough to average out I2C read jitter over ~60 FIFO reads,
// short enough to follow oscillator drift with temperature
#define LSM6DSO_TS_SYNC_MEMORY_S 10.0

// Gyroscope: +-2000 dps, 70 mdps/LSB -> 0.1 dps units = raw * 7 / 10
#define LSM6DSO_GYRO_SCALE_NUM 7
#define LSM6DSO_GYRO_SCALE_DEN 10

// Maximum FIFO watermark (WTM[8:0] is 9 bits)
#define LSM6DSO_FIFO_WTM_MAX PBL_BIT_MASK(9)

// Registers
#define LSM6DSO_FIFO_CTRL1        0x07U
#define LSM6DSO_FIFO_CTRL2        0x08U
#define LSM6DSO_FIFO_CTRL3        0x09U
#define LSM6DSO_FIFO_CTRL4        0x0AU
#define LSM6DSO_INT1_CTRL         0x0DU
#define LSM6DSO_WHO_AM_I          0x0FU
#define LSM6DSO_CTRL1_XL          0x10U
#define LSM6DSO_CTRL2_G           0x11U
#define LSM6DSO_CTRL10_C          0x19U
#define LSM6DSO_TIMESTAMP0        0x40U
#define LSM6DSO_INTERNAL_FREQ_FINE 0x63U
#define LSM6DSO_CTRL3_C           0x12U
#define LSM6DSO_CTRL5_C           0x14U
#define LSM6DSO_CTRL9_XL          0x18U
#define LSM6DSO_ALL_INT_SRC       0x1AU
#define LSM6DSO_WAKE_UP_SRC       0x1BU
#define LSM6DSO_STATUS_REG        0x1EU
#define LSM6DSO_OUTX_L_A          0x28U
#define LSM6DSO_FIFO_STATUS1      0x3AU
#define LSM6DSO_FIFO_STATUS2      0x3BU
#define LSM6DSO_TAP_CFG0          0x56U
#define LSM6DSO_TAP_CFG2          0x58U
#define LSM6DSO_WAKE_UP_THS       0x5BU
#define LSM6DSO_WAKE_UP_DUR       0x5CU
#define LSM6DSO_MD1_CFG           0x5EU
#define LSM6DSO_FIFO_DATA_OUT_TAG 0x78U

// WHO_AM_I value
#define LSM6DSO_WHO_AM_I_VAL 0x6CU

// CTRL1_XL fields
#define LSM6DSO_CTRL1_XL_ODR_MASK  PBL_GENMASK(7, 4)
#define LSM6DSO_CTRL1_XL_ODR_OFF   0x0U
#define LSM6DSO_CTRL1_XL_ODR_12HZ5 0x1U
#define LSM6DSO_CTRL1_XL_ODR_26HZ  0x2U
#define LSM6DSO_CTRL1_XL_ODR_52HZ  0x3U
#define LSM6DSO_CTRL1_XL_ODR_104HZ 0x4U
#define LSM6DSO_CTRL1_XL_ODR_208HZ 0x5U
#define LSM6DSO_CTRL1_XL_ODR_416HZ 0x6U
#define LSM6DSO_CTRL1_XL_ODR_833HZ 0x7U

// CTRL2_G fields (ODR codes as CTRL1_XL)
#define LSM6DSO_CTRL2_G_ODR_MASK PBL_GENMASK(7, 4)
#define LSM6DSO_CTRL2_G_FS_MASK  PBL_GENMASK(3, 2)
#define LSM6DSO_CTRL2_G_FS_2000  0x3U

// CTRL10_C fields
#define LSM6DSO_CTRL10_C_TIMESTAMP_EN PBL_BIT(5)
#define LSM6DSO_CTRL1_XL_FS_MASK   PBL_GENMASK(3, 2)
#define LSM6DSO_CTRL1_XL_FS_2G     0x0U
#define LSM6DSO_CTRL1_XL_FS_16G    0x1U
#define LSM6DSO_CTRL1_XL_FS_4G     0x2U
#define LSM6DSO_CTRL1_XL_FS_8G     0x3U

// CTRL3_C fields
#define LSM6DSO_CTRL3_C_SW_RESET PBL_BIT(0)
#define LSM6DSO_CTRL3_C_IF_INC   PBL_BIT(2)
#define LSM6DSO_CTRL3_C_BDU      PBL_BIT(6)

// CTRL5_C fields
#define LSM6DSO_CTRL5_C_XL_ULP_EN PBL_BIT(7)

// CTRL9_XL fields
#define LSM6DSO_CTRL9_XL_I3C_DISABLE PBL_BIT(1)

// INT1_CTRL fields
#define LSM6DSO_INT1_CTRL_DRDY_XL  PBL_BIT(0)
#define LSM6DSO_INT1_CTRL_FIFO_TH  PBL_BIT(3)
#define LSM6DSO_INT1_CTRL_FIFO_OVR PBL_BIT(4)

// WAKE_UP_SRC fields
#define LSM6DSO_WAKE_UP_SRC_WU_IA              PBL_BIT(3)
#define LSM6DSO_WAKE_UP_SRC_AXIS_WU(chip_axis) PBL_BIT(2U - (chip_axis))

// STATUS_REG fields
#define LSM6DSO_STATUS_REG_XLDA PBL_BIT(0)

// FIFO_CTRL2 fields
#define LSM6DSO_FIFO_CTRL2_WTM8 PBL_BIT(0)

// FIFO_CTRL3 fields
#define LSM6DSO_FIFO_CTRL3_BDR_XL_MASK  PBL_GENMASK(3, 0)
#define LSM6DSO_FIFO_CTRL3_BDR_XL_12HZ5 0x1U
#define LSM6DSO_FIFO_CTRL3_BDR_XL_26HZ  0x2U
#define LSM6DSO_FIFO_CTRL3_BDR_XL_52HZ  0x3U
#define LSM6DSO_FIFO_CTRL3_BDR_XL_104HZ 0x4U
#define LSM6DSO_FIFO_CTRL3_BDR_XL_208HZ 0x5U
#define LSM6DSO_FIFO_CTRL3_BDR_XL_416HZ 0x6U
#define LSM6DSO_FIFO_CTRL3_BDR_XL_833HZ 0x7U
#define LSM6DSO_FIFO_CTRL3_BDR_GY_MASK  PBL_GENMASK(7, 4)

// FIFO_CTRL4 fields
#define LSM6DSO_FIFO_CTRL4_MODE_MASK   PBL_GENMASK(2, 0)
#define LSM6DSO_FIFO_CTRL4_MODE_BYPASS 0x0U
#define LSM6DSO_FIFO_CTRL4_MODE_STREAM 0x6U
#define LSM6DSO_FIFO_CTRL4_DEC_TS_BATCH_MASK PBL_GENMASK(7, 6)
#define LSM6DSO_FIFO_CTRL4_DEC_TS_BATCH_1    0x1U

// FIFO_STATUS fields
#define LSM6DSO_FIFO_STATUS2_DIFF_HI_MASK PBL_GENMASK(1, 0)
#define LSM6DSO_FIFO_STATUS2_FIFO_OVR_IA  PBL_BIT(6)
#define LSM6DSO_FIFO_STATUS2_FIFO_WTM_IA  PBL_BIT(7)

// TAP_CFG0 fields (slope_fds left at 0 to select the slope filter for wake-up)
#define LSM6DSO_TAP_CFG0_LIR             PBL_BIT(0)
#define LSM6DSO_TAP_CFG0_INT_CLR_ON_READ PBL_BIT(6)

// TAP_CFG2 fields
#define LSM6DSO_TAP_CFG2_INTERRUPTS_ENABLE PBL_BIT(7)

// WAKE_UP_THS fields
#define LSM6DSO_WAKE_UP_THS_WK_THS_MASK PBL_GENMASK(5, 0)

// WAKE_UP_DUR fields
#define LSM6DSO_WAKE_UP_DUR_WAKE_DUR_MASK PBL_GENMASK(6, 5)

// MD1_CFG fields
#define LSM6DSO_MD1_CFG_INT1_WU PBL_BIT(5)

////////////////////////////////////////////////////////////////////////////////
// Private
////////////////////////////////////////////////////////////////////////////////

static bool prv_lsm6dso_write(uint8_t reg, const uint8_t *data, uint16_t len) {
  return pbl_i2c_burst_write(&s_dev->i2c, reg, data, len) == 0;
}

static bool prv_lsm6dso_read(uint8_t reg, uint8_t *data, uint16_t len) {
  return pbl_i2c_burst_read(&s_dev->i2c, reg, data, len) == 0;
}

static bool prv_lsm6dso_read_fifo(uint16_t samples) {
  uint8_t reg = LSM6DSO_FIFO_DATA_OUT_TAG;
  struct pbl_i2c_msg msgs[] = {
    {.buf = &reg, .len = 1U, .flags = PBL_I2C_MSG_WRITE},
    {.buf = s_dev->state->raw_sample_buf,
     .len = samples * LSM6DSO_FIFO_WORD_SIZE_BYTES,
     .flags = PBL_I2C_MSG_READ | PBL_I2C_MSG_DMA_SAFE},
  };

  return pbl_i2c_transfer(&s_dev->i2c, msgs, PBL_ARRAY_SIZE(msgs)) == 0;
}

static uint8_t prv_fs_bits(void) {
  switch (s_dev->state->range_mg) {
    case 2000U:
      return PBL_FIELD_PREP(LSM6DSO_CTRL1_XL_FS_MASK, LSM6DSO_CTRL1_XL_FS_2G);
    case 4000U:
      return PBL_FIELD_PREP(LSM6DSO_CTRL1_XL_FS_MASK, LSM6DSO_CTRL1_XL_FS_4G);
    case 8000U:
      return PBL_FIELD_PREP(LSM6DSO_CTRL1_XL_FS_MASK, LSM6DSO_CTRL1_XL_FS_8G);
    case 16000U:
      return PBL_FIELD_PREP(LSM6DSO_CTRL1_XL_FS_MASK, LSM6DSO_CTRL1_XL_FS_16G);
    default:
      return PBL_FIELD_PREP(LSM6DSO_CTRL1_XL_FS_MASK, LSM6DSO_CTRL1_XL_FS_2G);
  }
}

static int16_t prv_raw_to_s16(const uint8_t *raw) {
  return (int16_t)pbl_get_le16(raw);
}

static int16_t prv_axis_raw_mg(IMUCoordinateAxis axis, const uint8_t *raw) {
  uint8_t offset;
  int16_t val;

  offset = s_dev->axis_map[axis];

  val = s_dev->axis_dir[axis] * (int16_t)(((int32_t)prv_raw_to_s16(&raw[offset * 2U]) *
                                           (int32_t)s_dev->state->range_mg) /
                                          (int32_t)LSM6DSO_S16_SCALE_RANGE);

  if (s_dev->state->rotated && (axis == AXIS_X || axis == AXIS_Y)) {
    val *= -1;
  }

  return val;
}

static void prv_raw_to_mg(const uint8_t *raw, struct pbl_accel_sample *sample) {
  sample->x = prv_axis_raw_mg(AXIS_X, raw);
  sample->y = prv_axis_raw_mg(AXIS_Y, raw);
  sample->z = prv_axis_raw_mg(AXIS_Z, raw);
}

static uint64_t prv_get_curr_system_time_us(void) {
  return pbl_realtime_us();
}

static uint32_t prv_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) | ((uint32_t)p[2] << 16U) |
         ((uint32_t)p[3] << 24U);
}

static uint8_t prv_words_per_slot(void) {
  return s_dev->state->gyro_enabled ? 3U : 2U;  // [gyro] + accel + timestamp
}

//! Sensor ticks -> wall clock microseconds. The sensor counter runs on the LSM6DSO's own
//! oscillator; the clocksync fit maps it onto monotonic uptime (offset and rate, refitted at every
//! FIFO read), so a wall clock change (the phone setting the time) can't bend it.
static uint64_t prv_ticks_to_realtime_us(uint32_t ts) {
  int64_t us =
      (int64_t)pbl_clocksync_to_uptime_us(&s_dev->state->sync, ts) + pbl_realtime_offset_us();
  return (us > 0) ? (uint64_t)us : 0U;
}

//! Read the live timestamp counter between two uptime reads and feed the pair to the clock fit.
static void prv_read_ts_ref(void) {
  struct pbl_lsm6dso_state *st = s_dev->state;
  uint8_t raw[4];
  uint64_t before = pbl_uptime_us();
  bool ok = prv_lsm6dso_read(LSM6DSO_TIMESTAMP0, raw, sizeof(raw));
  uint64_t after = pbl_uptime_us();
  if (!ok) {
    PBL_LOG_ERR("Could not read TIMESTAMP registers");
    return;  // the fit keeps its previous pairs
  }
  st->ts_ref = prv_le32(raw);
  pbl_clocksync_add_pair(&st->sync, st->ts_ref, before, after);
}

//! Sensor ticks between samples at the current rate, to extrapolate a slot lacking a timestamp.
static uint32_t prv_ticks_per_sample(void) {
  return (uint32_t)((double)s_dev->state->sampling_interval_us /
                    pbl_clocksync_us_per_tick(&s_dev->state->sync));
}

static void prv_emit_slot(void) {
  struct pbl_lsm6dso_state *st = s_dev->state;
  if (st->slot.has_xl && st->out_n < PBL_ARRAY_SIZE(st->out_ts_us)) {
    uint32_t ts;
    if (st->slot.has_ts) {
      ts = st->slot.ts;
    } else if (st->ts_last_valid) {
      ts = st->ts_last + prv_ticks_per_sample();
    } else {
      ts = st->ts_ref;
    }
    st->ts_last = ts;
    st->ts_last_valid = true;
    memcpy(st->out_buf[st->out_n], st->slot.xl, 6);
    if (st->slot.has_g) {
      memcpy(&st->out_buf[st->out_n][6], st->slot.g, 6);
    } else {
      memset(&st->out_buf[st->out_n][6], 0, 6);
    }
    st->out_ts_us[st->out_n++] = prv_ticks_to_realtime_us(ts);
  }
  st->slot.valid = false;
}

//! Group the FIFO words read into time slots (TAG_CNT) and collect complete samples in out_buf.
static void prv_lsm6dso_parse_fifo(uint16_t words) {
  struct pbl_lsm6dso_state *st = s_dev->state;
  st->out_n = 0;
  for (uint16_t i = 0U; i < words; i++) {
    const uint8_t *w = &st->raw_sample_buf[i * LSM6DSO_FIFO_WORD_SIZE_BYTES];
    uint8_t sensor = LSM6DSO_FIFO_TAG_SENSOR(w[0]);
    uint8_t cnt = LSM6DSO_FIFO_TAG_CNT(w[0]);
    if (st->slot.valid && cnt != st->slot.cnt) {
      prv_emit_slot();
    }
    if (!st->slot.valid) {
      st->slot.valid = true;
      st->slot.cnt = cnt;
      st->slot.has_ts = st->slot.has_xl = st->slot.has_g = false;
    }
    switch (sensor) {
      case LSM6DSO_FIFO_TAG_XL_NC:
        memcpy(st->slot.xl, &w[1], 6);
        st->slot.has_xl = true;
        break;
      case LSM6DSO_FIFO_TAG_GY_NC:
        memcpy(st->slot.g, &w[1], 6);
        st->slot.has_g = true;
        break;
      case LSM6DSO_FIFO_TAG_TS:
        st->slot.ts = prv_le32(&w[1]);
        st->slot.has_ts = true;
        break;
      default:  // configuration-change / temperature words
        break;
    }
  }
  // A complete last slot goes out now; a partial one waits for its remaining words
  if (st->slot.valid && st->slot.has_xl && st->slot.has_ts &&
      (st->slot.has_g || !st->gyro_enabled)) {
    prv_emit_slot();
  }
}

//! Forget any partial slot (the FIFO was flushed or reconfigured).
static void prv_reset_slots(void) {
  s_dev->state->slot.valid = false;
  s_dev->state->ts_last_valid = false;
  // Also restart the timestamp fit: while the stream was off the counter may have stopped, and
  // the oscillator runs at a slightly different rate in ULP and high-performance (gyroscope) mode
  pbl_clocksync_restart(&s_dev->state->sync);
}

// Dispatch the samples parsed into out_buf, each with its own hardware timestamp.
static void prv_lsm6dso_process_samples(void) {
  struct pbl_lsm6dso_state *st = s_dev->state;
  if (st->out_n == 0U) {
    return;
  }
  int8_t rotate = st->rotated ? -1 : 1;
  AccelRawBatch batch = {
    .data = st->out_buf[0],
    .num_samples = st->out_n,
    .stride = LSM6DSO_OUT_SAMPLE_BYTES,
    .axis =
        {
          [AXIS_X] =
              {.offset = s_dev->axis_map[AXIS_X] * 2U,
               .sign = (int8_t)(s_dev->axis_dir[AXIS_X] * rotate)},
          [AXIS_Y] =
              {.offset = s_dev->axis_map[AXIS_Y] * 2U,
               .sign = (int8_t)(s_dev->axis_dir[AXIS_Y] * rotate)},
          [AXIS_Z] = {.offset = s_dev->axis_map[AXIS_Z] * 2U, .sign = s_dev->axis_dir[AXIS_Z]},
        },
    .scale_num = st->range_mg,
    .scale_den = LSM6DSO_S16_SCALE_RANGE,
    .first_timestamp_us = st->out_ts_us[0],
    .sampling_interval_us = st->sampling_interval_us,
    .timestamps_us = st->out_ts_us,
  };
  if (st->gyro_enabled) {
    for (IMUCoordinateAxis a = AXIS_X; a <= AXIS_Z; a++) {
      batch.gyro_axis[a].offset = 6U + s_dev->axis_map[a] * 2U;
      batch.gyro_axis[a].sign = batch.axis[a].sign;
    }
    batch.gyro_scale_num = LSM6DSO_GYRO_SCALE_NUM;
    batch.gyro_scale_den = LSM6DSO_GYRO_SCALE_DEN;
  }

  pbl_accel_report_samples(&s_dev->accel, &batch);
}

static uint8_t prv_get_bdr(uint32_t sampling_interval_us) {
  if (sampling_interval_us > 0UL && sampling_interval_us <= 1200UL) {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_833HZ;
  } else if (sampling_interval_us > 0UL && sampling_interval_us <= 2403UL) {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_416HZ;
  } else if (sampling_interval_us >= 80000UL) {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_12HZ5;
  } else if (sampling_interval_us >= 38461UL) {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_26HZ;
  } else if (sampling_interval_us >= 19230UL) {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_52HZ;
  } else if (sampling_interval_us >= 9615UL) {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_104HZ;
  } else {
    return LSM6DSO_FIFO_CTRL3_BDR_XL_208HZ;
  }
}

static bool prv_lsm6dso_enable_fifo(uint16_t num_samples) {
  bool ret;
  uint8_t val;
  uint16_t wtm;

  // Bypass mode to flush the FIFO
  val = PBL_FIELD_PREP(LSM6DSO_FIFO_CTRL4_MODE_MASK, LSM6DSO_FIFO_CTRL4_MODE_BYPASS);
  ret = prv_lsm6dso_write(LSM6DSO_FIFO_CTRL4, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL4 register");
    return ret;
  }

  prv_reset_slots();

  // Watermark threshold WTM[8:0] counts FIFO words: every time slot carries an accelerometer word,
  // a timestamp word and, with the gyroscope on, a gyroscope word
  wtm = PBL_MIN((uint32_t)num_samples * prv_words_per_slot(), LSM6DSO_FIFO_WTM_MAX);

  val = PBL_FIELD_GET(PBL_GENMASK(7, 0), wtm);
  ret = prv_lsm6dso_write(LSM6DSO_FIFO_CTRL1, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL1 register");
    return ret;
  }

  val = PBL_FIELD_PREP(LSM6DSO_FIFO_CTRL2_WTM8, PBL_FIELD_GET(PBL_BIT(8), wtm));
  ret = prv_lsm6dso_write(LSM6DSO_FIFO_CTRL2, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL2 register");
    return ret;
  }

  // Accelerometer (and gyroscope) batch data rate
  val = PBL_FIELD_PREP(LSM6DSO_FIFO_CTRL3_BDR_XL_MASK,
                       prv_get_bdr(s_dev->state->sampling_interval_us));
  if (s_dev->state->gyro_enabled) {
    val |= PBL_FIELD_PREP(LSM6DSO_FIFO_CTRL3_BDR_GY_MASK,
                          prv_get_bdr(s_dev->state->sampling_interval_us));
  }
  ret = prv_lsm6dso_write(LSM6DSO_FIFO_CTRL3, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL3 register");
    return ret;
  }

  // Continuous (stream) mode, with a timestamp batched in every time slot
  val = PBL_FIELD_PREP(LSM6DSO_FIFO_CTRL4_MODE_MASK, LSM6DSO_FIFO_CTRL4_MODE_STREAM) |
        PBL_FIELD_PREP(LSM6DSO_FIFO_CTRL4_DEC_TS_BATCH_MASK, LSM6DSO_FIFO_CTRL4_DEC_TS_BATCH_1);
  ret = prv_lsm6dso_write(LSM6DSO_FIFO_CTRL4, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write FIFO_CTRL4 register");
    return ret;
  }

  PBL_LOG_DBG("FIFO enabled with threshold %" PRIu16, wtm);

  return true;
}

//! Salvage and dispatch any samples queued in the FIFO
static void prv_lsm6dso_drain_fifo(void) {
  for (unsigned int i = 0U; i < LSM6DSO_DRAIN_CHUNKS_MAX; i++) {
    uint8_t status[2];
    uint16_t samples;

    if (!prv_lsm6dso_read(LSM6DSO_FIFO_STATUS1, status, 2)) {
      PBL_LOG_ERR("Could not read FIFO_STATUS registers");
      return;
    }

    samples = (PBL_FIELD_GET(LSM6DSO_FIFO_STATUS2_DIFF_HI_MASK, status[1]) << 8U) | status[0];
    samples = PBL_MIN(samples, LSM6DSO_FIFO_SIZE);
    if (samples == 0U) {
      return;
    }

    if (!prv_lsm6dso_read_fifo(samples)) {
      PBL_LOG_ERR("Failed to read samples");
      return;
    }

    prv_read_ts_ref();
    prv_lsm6dso_parse_fifo(samples);
    prv_lsm6dso_process_samples();
    s_dev->state->last_fifo_read_tick = pbl_uptime_ticks();
  }
}

static void prv_lsm6dso_recover(void);
static uint32_t prv_ms_since_last_fifo_read(void);

static void prv_lsm6dso_dispatch_shake(uint8_t wake_up_src) {
  uint8_t raw[LSM6DSO_SAMPLE_SIZE_BYTES] = {0U};
  IMUCoordinateAxis axis = AXIS_Z;
  int16_t val = 0;
  bool found = false;

  if (!prv_lsm6dso_read(LSM6DSO_OUTX_L_A, raw, sizeof(raw))) {
    PBL_LOG_ERR("Failed to read sample");
  }

  for (IMUCoordinateAxis a = AXIS_X; a <= AXIS_Z; a++) {
    int16_t v;

    if ((wake_up_src & LSM6DSO_WAKE_UP_SRC_AXIS_WU(s_dev->axis_map[a])) == 0U) {
      continue;
    }

    v = prv_axis_raw_mg(a, raw);
    if (!found || PBL_ABS(v) > PBL_ABS(val)) {
      axis = a;
      val = v;
      found = true;
    }
  }

  pbl_accel_report_event(&s_dev->accel, &(const struct pbl_accel_event){
                                          .type = PBL_ACCEL_EVENT_SHAKE,
                                          .axis = axis,
                                          .direction = (val < 0) ? -1 : 1,
                                        });
}

//! INT1 servicing pass kind, logged by the no-action diagnostic
enum int1_pass {
  INT1_PASS_INITIAL = 0,
  INT1_PASS_REQUEUE = 1,
  INT1_PASS_RETRY = 2,
};

//! Single INT1 servicing pass; returns true if any INT source was handled.
//! fifo_progress is set when FIFO data was consumed (samples or overrun).
static bool prv_lsm6dso_service_int1(enum int1_pass pass, bool *fifo_progress) {
  bool ret;
  uint8_t fifo_status[2] = {0U, 0U};
  uint8_t wake_up_src = 0U;
  bool action_taken = false;
  bool fifo_overrun = false;
  bool shake = false;
  uint16_t samples = 0U;

  // Read all device registers back-to-back to keep the I2C critical section
  // short, then convert/dispatch below. The sample processing has no dependency
  // on these reads, so deferring it avoids stretching the gap between the FIFO
  // read and the WAKE_UP_SRC read if this task gets preempted mid-handler.
  if (s_dev->state->num_samples > 0U) {
    // WTM/OVR flags and DIFF must come from one burst read: a split read can
    // catch the FIFO counter mid-update and see WTM_IA set with DIFF reading 0
    ret = prv_lsm6dso_read(LSM6DSO_FIFO_STATUS1, fifo_status, 2);
    if (!ret) {
      PBL_LOG_ERR("Could not read FIFO_STATUS registers");
      return false;
    }

    if ((fifo_status[1] & LSM6DSO_FIFO_STATUS2_FIFO_OVR_IA) != 0U) {
      fifo_overrun = true;
    } else if ((fifo_status[1] & LSM6DSO_FIFO_STATUS2_FIFO_WTM_IA) != 0U) {
      samples =
          (PBL_FIELD_GET(LSM6DSO_FIFO_STATUS2_DIFF_HI_MASK, fifo_status[1]) << 8U) | fifo_status[0];
      if (samples > LSM6DSO_FIFO_SIZE) {
        samples = LSM6DSO_FIFO_SIZE;
      }

      if (samples > 0U) {
        if (!prv_lsm6dso_read_fifo(samples)) {
          PBL_LOG_ERR("Failed to read samples");
          return false;
        }
        prv_read_ts_ref();
        s_dev->state->last_fifo_read_tick = pbl_uptime_ticks();
      }
    }
  }

  if (s_dev->state->shake_detection_enabled) {
    ret = prv_lsm6dso_read(LSM6DSO_WAKE_UP_SRC, &wake_up_src, 1);
    if (!ret) {
      PBL_LOG_ERR("Could not read WAKE_UP_SRC register");
      return false;
    }

    shake = (wake_up_src & LSM6DSO_WAKE_UP_SRC_WU_IA) != 0U;
  }

  if (fifo_overrun) {
    PBL_LOG_WRN("FIFO overrun detected, recovering");
    prv_reset_slots();
    prv_lsm6dso_recover();
    action_taken = true;
    *fifo_progress = true;
  } else if (samples > 0U) {
    prv_lsm6dso_parse_fifo(samples);
    prv_lsm6dso_process_samples();
    action_taken = true;
    *fifo_progress = true;
  }

  if (shake) {
    // WU_IA stays set while the wake-up condition persists (LIR), so only
    // dispatch on its assertion to avoid flooding events
    if (!s_dev->state->wu_active) {
      prv_lsm6dso_dispatch_shake(wake_up_src);
    }
    action_taken = true;
  }
  s_dev->state->wu_active = shake;

  if (!action_taken) {
    // Registers not read this pass (gated on num_samples/shake state) log as 0
    PBL_LOG_WRN("INT1 triggered but no action taken (FIFO_STATUS2 0x%02" PRIx8
                " WAKE_UP_SRC 0x%02" PRIx8 " num_samples %" PRIu16
                " shake_en %d pass %u"
                " last_read %" PRIu32 "ms ago)",
                fifo_status[1], wake_up_src, s_dev->state->num_samples,
                s_dev->state->shake_detection_enabled, (unsigned int)pass,
                prv_ms_since_last_fifo_read());
  }

  return action_taken;
}

static void prv_work_cb(void *data) {
  void (*work)(void) = data;

  pbl_accel_lock(&s_dev->accel);
  work();
  pbl_accel_unlock(&s_dev->accel);
}

//! Run driver work on the timer task, with the device lock held
static void prv_queue_work(void (*work)(void)) {
  pbl_sys_work_call_high(prv_work_cb, (void *)work);
}

static void prv_queue_work_from_isr(void (*work)(void)) {
  pbl_sys_work_call_high_from_isr(prv_work_cb, (void *)work);
}

static void prv_lsm6dso_int1_work_handler(void) {
  bool fifo_progress = false;
  // Best-effort pass tagging: an ISR enqueue racing a pending requeue gets
  // labeled as a requeue; diagnostic only
  enum int1_pass pass = s_dev->state->int1_requeued ? INT1_PASS_REQUEUE : INT1_PASS_INITIAL;
  bool action_taken;

  s_dev->state->int1_requeued = false;
  action_taken = prv_lsm6dso_service_int1(pass, &fifo_progress);

  // Sources asserting while the pad is high produce no new edge: requeue on
  // FIFO progress, recover when nothing was serviced (stuck pad). A pad held
  // by a persistent wake-up condition is left to the stall watchdog.
  if (!pbl_gpio_get(&s_dev->int1_in)) {
    return;
  }

  if (fifo_progress) {
    s_dev->state->int1_requeued = true;
    prv_queue_work(prv_lsm6dso_int1_work_handler);
    return;
  }

  if (action_taken) {
    return;
  }

  // A source asserting between the status reads and the pad re-check is
  // indistinguishable from a stuck pad, and recovering would discard it.
  // Retry the servicing pass once (a latched wake-up or a fresh FIFO
  // threshold is visible on re-read) and only recover if the pad is still
  // high with nothing serviced.
  action_taken = prv_lsm6dso_service_int1(INT1_PASS_RETRY, &fifo_progress);
  if (fifo_progress) {
    s_dev->state->int1_requeued = true;
    prv_queue_work(prv_lsm6dso_int1_work_handler);
  } else if (!action_taken && pbl_gpio_get(&s_dev->int1_in)) {
    prv_lsm6dso_recover();
  }
}

static void prv_lsm6dso_int1_irq_handler(void *arg) {
  s_dev->state->num_int1_irqs++;
  // A rising edge proves the pad was low, i.e. the wake-up source deasserted
  s_dev->state->wu_active = false;
  // ... which also breaks any stuck-high streak the watchdog counted
  s_dev->state->shake_stuck_passes = 0U;
  prv_queue_work_from_isr(prv_lsm6dso_int1_work_handler);
}

static bool prv_configure_odr(uint32_t sampling_interval_us, bool shake_detection_enabled) {
  uint8_t odr;
  uint8_t val;
  bool ret;

  // If shake detection is enabled, ensure a minimum ODR of 12.5Hz (80ms)
  if (shake_detection_enabled && (sampling_interval_us == 0UL)) {
    sampling_interval_us = 80000UL;
  }

  if (sampling_interval_us == 0U) {
    odr = LSM6DSO_CTRL1_XL_ODR_OFF;
    sampling_interval_us = 0UL;
  } else if (sampling_interval_us >= 80000UL) {
    odr = LSM6DSO_CTRL1_XL_ODR_12HZ5;
    sampling_interval_us = 80000UL;
  } else if (sampling_interval_us >= 38461UL) {
    odr = LSM6DSO_CTRL1_XL_ODR_26HZ;
    sampling_interval_us = 38461UL;
  } else if (sampling_interval_us >= 19230UL) {
    odr = LSM6DSO_CTRL1_XL_ODR_52HZ;
    sampling_interval_us = 19230UL;
  } else if (sampling_interval_us >= 9615UL) {
    odr = LSM6DSO_CTRL1_XL_ODR_104HZ;
    sampling_interval_us = 9615UL;
  } else if (sampling_interval_us >= 4807UL || !s_dev->state->gyro_enabled) {
    // Ultra-low-power mode tops out at 208 Hz
    odr = LSM6DSO_CTRL1_XL_ODR_208HZ;
    sampling_interval_us = 4807UL;
  } else if (sampling_interval_us >= 2403UL) {
    odr = LSM6DSO_CTRL1_XL_ODR_416HZ;
    sampling_interval_us = 2403UL;
  } else {
    odr = LSM6DSO_CTRL1_XL_ODR_833HZ;
    sampling_interval_us = 1200UL;
  }

  val = PBL_FIELD_PREP(LSM6DSO_CTRL1_XL_ODR_MASK, odr) | prv_fs_bits();

  PBL_LOG_DBG("Configuring ODR to %" PRIu32 " ms (%" PRIu32 " mHz)",
              (uint32_t)(sampling_interval_us / 1000UL),
              (uint32_t)(sampling_interval_us > 0UL ? 1000000000UL / sampling_interval_us : 0UL));

  ret = prv_lsm6dso_write(LSM6DSO_CTRL1_XL, &val, 1);
  if (!ret) {
    return ret;
  }

  // Gyroscope at the same ODR, or powered down
  val = (s_dev->state->gyro_enabled && odr != LSM6DSO_CTRL1_XL_ODR_OFF)
            ? (PBL_FIELD_PREP(LSM6DSO_CTRL2_G_ODR_MASK, odr) |
               PBL_FIELD_PREP(LSM6DSO_CTRL2_G_FS_MASK, LSM6DSO_CTRL2_G_FS_2000))
            : 0U;
  ret = prv_lsm6dso_write(LSM6DSO_CTRL2_G, &val, 1);
  if (!ret) {
    return ret;
  }

  // Allow the accelerometer to stabilize after an ODR/power-mode change
  if (odr != LSM6DSO_CTRL1_XL_ODR_OFF) {
    pbl_sleep_ms(10);
  }

  s_dev->state->sampling_interval_us = sampling_interval_us;

  return true;
}

static bool prv_configure_int1(bool shake_detection_enabled, bool fifo_enabled) {
  bool ret;
  uint8_t int1_ctrl;
  uint8_t md1_cfg;
  uint8_t tap_cfg2;

  int1_ctrl = 0U;
  md1_cfg = 0U;

  if (fifo_enabled) {
    int1_ctrl |= LSM6DSO_INT1_CTRL_FIFO_TH | LSM6DSO_INT1_CTRL_FIFO_OVR;
  }

  if (shake_detection_enabled) {
    md1_cfg |= LSM6DSO_MD1_CFG_INT1_WU;
  }

  ret = prv_lsm6dso_write(LSM6DSO_INT1_CTRL, &int1_ctrl, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write INT1_CTRL register");
    return ret;
  }

  PBL_LOG_DBG("INT1_CTRL configured: %02" PRIx8, int1_ctrl);

  // Basic interrupt functions must be enabled for the wake-up event routing
  tap_cfg2 = shake_detection_enabled ? LSM6DSO_TAP_CFG2_INTERRUPTS_ENABLE : 0U;
  ret = prv_lsm6dso_write(LSM6DSO_TAP_CFG2, &tap_cfg2, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write TAP_CFG2 register");
    return ret;
  }

  ret = prv_lsm6dso_write(LSM6DSO_MD1_CFG, &md1_cfg, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write MD1_CFG register");
    return ret;
  }

  PBL_LOG_DBG("MD1_CFG configured: %02" PRIx8, md1_cfg);

  return true;
}

//! Recover a dead INT1/FIFO stream by re-asserting ODR, FIFO and INT routing.
//! Routing is quiesced first so a latched-high pad produces a fresh edge.
static void prv_lsm6dso_recover(void) {
  uint8_t val;

  s_dev->state->num_recoveries++;
  PBL_ANALYTICS_ADD(accel_stream_recovery_count, 1);
  PBL_LOG_WRN("Recovering accel stream (count %" PRIu32 ")", s_dev->state->num_recoveries);

  if (!prv_configure_int1(false, false)) {
    return;
  }

  // Salvage queued samples
  if (s_dev->state->num_samples > 0U) {
    prv_lsm6dso_drain_fifo();
  }

  // Clear any latched function INT source while routing is quiesced
  if (!prv_lsm6dso_read(LSM6DSO_ALL_INT_SRC, &val, 1)) {
    PBL_LOG_ERR("Could not read ALL_INT_SRC register");
    return;
  }

  if (val != 0U) {
    PBL_LOG_WRN("Discarded latched INT sources during recovery (ALL_INT_SRC 0x%02" PRIx8 ")", val);
  }

  if (!prv_configure_odr(s_dev->state->sampling_interval_us,
                         s_dev->state->shake_detection_enabled)) {
    PBL_LOG_ERR("Could not configure ODR");
    return;
  }

  if (s_dev->state->num_samples > 0U) {
    if (!prv_lsm6dso_enable_fifo(s_dev->state->num_samples)) {
      return;
    }
  }

  if (!prv_configure_int1(s_dev->state->shake_detection_enabled, s_dev->state->num_samples > 0U)) {
    return;
  }

  s_dev->state->wu_active = false;
  s_dev->state->last_fifo_read_tick = pbl_uptime_ticks();
}

static uint32_t prv_ms_since_last_fifo_read(void) {
  uint64_t ticks = pbl_uptime_ticks() - s_dev->state->last_fifo_read_tick;
  return (uint32_t)((ticks * 1000) / PBL_TICK_HZ);
}

static uint32_t prv_stall_threshold_ms(void) {
  return PBL_MAX(LSM6DSO_STALL_MARGIN * s_dev->state->int1_period_ms, LSM6DSO_STALL_MIN_MS);
}

//! Arm/disarm the INT1 stall watchdog to match the active INT1 sources.
//! Call after num_samples or shake_detection_enabled changes.
static void prv_update_int1_watchdog(void) {
  pbl_wallclock_work_stop(&s_dev->state->int1_wdt_timer);
  s_dev->state->shake_stuck_passes = 0U;

  if (s_dev->state->num_samples > 0U) {
    s_dev->state->last_fifo_read_tick = pbl_uptime_ticks();
    s_dev->state->int1_period_ms =
        (s_dev->state->sampling_interval_us * s_dev->state->num_samples) / 1000;
    pbl_wallclock_work_start(&pbl_sys_workq_high, &s_dev->state->int1_wdt_timer, 1,
                             PBL_MAX(PBL_DIV_ROUND_UP(s_dev->state->int1_period_ms, 1000UL), 1U));
  } else if (s_dev->state->shake_detection_enabled) {
    pbl_wallclock_work_start(&pbl_sys_workq_high, &s_dev->state->int1_wdt_timer, 1,
                             LSM6DSO_SHAKE_WDT_PERIOD_S);
  }
}

//! Shared by the INT1 watchdog and the peek staleness trigger;
//! re-validates the stall at execution time.
static void prv_stall_check_work_cb(void) {
  uint32_t ms_since_last_read;
  uint8_t fifo_status[2] = {0U, 0U};

  s_dev->state->recovery_pending = false;

  // Sampling may have stopped between scheduling and execution
  if (s_dev->state->num_samples == 0U) {
    // Shake-only mode: re-run the servicing pass if the pad is stuck high
    // (reading the INT source clears the latch); escalate to a full recovery
    // after consecutive passes that never release it.
    if (s_dev->state->shake_detection_enabled && pbl_gpio_get(&s_dev->int1_in)) {
      prv_lsm6dso_int1_work_handler();
      // A pad released by the pass is healthy; count only a still-high pad
      if (!pbl_gpio_get(&s_dev->int1_in)) {
        s_dev->state->shake_stuck_passes = 0U;
      } else if (++s_dev->state->shake_stuck_passes >= LSM6DSO_SHAKE_STUCK_PASSES_MAX) {
        PBL_LOG_WRN("INT1 pad stuck high for %" PRIu8 " shake watchdog passes, recovering",
                    s_dev->state->shake_stuck_passes);
        prv_lsm6dso_recover();
        s_dev->state->shake_stuck_passes = 0U;
      }
    } else {
      s_dev->state->shake_stuck_passes = 0U;
    }
    return;
  }

  ms_since_last_read = prv_ms_since_last_fifo_read();
  if (ms_since_last_read < prv_stall_threshold_ms()) {
    return;
  }

  // Tells a lost edge (pad high) from an IMU not raising INT1 (pad low)
  if (!prv_lsm6dso_read(LSM6DSO_FIFO_STATUS1, fifo_status, 2)) {
    PBL_LOG_ERR("Could not read FIFO_STATUS registers");
  }

  PBL_LOG_WRN("FIFO stream stalled for %" PRIu32 " ms (INT1 pad %d irqs %" PRIu32
              " FIFO_STATUS1 0x%02" PRIx8 " FIFO_STATUS2 0x%02" PRIx8 ")",
              ms_since_last_read, pbl_gpio_get(&s_dev->int1_in), s_dev->state->num_int1_irqs,
              fifo_status[0], fifo_status[1]);
  prv_lsm6dso_recover();
}

static void prv_int1_wdt_cb(struct pbl_work *work) {
  prv_queue_work(prv_stall_check_work_cb);
}

////////////////////////////////////////////////////////////////////////////////
// Accelerometer interface
////////////////////////////////////////////////////////////////////////////////

static int prv_init(const struct pbl_accel *accel) {
  bool ret;
  int err;
  uint8_t val;

  s_dev = PBL_CONTAINER_OF(accel, const struct pbl_lsm6dso, accel);

  // Check device ID
  ret = prv_lsm6dso_read(LSM6DSO_WHO_AM_I, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not read WHO_AM_I register");
    return -ENODEV;
  }

  if (val != LSM6DSO_WHO_AM_I_VAL) {
    PBL_LOG_ERR("Unexpected id: 0x%02X!=0x%02X", val, LSM6DSO_WHO_AM_I_VAL);
    return -ENODEV;
  }

  // Perform a software reset (so we can rely on defaults)
  val = LSM6DSO_CTRL3_C_SW_RESET;
  ret = prv_lsm6dso_write(LSM6DSO_CTRL3_C, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL3_C register");
    return -EIO;
  }

  pbl_busy_wait_us(LSM6DSO_RESET_TIME_US);

  do {
    ret = prv_lsm6dso_read(LSM6DSO_CTRL3_C, &val, 1);
    if (!ret) {
      PBL_LOG_ERR("Could not read CTRL3_C register");
      return -EIO;
    }
  } while ((val & LSM6DSO_CTRL3_C_SW_RESET) != 0U);

  // Enable register auto-increment and block data update
  val = LSM6DSO_CTRL3_C_IF_INC | LSM6DSO_CTRL3_C_BDU;
  ret = prv_lsm6dso_write(LSM6DSO_CTRL3_C, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL3_C register");
    return -EIO;
  }

  // Disable I3C interface
  val = LSM6DSO_CTRL9_XL_I3C_DISABLE;
  ret = prv_lsm6dso_write(LSM6DSO_CTRL9_XL, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL9_XL register");
    return -EIO;
  }

  // Ultra-low-power mode (accelerometer still powered down here, as required to
  // toggle XL_ULP_EN). Gyroscope stays in power-down, so ULP is allowed.
  val = LSM6DSO_CTRL5_C_XL_ULP_EN;
  ret = prv_lsm6dso_write(LSM6DSO_CTRL5_C, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL5_C register");
    return -EIO;
  }

  // Hardware timestamp counter, batched into the FIFO with every sample
  val = LSM6DSO_CTRL10_C_TIMESTAMP_EN;
  ret = prv_lsm6dso_write(LSM6DSO_CTRL10_C, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL10_C register");
    return -EIO;
  }

  // Factory oscillator trim: tick = 25 us / (1 + 0.0015 * INTERNAL_FREQ_FINE)
  ret = prv_lsm6dso_read(LSM6DSO_INTERNAL_FREQ_FINE, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not read INTERNAL_FREQ_FINE register");
    return -EIO;
  }
  double tick_us = LSM6DSO_TS_TICK_US_NOMINAL * 10000.0 / (double)(10000 + 15 * (int8_t)val);
  pbl_clocksync_init(&s_dev->state->sync, tick_us, 32U, LSM6DSO_TS_SYNC_MEMORY_S);
  PBL_LOG_INFO("Timestamp tick %" PRIu32 " ns (freq_fine %d)", (uint32_t)(tick_us * 1000.0),
               (int8_t)val);

  // Configure scale (ODR off until sampling is requested)
  s_dev->state->range_mg = CONFIG_ACCEL_LSM6DSO_SCALE_MG;
  val = PBL_FIELD_PREP(LSM6DSO_CTRL1_XL_ODR_MASK, LSM6DSO_CTRL1_XL_ODR_OFF) | prv_fs_bits();
  ret = prv_lsm6dso_write(LSM6DSO_CTRL1_XL, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL1_XL register");
    return -EIO;
  }

  // Slope filter for wake-up, latch interrupts and clear them on read
  val = LSM6DSO_TAP_CFG0_LIR | LSM6DSO_TAP_CFG0_INT_CLR_ON_READ;
  ret = prv_lsm6dso_write(LSM6DSO_TAP_CFG0, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write TAP_CFG0 register");
    return -EIO;
  }

  // Configure wake-up threshold defaults
  val = PBL_FIELD_PREP(LSM6DSO_WAKE_UP_DUR_WAKE_DUR_MASK, CONFIG_ACCEL_LSM6DSO_WK_DUR_DEFAULT);
  ret = prv_lsm6dso_write(LSM6DSO_WAKE_UP_DUR, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write WAKE_UP_DUR register");
    return -EIO;
  }

  val = PBL_FIELD_PREP(LSM6DSO_WAKE_UP_THS_WK_THS_MASK, CONFIG_ACCEL_LSM6DSO_WK_THS_DEFAULT);
  ret = prv_lsm6dso_write(LSM6DSO_WAKE_UP_THS, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write WAKE_UP_THS register");
    return -EIO;
  }

  s_dev->state->wk_ths_curr = CONFIG_ACCEL_LSM6DSO_WK_THS_DEFAULT;

  err = pbl_device_init(&s_dev->int1.intc->dev);
  if (err != 0) {
    PBL_LOG_ERR("INT1 interrupt controller unavailable");
    return err;
  }

  err = pbl_gpio_configure(&s_dev->int1_in, PBL_GPIO_INPUT);
  if (err != 0) {
    PBL_LOG_ERR("Could not configure the INT1 input (%d)", err);
    return err;
  }

  err = pbl_intc_request(&s_dev->int1, prv_lsm6dso_int1_irq_handler, nullptr);
  if (err != 0) {
    PBL_LOG_ERR("Could not request INT1 (%d)", err);
    return err;
  }

  err = pbl_intc_enable(&s_dev->int1);
  if (err != 0) {
    PBL_LOG_ERR("Could not enable INT1 (%d)", err);
    return err;
  }

  pbl_wallclock_work_init(&s_dev->state->int1_wdt_timer, prv_int1_wdt_cb);

  return 0;
}

static int prv_set_sampling_interval(const struct pbl_accel *accel, uint32_t interval_us,
                                     uint32_t *actual_us) {
  // FIXME: we should technically stop and drain the FIFO here, otherwise
  // we may report existing samples in the FIFO buffer with an incorrect timestamp

  const uint32_t prev_interval_us = s_dev->state->sampling_interval_us;
  if (!prv_configure_odr(interval_us, s_dev->state->shake_detection_enabled)) {
    PBL_LOG_ERR("Could not configure ODR");
    return -EIO;
  }

  // Re-arm the watchdog only on an actual ODR change (a re-arm resets the
  // stall tracking, so repeat calls must not defer it)
  if (s_dev->state->sampling_interval_us != prev_interval_us) {
    prv_update_int1_watchdog();
  }

  PBL_LOG_DBG("Set sampling interval to %" PRIu32 " us", s_dev->state->sampling_interval_us);

  *actual_us = s_dev->state->sampling_interval_us;

  return 0;
}

static uint32_t prv_get_sampling_interval(const struct pbl_accel *accel) {
  return s_dev->state->sampling_interval_us;
}

static uint32_t prv_get_max_num_samples(const struct pbl_accel *accel) {
  return LSM6DSO_FIFO_THRESHOLD;
}

static int prv_set_num_samples(const struct pbl_accel *accel, uint32_t num_samples) {
  bool ret;
  uint8_t val;

  // Limit to FIFO threshold
  if (num_samples > LSM6DSO_FIFO_THRESHOLD) {
    num_samples = LSM6DSO_FIFO_THRESHOLD;
  }

  // Disable all INT1 before changing FIFO threshold
  prv_configure_int1(false, false);

  // Config change invalidates any queued stall check; re-arm the peek trigger
  s_dev->state->recovery_pending = false;

  // Salvage queued samples
  if (s_dev->state->num_samples > 0U) {
    prv_lsm6dso_drain_fifo();
  }

  if (num_samples == 0U) {
    // Bypass FIFO (disable)
    val = PBL_FIELD_PREP(LSM6DSO_FIFO_CTRL4_MODE_MASK, LSM6DSO_FIFO_CTRL4_MODE_BYPASS);
    if (!prv_lsm6dso_write(LSM6DSO_FIFO_CTRL4, &val, 1)) {
      PBL_LOG_ERR("Could not write FIFO_CTRL4 register");
    }
  } else {
    // Configure FIFO in continuous mode with threshold
    ret = prv_lsm6dso_enable_fifo((uint16_t)num_samples);
    if (!ret) {
      PBL_LOG_ERR("Could not enable FIFO");
      return -EIO;
    }
  }

  // Re-configure INT1
  ret = prv_configure_int1(s_dev->state->shake_detection_enabled, num_samples > 0U);
  if (!ret) {
    PBL_LOG_ERR("Could not configure INT1");
    return -EIO;
  }

  s_dev->state->num_samples = num_samples;
  // Arm/disarm the INT1 liveness watchdog for the new source configuration
  prv_update_int1_watchdog();

  PBL_LOG_DBG("Set number of samples to %" PRIu32, num_samples);

  return 0;
}

//! True once a sample is ready, or when the status read fails.
static bool prv_data_ready(bool *read_ok) {
  uint8_t status;
  *read_ok = prv_lsm6dso_read(LSM6DSO_STATUS_REG, &status, 1);
  return !*read_ok || (status & LSM6DSO_STATUS_REG_XLDA) != 0U;
}

static int prv_peek(const struct pbl_accel *accel, struct pbl_accel_sample *data) {
  int err = 0;
  bool ret;
  uint8_t ctrl1;
  uint8_t ctrl1_bck;
  uint8_t raw[LSM6DSO_SAMPLE_SIZE_BYTES];

  // If sampling is active, the FIFO batches samples for subscribers, so the
  // cached sample can be a full watermark period old. Read the output
  // registers instead: they update at ODR independently of the FIFO.
  if (s_dev->state->num_samples > 0U) {
    // Self-heal: a stalled stream would otherwise freeze peek data forever
    if (!s_dev->state->recovery_pending &&
        (prv_ms_since_last_fifo_read() >= prv_stall_threshold_ms())) {
      s_dev->state->recovery_pending = true;
      prv_queue_work(prv_stall_check_work_cb);
    }

    ret = prv_lsm6dso_read(LSM6DSO_OUTX_L_A, raw, sizeof(raw));
    if (!ret) {
      PBL_LOG_ERR("Failed to read sample");
      return -EIO;
    }

    prv_raw_to_mg(raw, data);
    data->timestamp_us = prv_get_curr_system_time_us();

    return 0;
  }

  // Save CTRL1_XL
  ret = prv_lsm6dso_read(LSM6DSO_CTRL1_XL, &ctrl1_bck, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not read CTRL1_XL register");
    return -EIO;
  }

  // Enable continuous conversion at 52Hz to obtain a single measurement
  ctrl1 = PBL_FIELD_PREP(LSM6DSO_CTRL1_XL_ODR_MASK, LSM6DSO_CTRL1_XL_ODR_52HZ) | prv_fs_bits();
  ret = prv_lsm6dso_write(LSM6DSO_CTRL1_XL, &ctrl1, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write CTRL1_XL register");
    return -EIO;
  }

  // Poll for data ready (timeout after 100ms, ~5x the expected 20ms at 52Hz ODR)
  bool read_ok;
  int rc = PBL_POLL_SLEEP_MS(prv_data_ready(&read_ok), LSM6DSO_DRDY_POLL_DELAY_MS,
                             LSM6DSO_DRDY_POLL_TIMEOUT_MS);
  if (!read_ok) {
    PBL_LOG_ERR("Could not read STATUS_REG register");
    err = -EIO;
    goto end;
  }
  if (rc < 0) {
    PBL_LOG_ERR("DRDY timeout after %d ms", LSM6DSO_DRDY_POLL_TIMEOUT_MS);
    err = -EIO;
    goto end;
  }

  // Read sample
  ret = prv_lsm6dso_read(LSM6DSO_OUTX_L_A, raw, sizeof(raw));
  if (!ret) {
    PBL_LOG_ERR("Failed to read sample");
    err = -EIO;
    goto end;
  }

  // Convert to mg and populate timestamp
  prv_raw_to_mg(raw, data);
  data->timestamp_us = prv_get_curr_system_time_us();

end:
  // Restore CTRL1_XL (back to previous state, e.g. power-down or shake ODR)
  (void)prv_lsm6dso_write(LSM6DSO_CTRL1_XL, &ctrl1_bck, 1);

  return err;
}

static int prv_set_shake_detection(const struct pbl_accel *accel, bool on) {
  bool ret;

  // Configure ODR (use current interval, will be adjusted if < 12.5Hz)
  ret = prv_configure_odr(s_dev->state->sampling_interval_us, on);
  if (!ret) {
    PBL_LOG_ERR("Could not configure ODR");
    return -EIO;
  }

  if (on) {
    uint8_t val;

    // WU_IA may be latched from a previous enablement or an ODR startup
    // transient; clear it so a phantom shake is not dispatched on enable
    if (!prv_lsm6dso_read(LSM6DSO_ALL_INT_SRC, &val, 1)) {
      PBL_LOG_ERR("Could not read ALL_INT_SRC register");
    }
  }

  // Configure INT1
  ret = prv_configure_int1(on, s_dev->state->num_samples > 0U);
  if (!ret) {
    PBL_LOG_ERR("Could not configure INT1");
    return -EIO;
  }

  s_dev->state->shake_detection_enabled = on;
  s_dev->state->wu_active = false;
  // Arm/disarm the INT1 liveness watchdog now that shake routing changed
  prv_update_int1_watchdog();

  PBL_LOG_DBG("%s shake detection", on ? "Enabled" : "Disabled");

  return 0;
}

static int prv_set_shake_sensitivity_high(const struct pbl_accel *accel, bool sensitivity_high) {
  bool ret;
  uint8_t val;

  val = PBL_FIELD_PREP(LSM6DSO_WAKE_UP_THS_WK_THS_MASK, sensitivity_high
                                                            ? CONFIG_ACCEL_LSM6DSO_WK_THS_MIN
                                                            : s_dev->state->wk_ths_curr);
  ret = prv_lsm6dso_write(LSM6DSO_WAKE_UP_THS, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write WAKE_UP_THS register");
    return -EIO;
  }

  PBL_LOG_DBG("Configured shake sensitivity to %s", sensitivity_high ? "high" : "normal");

  return 0;
}

static int prv_set_shake_sensitivity_percent(const struct pbl_accel *accel, uint8_t percent) {
  bool ret;
  uint8_t val;
  uint8_t raw;

  // Reverse mapping: 0 = min sensitivity (MAX threshold), 100 = max sensitivity (MIN threshold)
  // [0, 100] -> [wk_ths_max, wk_ths_min]
  raw = CONFIG_ACCEL_LSM6DSO_WK_THS_MAX -
        (percent * (CONFIG_ACCEL_LSM6DSO_WK_THS_MAX - CONFIG_ACCEL_LSM6DSO_WK_THS_MIN)) / 100U;

  val = PBL_FIELD_PREP(LSM6DSO_WAKE_UP_THS_WK_THS_MASK, raw);
  ret = prv_lsm6dso_write(LSM6DSO_WAKE_UP_THS, &val, 1);
  if (!ret) {
    PBL_LOG_ERR("Could not write WAKE_UP_THS register");
    return -EIO;
  }

  s_dev->state->wk_ths_curr = raw;

  PBL_LOG_DBG("Configured shake sensitivity to %" PRIu8 " (%" PRIu8 ")", percent, raw);

  return 0;
}

static int prv_set_rotated(const struct pbl_accel *accel, bool rotated) {
  s_dev->state->rotated = rotated;
  PBL_LOG_DBG("Set rotated state to %s", rotated ? "true" : "false");

  return 0;
}

// Smallest supported range covering range_mg (0: the configured default)
static uint16_t prv_range_mg(uint32_t range_mg) {
  if (range_mg == 0U) {
    return CONFIG_ACCEL_LSM6DSO_SCALE_MG;
  }
  uint16_t r = 2000U;
  while (r < 16000U && r < range_mg) {
    r *= 2U;
  }
  return r;
}

static int prv_set_config(const struct pbl_accel *accel, const struct pbl_accel_config *config) {
  struct pbl_lsm6dso_state *st = s_dev->state;
  bool on = config->gyro;
  uint16_t range_mg = prv_range_mg(config->range_mg);
  uint8_t val;

  if (st->gyro_enabled == on && st->range_mg == range_mg) {
    return 0;
  }

  prv_configure_int1(false, false);

  // Salvage queued samples before the FIFO is reconfigured
  if (st->num_samples > 0U) {
    prv_lsm6dso_drain_fifo();
  }

  // XL_ULP_EN may only change with the accelerometer powered down, and excludes the gyroscope
  val = PBL_FIELD_PREP(LSM6DSO_CTRL1_XL_ODR_MASK, LSM6DSO_CTRL1_XL_ODR_OFF) | prv_fs_bits();
  if (!prv_lsm6dso_write(LSM6DSO_CTRL1_XL, &val, 1)) {
    return -EIO;
  }
  val = 0U;
  if (!prv_lsm6dso_write(LSM6DSO_CTRL2_G, &val, 1)) {
    return -EIO;
  }
  val = on ? 0U : LSM6DSO_CTRL5_C_XL_ULP_EN;
  if (!prv_lsm6dso_write(LSM6DSO_CTRL5_C, &val, 1)) {
    return -EIO;
  }
  st->gyro_enabled = on;
  st->range_mg = range_mg;

  if (!prv_configure_odr(st->sampling_interval_us, st->shake_detection_enabled)) {
    return -EIO;
  }
  if (st->num_samples > 0U && !prv_lsm6dso_enable_fifo(st->num_samples)) {
    return -EIO;
  }
  if (!prv_configure_int1(st->shake_detection_enabled, st->num_samples > 0U)) {
    return -EIO;
  }
  prv_update_int1_watchdog();

  PBL_LOG_INFO("Gyroscope %s, range %u mg", on ? "on" : "off", range_mg);
  return 0;
}

const struct pbl_accel_ops pbl_lsm6dso_ops = {
  .init = prv_init,
  .set_rotated = prv_set_rotated,
  .set_sampling_interval = prv_set_sampling_interval,
  .get_sampling_interval = prv_get_sampling_interval,
  .set_num_samples = prv_set_num_samples,
  .get_max_num_samples = prv_get_max_num_samples,
  .peek = prv_peek,
  .set_shake_detection = prv_set_shake_detection,
  .set_shake_sensitivity_high = prv_set_shake_sensitivity_high,
  .set_shake_sensitivity_percent = prv_set_shake_sensitivity_percent,
  .set_config = prv_set_config,
};

#include PBL_DT_INSTANCES(st_lsm6dso)
