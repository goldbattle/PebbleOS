/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// LSM6DSO driver: FIFO slot parsing, sensor-timestamp mapping, gyroscope and watermark setup,
// against a register-array I2C fake with a scripted FIFO.

#include <stdint.h>
#include <string.h>

#include <pbl/drivers/accel.h>
#include <pbl/drivers/gpio.h>
#include <pbl/drivers/i2c.h>
#include <pbl/drivers/imu/lsm6dso/lsm6dso.h>
#include <pbl/drivers/intc.h>
#include <pbl/kernel/work.h>

#include <clar.h>
#include <fake_mutex.h>

#define REG_FIFO_CTRL1 0x07
#define REG_FIFO_CTRL2 0x08
#define REG_FIFO_CTRL3 0x09
#define REG_FIFO_CTRL4 0x0A
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_CTRL5_C 0x14
#define REG_FIFO_STATUS1 0x3A
#define REG_TIMESTAMP0 0x40
#define REG_FREQ_FINE 0x63
#define REG_FIFO_DATA_OUT_TAG 0x78

#define TAG_GY 0x01
#define TAG_XL 0x02
#define TAG_TS 0x04

#define UPTIME_US 1000000ULL
#define RT_OFFSET_US 1700000000000000LL
#define TS_BASE 100000U
#define TS_STEP 385U
#define TS_REF_AHEAD 40U

////////////////////////////////////////////////////////////////////////////////
// Fakes: clocks
////////////////////////////////////////////////////////////////////////////////

static uint64_t s_uptime_us;
static int64_t s_rt_offset_us;

uint64_t pbl_uptime_us(void) {
  return s_uptime_us;
}

int64_t pbl_realtime_offset_us(void) {
  return s_rt_offset_us;
}

uint64_t pbl_realtime_us(void) {
  return (uint64_t)((int64_t)s_uptime_us + s_rt_offset_us);
}

uint64_t pbl_uptime_ms(void) {
  return s_uptime_us / 1000;
}

uint64_t pbl_uptime_ticks(void) {
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Fakes: work queue, interrupt controller, GPIO
////////////////////////////////////////////////////////////////////////////////

static pbl_work_call_fn_t s_work_cb;
static void *s_work_data;

void pbl_sys_work_call_high_from_isr(pbl_work_call_fn_t cb, void *data) {
  s_work_cb = cb;
  s_work_data = data;
}

bool pbl_sys_work_call_high(pbl_work_call_fn_t cb, void *data) {
  s_work_cb = cb;
  s_work_data = data;
  return true;
}

static pbl_intc_handler_t s_int1_handler;
static void *s_int1_arg;

int pbl_intc_request(const struct pbl_irq_spec *spec, pbl_intc_handler_t handler, void *arg) {
  s_int1_handler = handler;
  s_int1_arg = arg;
  return 0;
}

int pbl_intc_enable(const struct pbl_irq_spec *spec) {
  return 0;
}

int pbl_gpio_configure(const struct pbl_gpio *gpio, uint32_t flags) {
  return 0;
}

int pbl_gpio_get(const struct pbl_gpio *gpio) {
  return 0;  // INT1 pad low: no requeue
}

////////////////////////////////////////////////////////////////////////////////
// Fake I2C bus: register array plus a scripted FIFO
////////////////////////////////////////////////////////////////////////////////

#define FIFO_WORD 7
#define FIFO_CAP 512
#define WRITE_LOG_CAP 256

static uint8_t s_regs[256];
static uint8_t s_fifo[FIFO_CAP][FIFO_WORD];
static unsigned s_fifo_n;
static uint32_t s_ts_ref;  // TIMESTAMP0..3

struct reg_write {
  uint8_t reg;
  uint8_t val;
};
static struct reg_write s_wlog[WRITE_LOG_CAP];
static unsigned s_wlog_n;

static void prv_reg_read(uint8_t reg, uint8_t *buf, uint32_t len) {
  if (reg == REG_FIFO_DATA_OUT_TAG) {
    cl_assert(len % FIFO_WORD == 0);
    unsigned words = len / FIFO_WORD;
    cl_assert(words <= s_fifo_n);
    memcpy(buf, s_fifo, len);
    memmove(s_fifo, s_fifo[words], (s_fifo_n - words) * FIFO_WORD);
    s_fifo_n -= words;
    return;
  }
  for (uint32_t i = 0; i < len; i++) {
    uint8_t r = (uint8_t)(reg + i);
    if (r == REG_FIFO_STATUS1) {
      buf[i] = s_fifo_n & 0xFF;
    } else if (r == REG_FIFO_STATUS1 + 1) {
      buf[i] = (s_fifo_n ? 0x80 : 0) | ((s_fifo_n >> 8) & 3);  // WTM_IA + DIFF[9:8]
    } else if (r >= REG_TIMESTAMP0 && r < REG_TIMESTAMP0 + 4) {
      buf[i] = (uint8_t)(s_ts_ref >> (8 * (r - REG_TIMESTAMP0)));
    } else {
      buf[i] = s_regs[r];
    }
  }
}

static void prv_reg_write(uint8_t reg, const uint8_t *buf, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    uint8_t r = (uint8_t)(reg + i);
    uint8_t v = buf[i];
    if (r == REG_CTRL3_C) {
      v &= (uint8_t)~1U;  // SW_RESET self-clears
    }
    s_regs[r] = v;
    cl_assert(s_wlog_n < WRITE_LOG_CAP);
    s_wlog[s_wlog_n++] = (struct reg_write){r, v};
  }
}

int pbl_i2c_transfer(const struct pbl_i2c_dev *dev, struct pbl_i2c_msg *msgs, uint8_t num_msgs) {
  cl_assert_equal_i(num_msgs, 2);
  cl_assert_equal_i(msgs[0].len, 1);
  uint8_t reg = msgs[0].buf[0];
  if (msgs[1].flags & PBL_I2C_MSG_READ) {
    prv_reg_read(reg, msgs[1].buf, msgs[1].len);
  } else {
    prv_reg_write(reg, msgs[1].buf, msgs[1].len);
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Device instance
////////////////////////////////////////////////////////////////////////////////

PBL_DEVICE_STATE_DEFINE(s_bus);
static const struct pbl_i2c_bus s_bus = {
  .dev = PBL_DEVICE_INIT(s_bus, "fakei2c", nullptr, nullptr)};

PBL_DEVICE_STATE_DEFINE(s_intc);
static const struct pbl_intc s_intc = {
  .dev = PBL_DEVICE_INIT(s_intc, "fakeintc", nullptr, nullptr)};

PBL_DEVICE_STATE_DEFINE(s_port);
static const struct pbl_gpio_port s_port = {
  .dev = PBL_DEVICE_INIT(s_port, "fakegpio", nullptr, nullptr)};

PBL_LSM6DSO_DEFINE(s_lsm6dso, &s_bus, 0x6a,
                   .int1 = PBL_IRQ_SPEC(&s_intc, 38, PBL_IRQ_TYPE_EDGE_RISING),
                   .int1_in = PBL_GPIO(&s_port, 38, 0), .axis_map = {0, 1, 2},
                   .axis_dir = {1, 1, 1});

const struct pbl_accel *const ACCEL = &s_lsm6dso;

////////////////////////////////////////////////////////////////////////////////
// Listener
////////////////////////////////////////////////////////////////////////////////

#define MAX_SAMPLES 64

struct got {
  int32_t x, y, z;
  int32_t gx, gy, gz;  // raw
  int32_t gx_dps10;
  uint64_t ts;
};
static struct got s_got[MAX_SAMPLES];
static unsigned s_got_n;
static int32_t s_gyro_den;
static unsigned s_batches;

static int32_t prv_axis(const AccelRawBatch *b, const uint8_t *s, int a) {
  int16_t raw = (int16_t)(s[b->axis[a].offset] | (s[b->axis[a].offset + 1] << 8));
  return b->axis[a].sign * ((int32_t)raw * b->scale_num) / b->scale_den;
}

static void prv_samples(const struct pbl_accel *accel, const AccelRawBatch *b, void *user) {
  s_batches++;
  s_gyro_den = b->gyro_scale_den;
  cl_assert(b->timestamps_us != nullptr);
  for (uint32_t i = 0; i < b->num_samples; i++) {
    const uint8_t *s = b->data + i * b->stride;
    struct got *g = &s_got[s_got_n++];
    g->x = prv_axis(b, s, AXIS_X);
    g->y = prv_axis(b, s, AXIS_Y);
    g->z = prv_axis(b, s, AXIS_Z);
    g->ts = b->timestamps_us[i];
    if (b->gyro_scale_den != 0) {
      const uint8_t *p = s + b->gyro_axis[0].offset;
      g->gx = (int16_t)(p[0] | (p[1] << 8));
      p = s + b->gyro_axis[1].offset;
      g->gy = (int16_t)(p[0] | (p[1] << 8));
      p = s + b->gyro_axis[2].offset;
      g->gz = (int16_t)(p[0] | (p[1] << 8));
      g->gx_dps10 = g->gx * b->gyro_scale_num / b->gyro_scale_den;
    }
  }
}

static const struct pbl_accel_listener s_listener = {.samples = prv_samples};

////////////////////////////////////////////////////////////////////////////////
// Helpers
////////////////////////////////////////////////////////////////////////////////

static void prv_push(uint8_t sensor, uint8_t cnt, const uint8_t data[6]) {
  cl_assert(s_fifo_n < FIFO_CAP);
  s_fifo[s_fifo_n][0] = (uint8_t)((sensor << 3) | ((cnt & 3) << 1));
  memcpy(&s_fifo[s_fifo_n][1], data, 6);
  s_fifo_n++;
}

static void prv_push_ts(uint8_t cnt, uint32_t ts) {
  uint8_t d[6] = {(uint8_t)ts, (uint8_t)(ts >> 8), (uint8_t)(ts >> 16), (uint8_t)(ts >> 24), 0, 0};
  prv_push(TAG_TS, cnt, d);
}

static void prv_put16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)((uint16_t)v >> 8);
}

// Slot k (0-based) accelerometer: 250*(k+1), -125*(k+1), 1000 mg at 2 g full scale
static void prv_push_xl(uint8_t cnt, int k) {
  uint8_t d[6];
  prv_put16(&d[0], (int16_t)(4096 * (k + 1)));
  prv_put16(&d[2], (int16_t)(-2048 * (k + 1)));
  prv_put16(&d[4], 16384);
  prv_push(TAG_XL, cnt, d);
}

// Slot k gyroscope raw: 100*(k+1), -200*(k+1), 300
static void prv_push_g(uint8_t cnt, int k) {
  uint8_t d[6];
  prv_put16(&d[0], (int16_t)(100 * (k + 1)));
  prv_put16(&d[2], (int16_t)(-200 * (k + 1)));
  prv_put16(&d[4], 300);
  prv_push(TAG_GY, cnt, d);
}

// Raise INT1 and run the work it queues on the "timer task"
static void prv_int1(void) {
  s_work_cb = nullptr;
  s_int1_handler(s_int1_arg);
  cl_assert(s_work_cb != nullptr);
  s_work_cb(s_work_data);
}

//! Expected sample time: uptime at the read minus the sample's age in sensor ticks. On uptime, not
//! the wall clock: the fake's wall clock offset (RT_OFFSET_US) must not show up in any stamp.
static uint64_t prv_expect_ts(uint32_t ts_ref, uint32_t ts, double tick_us) {
  double age_us = (double)(int32_t)(ts_ref - ts) * tick_us;
  return (uint64_t)((int64_t)s_uptime_us - (int64_t)(age_us + 0.5));
}

static void prv_setup(int8_t freq_fine) {
  static bool s_first = true;

  memset(s_regs, 0, sizeof(s_regs));
  s_regs[REG_WHO_AM_I] = 0x6C;
  s_regs[REG_FREQ_FINE] = (uint8_t)freq_fine;
  s_fifo_n = 0;
  s_wlog_n = 0;
  s_got_n = 0;
  s_batches = 0;
  s_gyro_den = -1;
  s_uptime_us = UPTIME_US;
  s_rt_offset_us = RT_OFFSET_US;
  memset(&s_lsm6dso_lsm6dso_state, 0, sizeof(s_lsm6dso_lsm6dso_state));
  if (s_first) {
    s_first = false;
    cl_assert_equal_i(pbl_device_init(&ACCEL->dev), 0);
  } else {
    cl_assert_equal_i(pbl_lsm6dso_ops.init(ACCEL), 0);
  }
  cl_assert_equal_i(pbl_accel_set_listener(ACCEL, &s_listener), 0);
  cl_assert_equal_i(pbl_accel_set_sampling_interval(ACCEL, 9615, nullptr), 0);
}

static void prv_check_xl(unsigned i, int k) {
  cl_assert_equal_i(s_got[i].x, 250 * (k + 1));
  cl_assert_equal_i(s_got[i].y, -125 * (k + 1));
  cl_assert_equal_i(s_got[i].z, 1000);
}

static void prv_check_g(unsigned i, int k) {
  cl_assert_equal_i(s_got[i].gx, 100 * (k + 1));
  cl_assert_equal_i(s_got[i].gy, -200 * (k + 1));
  cl_assert_equal_i(s_got[i].gz, 300);
  cl_assert_equal_i(s_got[i].gx_dps10, 70 * (k + 1));
}

void test_lsm6dso__initialize(void) {
}

void test_lsm6dso__cleanup(void) {
  pbl_accel_set_num_samples(ACCEL, 0);
  pbl_accel_set_listener(ACCEL, nullptr);
  fake_mutex_reset(true /* assert_all_unlocked */);
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

void test_lsm6dso__accel_only_stream(void) {
  prv_setup(0);
  cl_assert_equal_i(pbl_accel_set_num_samples(ACCEL, 5), 0);
  cl_assert_equal_i(s_regs[REG_FIFO_CTRL1], 10);  // 2 words per slot

  for (int k = 0; k < 5; k++) {
    prv_push_xl(k, k);
    prv_push_ts(k, TS_BASE + TS_STEP * k);
  }
  s_ts_ref = TS_BASE + TS_STEP * 4 + TS_REF_AHEAD;
  prv_int1();

  cl_assert_equal_i(s_got_n, 5);
  cl_assert_equal_i(s_gyro_den, 0);
  for (int k = 0; k < 5; k++) {
    prv_check_xl(k, k);
    // up 1000000 - (40 + 385*(4-k)) * 25 us (uptime: no wall clock offset)
    uint64_t expect = 1000000ULL - (uint64_t)(TS_REF_AHEAD + TS_STEP * (4 - k)) * 25;
    cl_assert(s_got[k].ts == expect);
    cl_assert(s_got[k].ts == prv_expect_ts(s_ts_ref, TS_BASE + TS_STEP * k, 25.0));
  }
  // Last sample: 40 ticks (1000 us) before the read
  cl_assert(s_got[4].ts == 999000ULL);
  cl_assert(s_got[0].ts == 1000000ULL - 1540ULL * 25 - 1000ULL);
}

static int prv_set_gyro(bool on) {
  return pbl_accel_set_config(ACCEL, &(struct pbl_accel_config){.gyro = on});
}

static void prv_gyro_stream(bool ts_first) {
  prv_setup(0);
  cl_assert_equal_i(pbl_accel_set_num_samples(ACCEL, 3), 0);
  s_wlog_n = 0;
  cl_assert_equal_i(prv_set_gyro(true), 0);

  // CTRL5_C=0 (ULP off) was written while the accelerometer ODR was off, then ODRs came back
  int i5 = -1;
  uint8_t xl_before = 0xFF;
  for (unsigned i = 0; i < s_wlog_n; i++) {
    if (s_wlog[i].reg == REG_CTRL1_XL && i5 < 0) {
      xl_before = s_wlog[i].val;
    }
    if (s_wlog[i].reg == REG_CTRL5_C) {
      i5 = (int)i;
      break;
    }
  }
  cl_assert(i5 >= 0);
  cl_assert_equal_i(s_wlog[i5].val, 0x00);
  cl_assert_equal_i(xl_before >> 4, 0);
  bool g_after = false;
  for (unsigned i = i5 + 1; i < s_wlog_n; i++) {
    if (s_wlog[i].reg == REG_CTRL2_G && (s_wlog[i].val >> 4) != 0) {
      g_after = true;
    }
  }
  cl_assert(g_after);
  cl_assert(s_regs[REG_CTRL1_XL] >> 4 != 0);
  cl_assert_equal_i(s_regs[REG_CTRL2_G] >> 4, s_regs[REG_CTRL1_XL] >> 4);
  cl_assert_equal_i(s_regs[REG_CTRL2_G] & 0x0C, 0x0C);  // 2000 dps
  cl_assert_equal_i(s_regs[REG_FIFO_CTRL3] >> 4, s_regs[REG_FIFO_CTRL3] & 0xF);
  cl_assert(s_regs[REG_FIFO_CTRL3] >> 4 != 0);
  cl_assert_equal_i(s_regs[REG_FIFO_CTRL4], 0x46);
  cl_assert_equal_i(s_regs[REG_FIFO_CTRL1], 9);

  for (int k = 0; k < 3; k++) {
    if (ts_first) {
      prv_push_ts(k, TS_BASE + TS_STEP * k);
      prv_push_xl(k, k);
      prv_push_g(k, k);
    } else {
      prv_push_g(k, k);
      prv_push_xl(k, k);
      prv_push_ts(k, TS_BASE + TS_STEP * k);
    }
  }
  s_ts_ref = TS_BASE + TS_STEP * 2 + TS_REF_AHEAD;
  prv_int1();

  cl_assert_equal_i(s_got_n, 3);
  cl_assert_equal_i(s_gyro_den, 10);
  for (int k = 0; k < 3; k++) {
    prv_check_xl(k, k);
    prv_check_g(k, k);
    cl_assert(s_got[k].ts == prv_expect_ts(s_ts_ref, TS_BASE + TS_STEP * k, 25.0));
    cl_assert(s_got[k].ts ==
              1000000ULL - (uint64_t)(40 + 385 * (2 - k)) * 25);
  }

  cl_assert_equal_i(prv_set_gyro(false), 0);
  cl_assert_equal_i(s_regs[REG_CTRL5_C], 0x80);
  cl_assert_equal_i(s_regs[REG_CTRL2_G], 0);
  cl_assert_equal_i(s_regs[REG_FIFO_CTRL3] >> 4, 0);
  cl_assert_equal_i(s_regs[REG_FIFO_CTRL1], 6);
}

void test_lsm6dso__gyro_g_xl_ts(void) {
  prv_gyro_stream(false);
}

void test_lsm6dso__gyro_ts_xl_g(void) {
  prv_gyro_stream(true);
}

void test_lsm6dso__slot_split_across_reads(void) {
  prv_setup(0);
  cl_assert_equal_i(pbl_accel_set_num_samples(ACCEL, 4), 0);

  // Read 1: slots 0 and 1 complete, slot 2 has only its XL word
  for (int k = 0; k < 2; k++) {
    prv_push_xl(k, k);
    prv_push_ts(k, TS_BASE + TS_STEP * k);
  }
  prv_push_xl(2, 2);
  s_ts_ref = TS_BASE + TS_STEP * 1 + TS_REF_AHEAD;
  prv_int1();
  cl_assert_equal_i(s_got_n, 2);
  prv_check_xl(0, 0);
  prv_check_xl(1, 1);

  // Read 2: slot 2's timestamp, then slot 3 complete
  prv_push_ts(2, TS_BASE + TS_STEP * 2);
  prv_push_xl(3, 3);
  prv_push_ts(3, TS_BASE + TS_STEP * 3);
  s_ts_ref = TS_BASE + TS_STEP * 3 + TS_REF_AHEAD;
  s_uptime_us = UPTIME_US + 2ULL * TS_STEP * 25ULL;  // the watch clock ran on with the sensor's
  prv_int1();

  cl_assert_equal_i(s_got_n, 4);
  cl_assert_equal_i(s_batches, 2);
  prv_check_xl(2, 2);
  prv_check_xl(3, 3);
  cl_assert(s_got[2].ts == prv_expect_ts(s_ts_ref, TS_BASE + TS_STEP * 2, 25.0));
  cl_assert(s_got[2].ts ==
            1000000ULL + 770ULL * 25 - (uint64_t)(40 + 385) * 25);
  cl_assert(s_got[3].ts == 1000000ULL + 770ULL * 25 - 40ULL * 25);
}

void test_lsm6dso__freq_fine_tick(void) {
  prv_setup(10);
  const double tick_us = 25.0 * 10000.0 / 10150.0;  // 24.6305 us
  cl_assert(pbl_clocksync_us_per_tick(&s_lsm6dso_lsm6dso_state.sync) == tick_us);
  cl_assert_equal_i(pbl_accel_set_num_samples(ACCEL, 5), 0);

  for (int k = 0; k < 5; k++) {
    prv_push_xl(k, k);
    prv_push_ts(k, TS_BASE + TS_STEP * k);
  }
  s_ts_ref = TS_BASE + TS_STEP * 4 + TS_REF_AHEAD;
  prv_int1();

  cl_assert_equal_i(s_got_n, 5);
  for (int k = 0; k < 5; k++) {
    prv_check_xl(k, k);
    uint64_t expect = prv_expect_ts(s_ts_ref, TS_BASE + TS_STEP * k, tick_us);
    cl_assert(s_got[k].ts == expect);
  }
  // 40 ticks = 985.2 us -> 985 us before the read; 1580 ticks = 38916.25 -> 38915 us
  cl_assert(s_got[4].ts == 1000000ULL - 985ULL);
  cl_assert(s_got[0].ts == 1000000ULL - 38916ULL);
}

// Starting the stream or switching the gyroscope (ULP <-> high-performance oscillator) restarts the
// timestamp fit, so pairs from another configuration never bend it
void test_lsm6dso__reconfiguration_restarts_clock_fit(void) {
  prv_setup(0);
  cl_assert_equal_i(pbl_accel_set_num_samples(ACCEL, 5), 0);
  cl_assert(!s_lsm6dso_lsm6dso_state.sync.valid);
  for (int k = 0; k < 5; k++) {
    prv_push_xl(k, k);
    prv_push_ts(k, TS_BASE + TS_STEP * k);
  }
  s_ts_ref = TS_BASE + TS_STEP * 4 + TS_REF_AHEAD;
  prv_int1();
  cl_assert(s_lsm6dso_lsm6dso_state.sync.valid);
  cl_assert_equal_i(prv_set_gyro(true), 0);
  cl_assert(!s_lsm6dso_lsm6dso_state.sync.valid);
}

void test_lsm6dso__watermark_with_gyro(void) {
  prv_setup(0);
  cl_assert_equal_i(pbl_accel_set_num_samples(ACCEL, 25), 0);
  cl_assert_equal_i(s_regs[REG_FIFO_CTRL1], 50);
  cl_assert_equal_i(prv_set_gyro(true), 0);
  cl_assert_equal_i(s_regs[REG_FIFO_CTRL1], 75);
  cl_assert_equal_i(s_regs[REG_FIFO_CTRL2] & 1, 0);
  // Setting the sample count while the gyroscope runs gives the same watermark
  cl_assert_equal_i(pbl_accel_set_num_samples(ACCEL, 0), 0);
  cl_assert_equal_i(pbl_accel_set_num_samples(ACCEL, 25), 0);
  cl_assert_equal_i(s_regs[REG_FIFO_CTRL1], 75);
}

// The range defaults to the Kconfig value, is rounded up to a supported one, and every batch is
// scaled by the range in effect
void test_lsm6dso__range_follows_config(void) {
  prv_setup(0);
  cl_assert_equal_i(pbl_accel_set_num_samples(ACCEL, 1), 0);
  cl_assert_equal_i(s_regs[REG_CTRL1_XL] & 0x0C, 0x00);  // FS_XL = 2 g

  cl_assert_equal_i(pbl_accel_set_config(ACCEL, &(struct pbl_accel_config){.range_mg = 5000}), 0);
  cl_assert_equal_i(s_regs[REG_CTRL1_XL] & 0x0C, 0x0C);  // FS_XL = 8 g
  prv_push_xl(0, 0);
  prv_push_ts(0, TS_BASE);
  s_ts_ref = TS_BASE + TS_REF_AHEAD;
  prv_int1();
  cl_assert_equal_i(s_got_n, 1);
  cl_assert_equal_i(s_got[0].x, 1000);  // 4 times the 2 g values of prv_check_xl()
  cl_assert_equal_i(s_got[0].z, 4000);

  cl_assert_equal_i(pbl_accel_set_config(ACCEL, &(struct pbl_accel_config){.range_mg = 99000}), 0);
  cl_assert_equal_i(s_regs[REG_CTRL1_XL] & 0x0C, 0x04);  // FS_XL = 16 g, the largest
  cl_assert_equal_i(pbl_accel_set_config(ACCEL, &(struct pbl_accel_config){0}), 0);
  cl_assert_equal_i(s_regs[REG_CTRL1_XL] & 0x0C, 0x00);
}
