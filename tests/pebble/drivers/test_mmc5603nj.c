/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// MMC5603NJ driver: sample time stamps from the clock fit over the poll windows, against a fake
// sensor that measures on its own (slightly slow) oscillator and is polled with jitter.

#include <stdint.h>
#include <string.h>

#include <pbl/drivers/i2c.h>
#include <pbl/drivers/imu/mmc5603nj/mmc5603nj.h>
#include <pbl/drivers/mag.h>

#include <clar.h>
#include <fake_mutex.h>
#include <fake_work.h>

#define REG_XOUT0 0x00
#define REG_STATUS1 0x18
#define REG_WHO_AM_I 0x39
#define STATUS1_MEAS_M_DONE 0x40

#define RT_OFFSET_US 1700000000000000LL
#define POLL_US 2000U  // the driver's poll period at 100 Hz (1000 / 100 / 4 ms)
#define TRUE_PERIOD_US 10020.0  // the sensor's oscillator: 0.2 % slower than the nominal 100 Hz

////////////////////////////////////////////////////////////////////////////////
// Fakes: clocks
////////////////////////////////////////////////////////////////////////////////

static uint64_t s_uptime_us;

uint64_t pbl_uptime_us(void) {
  return s_uptime_us;
}

int64_t pbl_realtime_offset_us(void) {
  return RT_OFFSET_US;
}

uint64_t pbl_realtime_us(void) {
  return s_uptime_us + RT_OFFSET_US;
}

////////////////////////////////////////////////////////////////////////////////
// Fake sensor on a register-array I2C bus
////////////////////////////////////////////////////////////////////////////////

static uint8_t s_regs[256];
static double s_t0_us;          // uptime of the sensor's sample 0
static uint32_t s_next_k;       // next sample the sensor will complete
static int64_t s_latest_k;      // latest completed sample, -1 before the first
static bool s_ready;            // MEAS_M_DONE: a sample completed since the data was last read
static int64_t s_read_k[512];   // which sample each data read got
static unsigned s_reads;

static double prv_true_us(uint32_t k) {
  return s_t0_us + k * TRUE_PERIOD_US;
}

//! Let the sensor run up to the current uptime.
static void prv_sensor_advance(void) {
  while (prv_true_us(s_next_k) <= (double)s_uptime_us) {
    s_latest_k = s_next_k++;
    s_ready = true;
  }
}

int pbl_i2c_transfer(const struct pbl_i2c_dev *dev, struct pbl_i2c_msg *msgs, uint8_t num_msgs) {
  uint8_t reg = msgs[0].buf[0];
  if (num_msgs == 1) {  // a register write: register, then the data, in one message
    memcpy(&s_regs[reg], msgs[0].buf + 1, msgs[0].len - 1);
    return 0;
  }
  cl_assert_equal_i(num_msgs, 2);
  if (msgs[1].flags & PBL_I2C_MSG_READ) {
    for (uint32_t i = 0; i < msgs[1].len; i++) {
      uint8_t r = (uint8_t)(reg + i);
      msgs[1].buf[i] = (r == REG_STATUS1) ? (s_ready ? STATUS1_MEAS_M_DONE : 0) : s_regs[r];
    }
    if (reg == REG_XOUT0) {  // the data read takes the latest sample
      cl_assert(s_reads < 512);
      s_read_k[s_reads++] = s_latest_k;
      s_ready = false;
    }
  } else {
    memcpy(&s_regs[reg], msgs[1].buf, msgs[1].len);
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Device instance and listener
////////////////////////////////////////////////////////////////////////////////

PBL_DEVICE_STATE_DEFINE(s_bus);
static const struct pbl_i2c_bus s_bus = {
  .dev = PBL_DEVICE_INIT(s_bus, "fakei2c", nullptr, nullptr)};

PBL_MMC5603NJ_DEFINE(s_mmc, &s_bus, 0x30, .axis_map = {0, 1, 2}, .axis_dir = {1, 1, 1});

static uint64_t s_ts[512];
static unsigned s_got;

static void prv_sample(const struct pbl_mag *mag, const MagData *sample, void *user_data) {
  cl_assert(s_got < 512);
  s_ts[s_got++] = sample->timestamp_us;
}

static const struct pbl_mag_listener s_listener = {.sample = prv_sample};

////////////////////////////////////////////////////////////////////////////////
// Helpers
////////////////////////////////////////////////////////////////////////////////

//! One poll of the driver, after the given time; the timer's own jitter is a few hundred us.
static void prv_poll_after(uint32_t us) {
  s_uptime_us += us;
  prv_sensor_advance();
  struct pbl_delayed_work *w = fake_work_get_next();
  cl_assert(w != nullptr);
  fake_work_fire(w);
}

static void prv_run_polls(unsigned n) {
  static unsigned s_i;
  for (unsigned i = 0; i < n; i++, s_i++) {
    prv_poll_after(POLL_US + (s_i % 5U) * 120U);  // 2.0 .. 2.48 ms
  }
}

//! Stamp of sample i minus the sensor's real measurement time, us.
static double prv_err_us(unsigned i) {
  return (double)(int64_t)(s_ts[i] - RT_OFFSET_US) - prv_true_us((uint32_t)s_read_k[i]);
}

void test_mmc5603nj__initialize(void) {
  static bool s_first = true;
  memset(s_regs, 0, sizeof(s_regs));
  s_regs[REG_WHO_AM_I] = 0x10;
  s_uptime_us = 1000000;
  s_t0_us = 1000000.0 + 3700.0;  // the first sample lands 3.7 ms after sampling starts
  s_next_k = 0;
  s_latest_k = -1;
  s_ready = false;
  s_reads = 0;
  s_got = 0;
  if (s_first) {
    s_first = false;
    cl_assert_equal_i(pbl_device_init(&s_mmc.dev), 0);
  }
  cl_assert_equal_i(pbl_mag_set_listener(&s_mmc, &s_listener), 0);
  cl_assert_equal_i(pbl_mag_set_sampling_interval(&s_mmc, 10000, nullptr), 0);
}

void test_mmc5603nj__cleanup(void) {
  pbl_mag_set_sampling_interval(&s_mmc, 0, nullptr);
  pbl_mag_set_listener(&s_mmc, nullptr);
  fake_mutex_reset(true /* assert_all_unlocked */);
  fake_work_cleanup();
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

// Each sample turns up somewhere in a 2-2.5 ms poll window; stamping it at the window's middle (the
// old way) is up to ~1.2 ms off. The fit over many windows lands far closer, and evenly spaced.
void test_mmc5603nj__stamps_converge_on_the_measurement_times(void) {
  prv_run_polls(2000);  // 4.4 s, ~440 samples
  cl_assert(s_got > 400);
  cl_assert_equal_i(s_got, s_reads);

  double worst = 0;
  for (unsigned i = s_got - 100; i < s_got; i++) {  // the last second
    double e = prv_err_us(i);
    worst = (e < 0 ? -e : e) > worst ? (e < 0 ? -e : e) : worst;
  }
  cl_assert(worst < 150.0);

  // Intervals: the sensor's real period, not jumping by poll windows
  for (unsigned i = s_got - 100; i < s_got; i++) {
    double dt = (double)(s_ts[i] - s_ts[i - 1]);
    cl_assert(dt > TRUE_PERIOD_US - 60.0 && dt < TRUE_PERIOD_US + 60.0);
  }
}

// A poll 25 ms late finds only the newest sample (the sensor overwrote the others). Its stamp still
// matches its own measurement time, and the following samples continue the right clock.
void test_mmc5603nj__a_late_poll_skips_the_missed_samples(void) {
  prv_run_polls(2000);
  unsigned before = s_got;
  prv_poll_after(25000);
  prv_run_polls(50);

  cl_assert(s_read_k[before] - s_read_k[before - 1] >= 2);  // samples were missed
  for (unsigned i = before; i < s_got; i++) {
    double e = prv_err_us(i);
    cl_assert(e > -200.0 && e < 200.0);
  }
}

// Each new sampling start begins a new clock at the nominal rate: no stamps carried over.
void test_mmc5603nj__restart_begins_a_new_clock(void) {
  prv_run_polls(500);
  cl_assert_equal_i(pbl_mag_set_sampling_interval(&s_mmc, 0, nullptr), 0);
  s_uptime_us += 5000000;  // 5 s later
  s_t0_us = (double)s_uptime_us + 1500.0;
  s_next_k = 0;
  s_latest_k = -1;
  s_ready = false;
  unsigned before = s_got;
  cl_assert_equal_i(pbl_mag_set_sampling_interval(&s_mmc, 10000, nullptr), 0);
  prv_run_polls(100);
  cl_assert(s_got > before);
  double e = prv_err_us(before);  // the first sample of the new run: within its poll window
  cl_assert(e > -2600.0 && e < 2600.0);
}
