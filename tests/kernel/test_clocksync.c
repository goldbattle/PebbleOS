/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#define _GNU_SOURCE
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <pbl/kernel/clocksync.h>

#include <clar.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// A device counter of 25 us nominal ticks whose oscillator runs 1.7 % fast and wanders +-50 ppm
// (temperature), read every 160 ms between two uptime reads 100..400 us apart, with occasional
// 3 ms bus stalls. Samples converted through the fit must land within tens of microseconds.

static double prv_dev_ticks(double t_us, double start) {
  double wander_us = 50e-6 * (1200e6 / (2 * M_PI)) * (1 - cos(2 * M_PI * t_us / 1200e6));
  return start + (t_us + wander_us) * 1.017 / 25.0;
}

static double prv_urand(void) {
  return rand() / (double)RAND_MAX;
}

static void prv_run(uint8_t bits, double start_ticks, double seconds, double *rms_us,
                    double *max_us) {
  struct pbl_clocksync cs;
  uint64_t mask = (bits >= 32) ? 0xFFFFFFFFull : ((1ull << bits) - 1);
  double sum_sq = 0, max_err = 0;
  long n = 0;

  pbl_clocksync_init(&cs, 25.0, bits, 10.0);
  srand(1);
  for (double t = 1e6; t < seconds * 1e6; t += 160e3) {
    double bracket = 100 + 300 * prv_urand() + (prv_urand() < 0.02 ? 3000 : 0);
    double latch = t + bracket * prv_urand();
    uint32_t raw = (uint32_t)((uint64_t)floor(prv_dev_ticks(latch, start_ticks)) & mask);
    pbl_clocksync_add_pair(&cs, raw, (uint64_t)t, (uint64_t)(t + bracket));
    if (t < 30e6) {
      continue;  // settling
    }
    for (int k = 0; k < 17; k++) {  // the samples of this FIFO read
      double ts = t - k * 9600;
      uint32_t r = (uint32_t)((uint64_t)floor(prv_dev_ticks(ts, start_ticks)) & mask);
      double e = (double)pbl_clocksync_to_uptime_us(&cs, r) - ts;
      sum_sq += e * e;
      max_err = fmax(max_err, fabs(e));
      n++;
    }
  }
  *rms_us = sqrt(sum_sq / n);
  *max_us = max_err;
}

void test_clocksync__first_pair_uses_nominal_rate(void) {
  struct pbl_clocksync cs;
  pbl_clocksync_init(&cs, 25.0, 32, 10.0);
  pbl_clocksync_add_pair(&cs, 1000U, 5000000U, 5000000U);
  cl_assert_equal_i(pbl_clocksync_to_uptime_us(&cs, 1000U), 5000000);
  cl_assert_equal_i(pbl_clocksync_to_uptime_us(&cs, 960U), 5000000 - 40 * 25);
}

void test_clocksync__tracks_fast_drifting_oscillator(void) {
  double rms, max;
  prv_run(32, 1000.0, 1800.0, &rms, &max);
  cl_assert(rms < 30.0);
  cl_assert(max < 150.0);
}

void test_clocksync__survives_counter_wrap(void) {
  double rms, max;
  prv_run(32, 4294967296.0 - 2e6, 600.0, &rms, &max);  // 32-bit wrap after ~50 s
  cl_assert(rms < 30.0);
  prv_run(16, 0.0, 300.0, &rms, &max);  // 16-bit counter wraps every ~1.6 s
  cl_assert(rms < 30.0);
  cl_assert(max < 150.0);
}

// The counter switches to a 1.5 % slower rate without stopping (another oscillator mode): after a
// restart the fit follows the new rate within seconds.
void test_clocksync__restart_follows_new_rate(void) {
  struct pbl_clocksync cs;
  double tick = 25.0;
  double ticks = 0.0;
  double t = 1e6;

  pbl_clocksync_init(&cs, 25.0, 32, 10.0);
  for (; t < 61e6; t += 160e3, ticks += 160e3 / tick) {
    pbl_clocksync_add_pair(&cs, (uint32_t)ticks, (uint64_t)t, (uint64_t)t);
  }
  tick = 25.0 * 1.015;
  pbl_clocksync_restart(&cs);
  for (double end = t + 3e6; t < end; t += 160e3, ticks += 160e3 / tick) {
    pbl_clocksync_add_pair(&cs, (uint32_t)ticks, (uint64_t)t, (uint64_t)t);
  }
  cl_assert(fabs(pbl_clocksync_us_per_tick(&cs) / tick - 1.0) < 1e-4);
}
