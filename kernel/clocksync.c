/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/kernel/clocksync.h>

// A pair whose bracket is this short counts fully; longer brackets weigh 1/bracket^2.
#define CLOCKSYNC_MIN_BRACKET_US 20.0
// Fit the rate only once the remembered pairs span this many ticks' worth of time.
#define CLOCKSYNC_MIN_SPAN_US 2000000.0

void pbl_clocksync_init(struct pbl_clocksync *cs, double nominal_us_per_tick, uint8_t counter_bits,
                        double memory_s) {
  *cs = (struct pbl_clocksync){
    .nominal_us_per_tick = nominal_us_per_tick,
    .tau_us = memory_s * 1e6,
    .counter_mask = (counter_bits >= 32U) ? 0xFFFFFFFFUL : ((1UL << counter_bits) - 1UL),
    .b = nominal_us_per_tick,
  };
}

//! Signed tick difference (to - from), unwrapped for a counter of counter_mask width.
static double prv_dticks(const struct pbl_clocksync *cs, uint32_t from, uint32_t to) {
  uint32_t d = (to - from) & cs->counter_mask;
  uint32_t half = (cs->counter_mask >> 1) + 1UL;
  return (d >= half) ? -(double)(cs->counter_mask - d + 1UL) : (double)d;
}

static void prv_refit(struct pbl_clocksync *cs) {
  double det = cs->sw * cs->sxx - cs->sx * cs->sx;
  // Spread of x (in us) the remembered pairs cover; below the minimum, keep the prior rate.
  double spread_us = (cs->sw > 0.0 && det > 0.0) ? (det / (cs->sw * cs->sw)) : 0.0;
  if (spread_us > 0.0) {
    spread_us = __builtin_sqrt(spread_us) * cs->nominal_us_per_tick;
  }
  if (cs->num_pairs >= 3U && spread_us >= CLOCKSYNC_MIN_SPAN_US / 4.0) {
    cs->b = (cs->sw * cs->sxy - cs->sx * cs->sy) / det;
  }
  cs->a = (cs->sw > 0.0) ? (cs->sy - cs->b * cs->sx) / cs->sw : 0.0;
}

void pbl_clocksync_add_pair(struct pbl_clocksync *cs, uint32_t ticks, uint64_t up_before_us,
                            uint64_t up_after_us) {
  uint64_t mid = up_before_us + (up_after_us - up_before_us) / 2U;
  double bracket = (double)(up_after_us - up_before_us);
  double w = 1.0 / ((bracket < CLOCKSYNC_MIN_BRACKET_US ? CLOCKSYNC_MIN_BRACKET_US : bracket) *
                    (bracket < CLOCKSYNC_MIN_BRACKET_US ? CLOCKSYNC_MIN_BRACKET_US : bracket));

  if (!cs->valid) {
    cs->valid = true;
    cs->ref_raw = ticks;
    cs->ref_up_us = mid;
    cs->sw = w;
    cs->sx = cs->sy = cs->sxx = cs->sxy = 0.0;
    cs->num_pairs = 1U;
    prv_refit(cs);
    return;
  }

  // Move the origin to the new pair: x' = x - dx, y' = y - dy (sums shift exactly)
  double dx = prv_dticks(cs, cs->ref_raw, ticks);
  double dy = (double)(int64_t)(mid - cs->ref_up_us);

  // Residual of the new pair against the current fit, before it joins the fit
  double resid = dy - (cs->a + cs->b * dx);

  // Forget old pairs with time constant tau (first-order decay, no libm needed)
  double elapsed = dy > 0.0 ? dy : 0.0;
  double keep = cs->tau_us / (cs->tau_us + elapsed);
  cs->sw *= keep;
  cs->sx *= keep;
  cs->sy *= keep;
  cs->sxx *= keep;
  cs->sxy *= keep;
  cs->var_us2 = keep * cs->var_us2 + (1.0 - keep) * resid * resid;

  cs->sxx = cs->sxx - 2.0 * dx * cs->sx + dx * dx * cs->sw;
  cs->sxy = cs->sxy - dx * cs->sy - dy * cs->sx + dx * dy * cs->sw;
  cs->sx -= dx * cs->sw;
  cs->sy -= dy * cs->sw;

  // Add the new pair at the origin (x = 0, y = 0)
  cs->sw += w;

  cs->ref_raw = ticks;
  cs->ref_up_us = mid;
  cs->num_pairs++;
  prv_refit(cs);
}

void pbl_clocksync_restart(struct pbl_clocksync *cs) {
  cs->valid = false;  // the next pair starts the fit over, from the current rate
}

uint64_t pbl_clocksync_to_uptime_us(const struct pbl_clocksync *cs, uint32_t ticks) {
  double x = prv_dticks(cs, cs->ref_raw, ticks);
  double y = cs->a + cs->b * x;
  int64_t up = (int64_t)cs->ref_up_us + (int64_t)(y >= 0.0 ? y + 0.5 : y - 0.5);
  return (up > 0) ? (uint64_t)up : 0U;
}

double pbl_clocksync_us_per_tick(const struct pbl_clocksync *cs) {
  return cs->b;
}

double pbl_clocksync_jitter_us(const struct pbl_clocksync *cs) {
  return __builtin_sqrt(cs->var_us2);
}
