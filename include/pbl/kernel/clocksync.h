/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @defgroup kernel_clocksync Clock synchronisation
 * @ingroup kernel
 * @brief Map a device's own counter (sensor timestamp, peripheral crystal) onto watch uptime.
 *
 * A driver whose peripheral timestamps samples with its own oscillator feeds timing pairs:
 * the device counter read together with uptime read just before and just after it. The module
 * keeps an exponentially weighted least-squares line fit uptime = a + b * ticks, so converted
 * times carry neither the jitter of a single pair (I2C/SPI read latency) nor the oscillator's
 * frequency error, and the fit follows slow drift (temperature) continuously.
 *
 * - Pairs are weighted by 1 / bracket^2 (bracket = read-after minus read-before): a pair read
 *   while the bus or CPU was busy counts for little.
 * - The fit is re-centred on every pair, so the sums stay small and precise over long runs.
 * - The counter may wrap (counter_bits); ticks to convert must be within half a wrap of the
 *   latest pair.
 * - The fit assumes one continuous counter. When the device is reconfigured (powered down, reset,
 *   switched to another oscillator mode), the driver calls pbl_clocksync_restart().
 * - Uptime is monotonic, so setting the wall clock never bends the fit. Add
 *   pbl_realtime_offset_us() to get wall clock time.
 *
 * @code{.c}
 * pbl_clocksync_init(&cs, 25.0, 32, 60.0);              // 25 us ticks, 32-bit counter, 60 s memory
 * uint64_t a = pbl_uptime_us(); ticks = read_counter(); uint64_t b = pbl_uptime_us();
 * pbl_clocksync_add_pair(&cs, ticks, a, b);
 * uint64_t t_us = pbl_clocksync_to_uptime_us(&cs, sample_ticks) + pbl_realtime_offset_us();
 * @endcode
 * @{
 */

/** @brief Clock synchronisation state. Treat as opaque. */
struct pbl_clocksync {
  double nominal_us_per_tick;
  double tau_us;
  uint32_t counter_mask;
  bool valid;
  uint32_t ref_raw;   // counter value of the latest pair (the fit's origin)
  uint64_t ref_up_us; // uptime origin (the latest pair's midpoint)
  // Weighted sums, origin at the latest pair: x = ticks - ref, y = uptime_us - ref_up_us
  double sw, sx, sy, sxx, sxy;
  double a, b; // fitted y = a + b x
  double var_us2; // weighted residual variance of recent pairs, for diagnostics
  uint32_t num_pairs;
};

/**
 * @brief Start (or restart) a sync.
 * @param cs State.
 * @param nominal_us_per_tick Nominal tick length (e.g. 25 for a 40 kHz timer); used until the
 *        fit has enough span, and as the prior.
 * @param counter_bits Width of the device counter (wraps at 2^counter_bits).
 * @param memory_s Time constant of the exponential forgetting, in seconds.
 */
void pbl_clocksync_init(struct pbl_clocksync *cs, double nominal_us_per_tick, uint8_t counter_bits,
                        double memory_s);

/**
 * @brief Add a timing pair: the device counter read between two uptime reads.
 * @param cs State.
 * @param ticks Counter value.
 * @param up_before_us Uptime right before reading the counter.
 * @param up_after_us Uptime right after reading the counter.
 */
void pbl_clocksync_add_pair(struct pbl_clocksync *cs, uint32_t ticks, uint64_t up_before_us,
                            uint64_t up_after_us);

/**
 * @brief Forget the remembered pairs, keeping the fitted rate as the prior for the next ones.
 *
 * Call it whenever the counter may have stopped, restarted or changed rate: old pairs from another
 * configuration would bend the fit for many memory lengths.
 * @param cs State.
 */
void pbl_clocksync_restart(struct pbl_clocksync *cs);

/**
 * @brief Convert a device counter value to uptime through the current fit.
 * @param cs State, with at least one pair.
 * @param ticks Counter value, within half a wrap of the latest pair.
 * @return Uptime in microseconds.
 */
uint64_t pbl_clocksync_to_uptime_us(const struct pbl_clocksync *cs, uint32_t ticks);

/** @brief Fitted tick length in microseconds (device oscillator vs the watch clock). */
double pbl_clocksync_us_per_tick(const struct pbl_clocksync *cs);

/** @brief RMS deviation of recent pairs from the fit, in microseconds (sync jitter). */
double pbl_clocksync_jitter_us(const struct pbl_clocksync *cs);

/** @} */
