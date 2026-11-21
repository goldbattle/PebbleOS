/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/input/input.h>
#include <pbl/kernel/time.h>

PBL_ITERABLE_SECTION_DECLARE(const struct pbl_input_callback, input_cbs);

void pbl_input_report(enum pbl_input_type type, uint16_t code, int32_t value, bool sync) {
  pbl_input_report_at(type, code, value, sync, pbl_realtime_us());
}

void pbl_input_report_at(enum pbl_input_type type, uint16_t code, int32_t value, bool sync,
                         uint64_t timestamp_us) {
  const struct pbl_input_event evt = {
    .type = type,
    .sync = sync,
    .code = code,
    .value = value,
    .timestamp_us = timestamp_us,
  };

  PBL_ITERABLE_SECTION_FOREACH(const struct pbl_input_callback, input_cbs, cb) {
    cb->callback(&evt, cb->user_data);
  }
}
