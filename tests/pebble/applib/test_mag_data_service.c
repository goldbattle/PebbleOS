/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// mag_data_service_subscribe(), the app side: what an app asks for is clamped and handed to the
// kernel, and each PEBBLE_MAG_SAMPLE_EVENT hands the queued samples to the app's handler in batches
// until the kernel ring is empty. The kernel side is faked here: a queue of samples behind
// sys_ecompass_mag_get_samples().

#include <applib/compass_service.h>
#include <applib/compass_service_private.h>
#include <applib/event_service_client.h>
#include <kernel/events.h>
#include <kernel/pebble_tasks.h>
#include <process_state/app_state/app_state.h>

#include <clar.h>

#include <stdlib.h>
#include <string.h>

// ---- fakes ---------------------------------------------------------------------------------

static PebbleTask s_task = PebbleTask_App;
PebbleTask pebble_task_get_current(void) {
  return s_task;
}

static MagServiceConfig *s_mag_config;
MagServiceConfig **app_state_get_mag_config(void) {
  return &s_mag_config;
}

// The kernel: the rate it grants (0: no magnetometer), what it was asked, and its sample queue
static uint32_t s_granted_hz;
static uint32_t s_asked_hz, s_asked_per_update;
static uint32_t s_queued, s_next_x;

uint32_t sys_ecompass_mag_request(uint32_t rate_hz, uint32_t samples_per_update) {
  s_asked_hz = rate_hz;
  s_asked_per_update = samples_per_update;
  return s_granted_hz == 0 ? 0 : (rate_hz < s_granted_hz ? rate_hz : s_granted_hz);
}

uint32_t sys_ecompass_mag_get_samples(uint32_t max, MagSample *samples) {
  const uint32_t n = s_queued < max ? s_queued : max;
  for (uint32_t i = 0; i < n; i++) {
    const uint32_t x = s_next_x++;
    samples[i] = (MagSample){.x = (int16_t)x, .timestamp_us = 1000U * x};
  }
  s_queued -= n;
  return n;
}

// The compass heading half of compass_service.c, not used here
bool sys_ecompass_service_subscribed(void) {
  return false;
}
void sys_ecompass_get_last_heading(CompassHeadingData *data) {
}
CompassServiceConfig **app_state_get_compass_config(void) {
  return nullptr;
}
CompassServiceConfig **worker_state_get_compass_config(void) {
  return nullptr;
}
CompassServiceConfig **kernel_applib_get_compass_config(void) {
  return nullptr;
}

static EventServiceInfo *s_subscribed;
void event_service_client_subscribe(EventServiceInfo *info) {
  s_subscribed = info;
}
void event_service_client_unsubscribe(EventServiceInfo *info) {
  if (s_subscribed == info) {
    s_subscribed = nullptr;
  }
}

static void prv_post_event(void) {
  cl_assert(s_subscribed != nullptr);
  PebbleEvent e = {.type = PEBBLE_MAG_SAMPLE_EVENT};
  s_subscribed->handler(&e, s_subscribed->context);
}

// ---- the app's handler -----------------------------------------------------------------------

#define MAX_BATCHES 8
static uint32_t s_batch_sizes[MAX_BATCHES];
static int16_t s_batch_first_x[MAX_BATCHES];
static int s_batches;
static bool s_unsubscribe_in_handler;

static void prv_handler(MagSample *samples, uint32_t num) {
  cl_assert(s_batches < MAX_BATCHES);
  s_batch_sizes[s_batches] = num;
  s_batch_first_x[s_batches] = samples[0].x;
  s_batches++;
  if (s_unsubscribe_in_handler) {
    mag_data_service_unsubscribe();
  }
}

void test_mag_data_service__initialize(void) {
  s_task = PebbleTask_App;
  s_mag_config = nullptr;
  s_subscribed = nullptr;
  s_granted_hz = 100;
  s_asked_hz = s_asked_per_update = 0;
  s_queued = s_next_x = 0;
  s_batches = 0;
  s_unsubscribe_in_handler = false;
}

void test_mag_data_service__cleanup(void) {
  mag_data_service_unsubscribe();
}

// 30 samples queued, batches of 13: the handler gets 13, 13, then the 4 left, in order.
void test_mag_data_service__an_event_drains_the_queue_in_batches(void) {
  cl_assert_equal_i(mag_data_service_subscribe(13, 100, prv_handler), 100);
  cl_assert_equal_i(s_asked_hz, 100);
  cl_assert_equal_i(s_asked_per_update, 13);

  s_queued = 30;
  prv_post_event();
  cl_assert_equal_i(s_batches, 3);
  cl_assert_equal_i(s_batch_sizes[0], 13);
  cl_assert_equal_i(s_batch_sizes[1], 13);
  cl_assert_equal_i(s_batch_sizes[2], 4);
  cl_assert_equal_i(s_batch_first_x[1], 13);
  cl_assert_equal_i(s_batch_first_x[2], 26);
  cl_assert_equal_i(s_queued, 0);
}

// The batch size is clamped to 1..64 (the kernel ring holds two batches).
void test_mag_data_service__batch_size_is_clamped(void) {
  mag_data_service_subscribe(500, 25, prv_handler);
  cl_assert_equal_i(s_asked_per_update, MAG_SAMPLES_PER_UPDATE_MAX);
  mag_data_service_subscribe(0, 25, prv_handler);
  cl_assert_equal_i(s_asked_per_update, 1);
}

// A handler may unsubscribe while handling a batch: no further batches, nothing used after free.
void test_mag_data_service__handler_may_unsubscribe(void) {
  mag_data_service_subscribe(10, 100, prv_handler);
  s_unsubscribe_in_handler = true;
  s_queued = 25;
  prv_post_event();
  cl_assert_equal_i(s_batches, 1);
  cl_assert(s_mag_config == nullptr);
  cl_assert(s_subscribed == nullptr);
}

// Subscribing again replaces the previous subscription.
void test_mag_data_service__subscribing_again_replaces_the_handler_and_batch(void) {
  mag_data_service_subscribe(10, 100, prv_handler);
  EventServiceInfo *first = s_subscribed;
  mag_data_service_subscribe(5, 50, prv_handler);
  cl_assert(s_subscribed != nullptr);
  cl_assert_equal_i(s_asked_per_update, 5);
  s_queued = 12;
  prv_post_event();
  cl_assert_equal_i(s_batches, 3);  // 5, 5, 2
  cl_assert_equal_i(s_batch_sizes[2], 2);
  (void)first;
}

// No magnetometer (the kernel grants 0 Hz), or not an app: 0, and nothing subscribed.
void test_mag_data_service__no_magnetometer_or_not_an_app(void) {
  s_granted_hz = 0;
  cl_assert_equal_i(mag_data_service_subscribe(10, 100, prv_handler), 0);
  cl_assert(s_subscribed == nullptr);

  s_granted_hz = 100;
  s_task = PebbleTask_Worker;
  cl_assert_equal_i(mag_data_service_subscribe(10, 100, prv_handler), 0);
  cl_assert(s_subscribed == nullptr);
}
