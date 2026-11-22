/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// click_recognizer_get_press_time_us(): a click handler learns when the button was physically
// pressed, not when the handler happens to run (a single click with a long click configured fires at
// the release, a long click after its delay).

#include <applib/ui/click.h>
#include <applib/ui/click_internal.h>

#include <clar.h>

// The recognizer reads the time with this syscall
static uint64_t s_now_us;
uint64_t sys_get_time_us(void) {
  return s_now_us;
}

// The pending hold timer, fired by the test
static AppTimerCallback s_timer_cb;
static void *s_timer_data;
static int s_timer;

AppTimer *app_timer_register(uint32_t timeout_ms, AppTimerCallback callback, void *callback_data) {
  s_timer_cb = callback;
  s_timer_data = callback_data;
  return (AppTimer *)&s_timer;
}

void app_timer_cancel(AppTimer *timer) {
  s_timer_cb = nullptr;
}

static void prv_fire_timer(void) {
  const AppTimerCallback cb = s_timer_cb;
  cl_assert(cb != nullptr);
  s_timer_cb = nullptr;
  cb(s_timer_data);
}

static ClickManager s_manager;
static ClickRecognizer *s_select;
static uint64_t s_single_us, s_long_us;
static int s_singles, s_longs;

static void prv_single(ClickRecognizerRef recognizer, void *context) {
  s_single_us = click_recognizer_get_press_time_us(recognizer);
  s_singles++;
}

static void prv_long(ClickRecognizerRef recognizer, void *context) {
  s_long_us = click_recognizer_get_press_time_us(recognizer);
  s_longs++;
}

void test_click_press_time__initialize(void) {
  click_manager_init(&s_manager);
  s_select = &s_manager.recognizers[BUTTON_ID_SELECT];
  s_select->config = (ClickConfig){
    .click.handler = prv_single,
    .long_click = {.handler = prv_long, .delay_ms = 700},
  };
  s_singles = s_longs = 0;
  s_single_us = s_long_us = 0;
  s_timer_cb = nullptr;
}

// Pressed at t, the event handled 20 ms later, released at t + 180 ms. With a long click configured
// the single click only fires at the release, yet it reports the press.
void test_click_press_time__single_click_reports_the_press_not_the_release(void) {
  const uint64_t pressed = 1795313518000000ULL;
  s_now_us = pressed + 20000;
  click_recognizer_handle_button_down_at(s_select, (uint32_t)pressed);
  s_now_us = pressed + 180000;
  click_recognizer_handle_button_up(s_select);
  cl_assert_equal_i(s_singles, 1);
  cl_assert(s_single_us == pressed);
}

// The long click fires 700 ms into the hold and reports the press.
void test_click_press_time__long_click_reports_the_press(void) {
  const uint64_t pressed = 1795313518000000ULL;
  s_now_us = pressed + 5000;
  click_recognizer_handle_button_down_at(s_select, (uint32_t)pressed);
  s_now_us = pressed + 700000;
  prv_fire_timer();  // the long click delay
  cl_assert_equal_i(s_longs, 1);
  cl_assert(s_long_us == pressed);
}

// Button events carry only the low 32 bits of the press time. If they wrapped (every ~71.6 min)
// between the press and its handling, the press is still the latest time with those bits that is
// not after now.
void test_click_press_time__press_just_before_the_low_bits_wrap(void) {
  const uint64_t pressed = (7ULL << 32) - 3000U;  // 3 ms before a wrap
  s_now_us = (7ULL << 32) + 12000U;               // handled 15 ms later, after the wrap
  click_recognizer_handle_button_down_at(s_select, (uint32_t)pressed);
  s_now_us += 100000U;
  click_recognizer_handle_button_up(s_select);
  cl_assert_equal_i(s_singles, 1);
  cl_assert(s_single_us == pressed);
}

// A press without an edge time (click_recognizer_handle_button_down) is taken to happen now.
void test_click_press_time__press_without_an_edge_time_is_now(void) {
  s_now_us = 1795313518123456ULL;
  click_recognizer_handle_button_down(s_select);
  s_now_us += 90000U;
  click_recognizer_handle_button_up(s_select);
  cl_assert(s_single_us == 1795313518123456ULL);
}
