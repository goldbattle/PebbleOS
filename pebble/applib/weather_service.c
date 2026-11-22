/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "weather_service.h"

#include <syscall/syscall.h>

#if defined(CONFIG_SERVICE_WEATHER)
#include <pbl/services/weather/weather_service.h>
#endif

bool weather_service_peek_current_temp(int *temp_out) {
#if defined(CONFIG_SERVICE_WEATHER)
  const int temp = sys_weather_get_current_temp();
  if (temp == WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP) {
    return false;
  }
  if (temp_out) {
    *temp_out = temp;
  }
  return true;
#else
  return false;
#endif
}
