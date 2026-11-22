/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>

//! @addtogroup Foundation
//! @{
//!   @addtogroup WeatherService
//!
//! rief The current temperature from the phone's weather sync
//!
//! The phone app keeps the watch's Weather app up to date. This gives apps the current temperature
//! of its first location, in the units the phone sends (the user's choice). There is none when the
//! user turned weather off, has no location, or the data is more than a day old.
//! @{

//! Peek at the current temperature.
//! @param temp_out Set to the temperature (degrees, in the phone's units) when there is one
//! @return true if there is a current temperature
bool weather_service_peek_current_temp(int *temp_out);

//!   @} // end addtogroup WeatherService
//! @} // end addtogroup Foundation
