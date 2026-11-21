/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/drivers/mag.h>
#include <pbl/services/ecompass.h>

//! @addtogroup Foundation
//! @{
//!   @addtogroup EventService
//!   @{
//!     @addtogroup CompassService
//!
//!     \brief The Compass Service combines information from Pebble's accelerometer and
//!     magnetometer to automatically calibrate
//!     the compass and transform the raw magnetic field information into a \ref CompassHeading,
//!     that is an angle to north. It also
//!     provides magnetic north and information about its status and accuracy through the \ref
//!     CompassHeadingData structure. The API is designed to also provide true north in a future
//!     release.
//!
//!     Note that not all platforms have compasses. To check for the presence of a compass at
//!     compile time for the current platform use the `PBL_COMPASS` define.
//!
//!     To learn more about the Compass Service and how to use it, read the
//!     <a href="https://developer.getpebble.com/guides/pebble-apps/sensors/magnetometer/">
//!     Determining Direction</a> guide.
//!
//!     For available code samples, see the
//!     <a href="https://github.com/pebble-examples/feature-compass">feature-compass</a> example.
//!
//!       @{

//! Peek at the last recorded reading.
//! @param[out] data a pointer to a pre-allocated CompassHeadingData
//! @return Always returns 0 to indicate success.
int compass_service_peek(CompassHeadingData *data);

//! Set the minimum angular change required to generate new compass heading events.
//! The angular distance is measured relative to the last delivered heading event.
//! Use 0 to be notified of all movements.
//! Negative values and values > TRIG_MAX_ANGLE / 2 are not valid.
//! The default value of this property is TRIG_MAX_ANGLE / 360.
//! @return 0, success.
//! @return Non-Zero, if filter is invalid.
//! @see compass_service_subscribe
int compass_service_set_heading_filter(CompassHeading filter);

//! Callback type for compass heading events
//! @param heading copy of last recorded heading
typedef void (*CompassHeadingHandler)(CompassHeadingData heading);

//! One magnetometer sample in the watch frame, units of 0.1 microtesla (= milligauss)
typedef struct PBL_PACKED MagSample {
  int16_t x, y, z;
  //! Sensor temperature in 0.01 degrees C, INT16_MIN if unavailable
  int16_t temp_c100;
  //! 1 if the compass service reports CompassStatusCalibrated. The calibration only runs while
  //! some process is subscribed to the compass service, otherwise this stays 0.
  uint8_t calibrated;
  //! Measurement time on the time_us() clock, accurate to about +-(half the sensor polling
  //! period), 5 ms at 25 Hz
  uint64_t timestamp_us;
} MagSample;

//! Callback type for batches of raw magnetometer samples
//! @param samples the samples, oldest first, valid for the duration of the call
//! @param num_samples the number of samples
typedef void (*MagDataHandler)(MagSample *samples, uint32_t num_samples);

//! Batches of samples_per_update raw magnetometer samples at about rate_hz, each stamped with its
//! measurement time on the time_us() clock. Replaces any previous magnetometer data subscription.
//! The magnetometer keeps running at the requested rate (alongside the compass service if both
//! are used, at the faster of the two rates) until mag_data_service_unsubscribe() or app exit.
//! If the app is too slow to keep up, the newest samples are dropped.
//! @param samples_per_update samples per handler call, clipped to 1..64
//! @param rate_hz the requested rate in Hz, clipped to 1..100
//! @param handler the callback
//! @return the rate in use, or 0 if there is no magnetometer (or not called from an app)
uint32_t mag_data_service_subscribe(uint32_t samples_per_update, uint32_t rate_hz,
                                    MagDataHandler handler);

//! Unsubscribe from the magnetometer data service
void mag_data_service_unsubscribe(void);

//! Subscribe to the compass heading event service. Once subscribed, the handler
//! gets called every time the angular distance relative to the previous value
//! exceeds the configured filter.
//! @param handler A callback to be executed on heading events
//! @see compass_service_set_heading_filter
//! @see compass_service_unsubscribe
void compass_service_subscribe(CompassHeadingHandler handler);

//! Unsubscribe from the compass heading event service. Once unsubscribed,
//! the previously registered handler will no longer be called.
//! @see compass_service_subscribe
void compass_service_unsubscribe(void);

//!     @} // end addtogroup CompassService
//!   @} // end addtogroup EventService
//! @} // end addtogroup Foundation
