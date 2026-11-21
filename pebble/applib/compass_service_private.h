/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include "compass_service.h"

#include <applib/app_timer.h>
#include <applib/event_service_client.h>

typedef struct {
  CompassHeading compass_filter;
  int32_t last_angle;
  CompassHeading heading_declination;

  AppTimer *peek_timer;
  CompassHeadingHandler compass_cb;

  EventServiceInfo info;
} CompassServiceConfig;

typedef struct {
  MagDataHandler handler;
  uint32_t samples_per_update;
  MagSample *samples;  //!< app heap buffer the batches are copied into
  EventServiceInfo info;
} MagServiceConfig;

//! Limits of mag_data_service_subscribe()
#define MAG_RATE_HZ_MAX 100U
#define MAG_SAMPLES_PER_UPDATE_MAX 64U

//! Record what the current task wants from mag_data_service_subscribe(); takes effect when it
//! subscribes to PEBBLE_MAG_SAMPLE_EVENT.
//! @return The rate in use, or 0 if there is no magnetometer.
uint32_t sys_ecompass_mag_request(uint32_t rate_hz, uint32_t samples_per_update);

//! Move up to max queued samples of the current task into the buffer.
//! @return the number of samples copied.
uint32_t sys_ecompass_mag_get_samples(uint32_t max, MagSample *samples);
