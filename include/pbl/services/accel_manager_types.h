/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <pbl/kernel/compiler.h>

/** @brief Valid accelerometer sampling rates, in Hz */
typedef enum {
  /** 10 HZ sampling rate */
  ACCEL_SAMPLING_10HZ = 10,
  /** 25 HZ sampling rate [Default] */
  ACCEL_SAMPLING_25HZ = 25,
  /** 50 HZ sampling rate */
  ACCEL_SAMPLING_50HZ = 50,
  /** 100 HZ sampling rate */
  ACCEL_SAMPLING_100HZ = 100,
} AccelSamplingRate;

/** @brief A single accelerometer sample for all three axes */
typedef struct PBL_PACKED {
  /** acceleration along the x axis */
  int16_t x;
  /** acceleration along the y axis */
  int16_t y;
  /** acceleration along the z axis */
  int16_t z;
} AccelRawData;

/**
 * @brief A single accelerometer sample for all three axes including timestamp and
 * vibration rumble status.
 */
typedef struct PBL_PACKED AccelData {
  /** acceleration along the x axis */
  int16_t x;
  /** acceleration along the y axis */
  int16_t y;
  /** acceleration along the z axis */
  int16_t z;

  /** true if the watch vibrated when this sample was collected */
  bool did_vibrate;

  /** timestamp, in milliseconds */
  uint64_t timestamp;
} AccelData;

/** @brief IMU sampling rates, in Hz (accelerometer, and gyroscope when enabled) */
typedef enum {
  /** 104 HZ sampling rate */
  IMU_SAMPLING_104HZ = 104,
  /** 208 HZ sampling rate */
  IMU_SAMPLING_208HZ = 208,
  /** 416 HZ sampling rate (requires the gyroscope) */
  IMU_SAMPLING_416HZ = 416,
} ImuSamplingRate;

/**
 * @brief IMU data service configuration, see imu_data_service_subscribe_config().
 * Zero-initialize it (designated initializers), then set what you need: every member left at 0
 * keeps its default, so members added in later SDKs leave existing apps unchanged. New members
 * are only ever appended.
 */
typedef struct {
  /** Number of samples to buffer before the handler is called (required) */
  uint32_t samples_per_update;
  /** Sampling rate (required) */
  ImuSamplingRate rate;
  /** Also sample the gyroscope (higher power use). Default: off */
  bool gyro;
  /** Accelerometer full-scale range in g: 2, 4, 8 or 16, rounded up to one the sensor supports.
   * A larger range has a coarser resolution. Default (0): the system range */
  uint8_t accel_range_g;
} ImuConfig;

/**
 * @brief One IMU sample, with the time the sensor acquired it.
 */
typedef struct PBL_PACKED ImuData {
  /** acceleration along the x axis, in milli-g */
  int16_t ax;
  /** acceleration along the y axis, in milli-g */
  int16_t ay;
  /** acceleration along the z axis, in milli-g */
  int16_t az;
  /** angular rate around the x axis, in 0.1 degrees per second (0 if the gyroscope is off) */
  int16_t gx;
  /** angular rate around the y axis, in 0.1 degrees per second */
  int16_t gy;
  /** angular rate around the z axis, in 0.1 degrees per second */
  int16_t gz;
  /** acquisition time from the sensor's own timestamp, in microseconds on the time_us() clock
   * (since boot, monotonic: setting the time never moves it) */
  uint64_t timestamp_us;
} ImuData;
