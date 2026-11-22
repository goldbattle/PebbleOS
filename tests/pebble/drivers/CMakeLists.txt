# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_flash_api
  SOURCES
    drivers/base/device.c
    drivers/flash/flash.c
  DEFINES
    CONFIG_DEVICE_LOG_LEVEL=0
    CONFIG_DRIVER_FLASH_LOG_LEVEL=0
  OVERRIDES dummy_board
)

pbl_clar_test(test_spi_nor
  SOURCES
    drivers/base/device.c
    drivers/flash/flash.c
    drivers/flash/gd25lq255e.c
    drivers/flash/gd25q256e.c
    drivers/flash/py25q128ha.c
    drivers/flash/otp.c
    drivers/flash/spi_nor.c
    drivers/otp/otp_flash.c
    drivers/regulator/regulator.c
    drivers/spi_mem/spi_mem.c
  DEFINES
    CONFIG_DEVICE_LOG_LEVEL=0
    CONFIG_DRIVER_FLASH_LOG_LEVEL=0
    CONFIG_DRIVER_SPI_MEM_LOG_LEVEL=0
  OVERRIDES dummy_board
)

pbl_clar_test(test_accel_sim
  SOURCES
    drivers/base/device.c
    drivers/imu/accel.c
    drivers/imu/accel_sim.c
    tests/fakes/fake_rtc.c
  DEFINES
    CONFIG_DEVICE_LOG_LEVEL=0
  OVERRIDES dummy_board
)

pbl_clar_test(test_mag_sim
  SOURCES
    drivers/base/device.c
    drivers/imu/mag.c
    drivers/imu/mag_sim.c
    tests/fakes/fake_rtc.c
  DEFINES
    CONFIG_DEVICE_LOG_LEVEL=0
  OVERRIDES dummy_board
)

pbl_clar_test(test_lsm6dso
  SOURCES
    kernel/clocksync.c
    drivers/base/device.c
    drivers/imu/accel.c
    drivers/imu/lsm6dso/lsm6dso.c
  DEFINES
    CONFIG_DEVICE_LOG_LEVEL=0
    CONFIG_DRIVER_IMU_LOG_LEVEL=0
    CONFIG_ACCEL_LSM6DSO_SCALE_MG=2000
    CONFIG_ACCEL_LSM6DSO_WK_DUR_DEFAULT=0
    CONFIG_ACCEL_LSM6DSO_WK_THS_MIN=0
    CONFIG_ACCEL_LSM6DSO_WK_THS_MAX=63
    CONFIG_ACCEL_LSM6DSO_WK_THS_DEFAULT=0
    PBL_NO_LINKER_SCRIPT
  OVERRIDES dummy_board
)

pbl_clar_test(test_flash_partition
  SOURCES
    tests/fakes/fake_spi_flash.c
    tests/pebble/drivers/flash_partitions_single_slot.c
)

pbl_clar_test(test_mmc5603nj
  SOURCES
    kernel/clocksync.c
    drivers/base/device.c
    drivers/imu/mag.c
    drivers/imu/mmc5603nj/mmc5603nj.c
  DEFINES
    CONFIG_DEVICE_LOG_LEVEL=0
    CONFIG_DRIVER_IMU_LOG_LEVEL=0
    PBL_NO_LINKER_SCRIPT
  OVERRIDES dummy_board
)
