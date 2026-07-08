/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file M5UnitUnifiedIMU.hpp
  @brief Main header of M5UnitUnifiedIMU

  @mainpage M5Unit-IMU
  M5UnitUnified library for IMU series (UnitIMUMini, Unit DoF6/DoF9/DoF10, Unit BMM350, built-in IMUs)
*/
#ifndef M5_UNIT_UNIFIED_IMU_HPP
#define M5_UNIT_UNIFIED_IMU_HPP

#include "imu/imu_types.hpp"
#include "imu/imu_data_source.hpp"
#include "unit/unit_mpu6886.hpp"

/*!
  @namespace m5
  @brief Top level namespace of M5Stack
 */
namespace m5 {

/*!
  @namespace unit
  @brief Unit-related namespace
 */
namespace unit {

//! @brief Alias for UnitIMUMini (MPU6886)
using UnitIMUMini = m5::unit::UnitMPU6886;

}  // namespace unit
}  // namespace m5
#endif
