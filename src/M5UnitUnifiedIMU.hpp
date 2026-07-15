/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file M5UnitUnifiedIMU.hpp
  @brief Main header of M5UnitUnifiedIMU

  @mainpage M5Unit-IMU
  M5UnitUnified library for IMU series (UnitMiniIMU, Unit DoF6/DoF9/DoF10, Unit BMM350, built-in IMUs)
*/
#ifndef M5_UNIT_UNIFIED_IMU_HPP
#define M5_UNIT_UNIFIED_IMU_HPP

#include "imu/imu_types.hpp"
#include "imu/imu_data_source.hpp"
#include "imu/axis_order.hpp"
#include "imu/ahrs_controller.hpp"
#include "imu/presets.hpp"
#include "imu/mag_detect.hpp"
#include "imu/imu_detect.hpp"
#include "imu/filter/filter_base.hpp"
#include "imu/filter/complementary.hpp"
#include "imu/filter/madgwick.hpp"
#include "imu/filter/mahony.hpp"
#include "imu/filter/ekf.hpp"
#include "imu/filter/fusion.hpp"
#include "imu/filter/vqf.hpp"
#include "unit/unit_mpu6xxx.hpp"
#include "unit/unit_mpu6886.hpp"
#include "unit/unit_mpu6050.hpp"
#include "unit/unit_mpu6500.hpp"
#include "unit/unit_mpu9250.hpp"
#include "unit/unit_bmm150.hpp"
#include "unit/unit_bmm350.hpp"
#include "unit/unit_bmi270.hpp"
#include "unit/unit_ak8963.hpp"
#include "unit/unit_mag3110.hpp"
#include "unit/unit_dof9.hpp"
#include "unit/unit_spa06.hpp"

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

//! @brief Alias for UnitMiniIMU (MPU6886)
using UnitMiniIMU = m5::unit::UnitMPU6886;
//! @brief Unit DoF6 (U228): 6-axis BMI270
using UnitDoF6 = m5::unit::UnitBMI270;
// Add product-name aliases here as needed (e.g. UnitDoF10)

}  // namespace unit
}  // namespace m5
#endif
