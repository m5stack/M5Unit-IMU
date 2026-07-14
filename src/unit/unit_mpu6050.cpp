/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6050.cpp
  @brief UnitMPU6050 statics
*/
#include "unit_mpu6050.hpp"
#include <M5Utility.hpp>

namespace m5 {
namespace unit {
using namespace m5::utility::mmh3;
using namespace m5::unit::types;

const char UnitMPU6050::name[] = "UnitMPU6050";
const types::uid_t UnitMPU6050::uid{"UnitMPU6050"_mmh3};
const types::attr_t UnitMPU6050::attr{attribute::AccessI2C};

}  // namespace unit
}  // namespace m5
