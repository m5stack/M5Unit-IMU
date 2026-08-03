/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6886.cpp
  @brief UnitMPU6886 static members
*/
#include "unit_mpu6886.hpp"
#include <M5Utility.hpp>

namespace m5 {
namespace unit {
using namespace m5::utility::mmh3;
using namespace m5::unit::types;

const char UnitMPU6886::name[] = "UnitMPU6886";
const types::uid_t UnitMPU6886::uid{"UnitMPU6886"_mmh3};
const types::attr_t UnitMPU6886::attr{attribute::AccessI2C};

}  // namespace unit
}  // namespace m5
