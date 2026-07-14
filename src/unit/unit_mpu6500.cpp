/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6500.cpp
  @brief UnitMPU6500 statics
*/
#include "unit_mpu6500.hpp"
#include <M5Utility.hpp>

namespace m5 {
namespace unit {
using namespace m5::utility::mmh3;
using namespace m5::unit::types;

const char UnitMPU6500::name[] = "UnitMPU6500";
const types::uid_t UnitMPU6500::uid{"UnitMPU6500"_mmh3};
const types::attr_t UnitMPU6500::attr{attribute::AccessI2C};

}  // namespace unit
}  // namespace m5
