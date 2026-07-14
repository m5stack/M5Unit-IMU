/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6886.hpp
  @brief UnitMPU6886 (TDK InvenSense 6-axis IMU). UnitMiniIMU and Core2/StickC built-in.
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_MPU6886_HPP
#define M5_UNIT_IMU_UNIT_UNIT_MPU6886_HPP

#include "unit_mpu6xxx.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitMPU6886
  @brief TDK InvenSense MPU-6886 6-axis IMU (accel + gyro + temp). WHO_AM_I 0x19.
 */
class UnitMPU6886 : public UnitMPU6xxx {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMPU6886, 0x68);

public:
    explicit UnitMPU6886(const uint8_t addr = DEFAULT_ADDRESS) : UnitMPU6xxx(addr)
    {
    }

protected:
    inline bool accepts_whoami(const uint8_t id) const override
    {
        return id == 0x19;
    }
    inline float temperature_res() const override
    {
        return 1.0f / 326.8f;
    }
    inline float temperature_offset() const override
    {
        return 25.0f;
    }
};

}  // namespace unit
}  // namespace m5
#endif
