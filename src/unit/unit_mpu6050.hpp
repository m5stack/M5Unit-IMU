/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6050.hpp
  @brief UnitMPU6050 (TDK InvenSense 6-axis IMU)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_MPU6050_HPP
#define M5_UNIT_IMU_UNIT_UNIT_MPU6050_HPP

#include "unit_mpu6xxx.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitMPU6050
  @brief TDK InvenSense MPU-6050 6-axis IMU (accel + gyro + temp). WHO_AM_I 0x68.
 */
class UnitMPU6050 : public UnitMPU6xxx {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMPU6050, 0x68);

public:
    explicit UnitMPU6050(const uint8_t addr = DEFAULT_ADDRESS) : UnitMPU6xxx(addr)
    {
    }

protected:
    inline bool accepts_whoami(const uint8_t id) const override
    {
        return id == 0x68;
    }
    inline float temperature_res() const override
    {
        return 1.0f / 340.0f;
    }
    inline float temperature_offset() const override
    {
        return 36.53f;
    }
};

}  // namespace unit
}  // namespace m5
#endif
