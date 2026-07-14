/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6500.hpp
  @brief UnitMPU6500 (TDK InvenSense 6-axis IMU core)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_MPU6500_HPP
#define M5_UNIT_IMU_UNIT_UNIT_MPU6500_HPP

#include "unit_mpu6xxx.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitMPU6500
  @brief TDK InvenSense MPU-6500 6-axis core (accel + gyro + temp)
  @details WHO_AM_I 0x70 standalone, 0x71 as the accel/gyro die of an MPU-9250 (both share the
  1/333.87 degC/LSB + 21 degC temperature scale).
 */
class UnitMPU6500 : public UnitMPU6xxx {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMPU6500, 0x68);

public:
    explicit UnitMPU6500(const uint8_t addr = DEFAULT_ADDRESS) : UnitMPU6xxx(addr)
    {
    }

protected:
    // 0x70 = standalone MPU-6500; 0x71 = the MPU-6500-class accel/gyro die inside an MPU-9250.
    inline bool accepts_whoami(const uint8_t id) const override
    {
        return id == 0x70 || id == 0x71;
    }
    inline float temperature_res() const override
    {
        return 1.0f / 333.87f;
    }
    inline float temperature_offset() const override
    {
        return 21.0f;
    }
};

}  // namespace unit
}  // namespace m5
#endif
