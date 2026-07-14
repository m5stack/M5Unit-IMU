/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu9250.hpp
  @brief UnitMPU9250: InvenSense MPU-9250 = MPU-6500 (accel/gyro) + AKM AK8963 (mag)
  @details Per PS-MPU-9250A, the MPU-9250 is a multi-chip module of two dies: an MPU-6500-class
  accel/gyro at 0x68 and an AK8963 magnetometer at 0x0C reached through the MPU-9250 aux-I2C bypass.
  This composite IS the accel/gyro (inherits UnitMPU6500) and owns a single UnitAK8963 as a framework
  child. begin() opens the aux bypass right after the inherited accel/gyro init; the framework then
  begins the mag child (bus shared via ensure_adapter()).
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_MPU9250_HPP
#define M5_UNIT_IMU_UNIT_UNIT_MPU9250_HPP

#include "unit_mpu6500.hpp"
#include "unit_ak8963.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitMPU9250
  @brief TDK InvenSense MPU-9250 9-axis IMU. Accel/gyro/temp (this unit itself) + AK8963 mag (mag()).
 */
class UnitMPU9250 : public UnitMPU6500 {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMPU9250, 0x68);

public:
    explicit UnitMPU9250(const uint8_t addr = DEFAULT_ADDRESS, const uint8_t mag_addr = 0x0C);
    virtual ~UnitMPU9250()
    {
    }

    //! @brief Magnetometer sub-unit (add to an AhrsController for Mag). Accel/gyro/temp is this unit itself.
    UnitAK8963& mag()
    {
        return _mag;
    }

    //! @brief Begin the accel/gyro die, then open the aux bypass. The framework begins the mag child next.
    virtual bool begin() override;

protected:
    //! @brief Only a real MPU9250's accel/gyro die (WHO_AM_I 0x71) is accepted here
    inline bool accepts_whoami(const uint8_t id) const override
    {
        return id == 0x71;
    }
    //! @brief Share this composite's bus with the mag child, duplicated at the child's address
    virtual std::shared_ptr<Adapter> ensure_adapter(const uint8_t ch) override;

private:
    UnitAK8963 _mag;
};

}  // namespace unit
}  // namespace m5
#endif
