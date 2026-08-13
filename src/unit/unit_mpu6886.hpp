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

    ///@name MotionEventSource (Wake-on-Motion)
    ///@{
    //! @brief Reports AnyMotion capability (WoM per datasheet section 5.1).
    m5::imu::MotionEventKind motionEventSpec() const override
    {
        return m5::imu::MotionEventKind::AnyMotion;
    }
    m5::imu::MotionEventKind enabledEvents() const override
    {
        return _enabled_mask;
    }
    //! @brief Enable AnyMotion. cfg must point to a valid AnyMotionConfig.
    bool enableEvent(m5::imu::MotionEventKind kind, const void* cfg) override;
    //! @brief Disable AnyMotion and restore the pre-WoM chip state.
    bool disableEvent(m5::imu::MotionEventKind kind) override;
    //! @brief Poll INT_STATUS and clear pending WoM flags into out.fired.
    bool readEventStatus(m5::imu::MotionEventStatus& out) override;
    ///@}

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

private:
    struct PreWomSnapshot {
        uint8_t pwr_mgmt_1{0};
        uint8_t pwr_mgmt_2{0};
        uint8_t accel_config2{0};
        uint8_t smplrt_div{0};
        uint8_t int_enable{0};
        uint8_t accel_intel_ctrl{0};
        bool captured{false};
    };
    PreWomSnapshot _pre_wom{};
    m5::imu::MotionEventKind _enabled_mask{m5::imu::MotionEventKind::None};
    m5::imu::AnyMotionConfig _wom_cfg{};
};

}  // namespace unit
}  // namespace m5
#endif
