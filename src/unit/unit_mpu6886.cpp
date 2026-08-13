/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6886.cpp
  @brief UnitMPU6886 static members and Wake-on-Motion implementation
*/
#include "unit_mpu6886.hpp"
#include <M5Utility.hpp>

namespace m5 {
namespace unit {
using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::mpu6886::command;

const char UnitMPU6886::name[] = "UnitMPU6886";
const types::uid_t UnitMPU6886::uid{"UnitMPU6886"_mmh3};
const types::attr_t UnitMPU6886::attr{attribute::AccessI2C};

namespace {
// WoM registers per MPU-6886 datasheet section 5.1
constexpr uint8_t REG_ACCEL_WOM_X_THR{0x20};
constexpr uint8_t REG_ACCEL_WOM_Y_THR{0x21};
constexpr uint8_t REG_ACCEL_WOM_Z_THR{0x22};
constexpr uint8_t REG_ACCEL_INTEL_CTRL{0x69};

constexpr uint8_t INT_ENABLE_WOM_XYZ{0xE0};             // WOM_X_INT_EN | WOM_Y_INT_EN | WOM_Z_INT_EN
constexpr uint8_t ACCEL_INTEL_CTRL_EN_MODE_ON{0xC0};    // ACCEL_INTEL_EN | ACCEL_INTEL_MODE
constexpr uint8_t PWR_MGMT_1_CYCLE_BIT{0x20};           // CYCLE = 1 (accel low-power mode)
constexpr uint8_t PWR_MGMT_2_STBY_XG_YG_ZG{0x07};       // STBY_XG | STBY_YG | STBY_ZG (gyro standby)
constexpr uint8_t ACCEL_CONFIG2_LPF_218HZ{0x01};        // A_DLPF_CFG = 1 (218.1 Hz)
constexpr uint8_t INT_STATUS_WOM_INT_MASK{0xE0};        // WOM_X_INT | WOM_Y_INT | WOM_Z_INT
}  // namespace

bool UnitMPU6886::enableEvent(m5::imu::MotionEventKind kind, const void* cfg)
{
    if (kind != m5::imu::MotionEventKind::AnyMotion) return false;
    if (cfg == nullptr) return false;
    _wom_cfg = *static_cast<const m5::imu::AnyMotionConfig*>(cfg);
    // Chip register programming is intentionally staged; this call only tracks state.
    _enabled_mask |= m5::imu::MotionEventKind::AnyMotion;
    return true;
}

bool UnitMPU6886::disableEvent(m5::imu::MotionEventKind kind)
{
    if (kind != m5::imu::MotionEventKind::AnyMotion) return false;
    // Pre-state restore is intentionally staged; this call only clears the tracker.
    _enabled_mask &= ~m5::imu::MotionEventKind::AnyMotion;
    return true;
}

bool UnitMPU6886::readEventStatus(m5::imu::MotionEventStatus& out)
{
    if (!m5::imu::any(_enabled_mask & m5::imu::MotionEventKind::AnyMotion)) return false;
    uint8_t status{};
    if (!readRegister8(REG_INT_STATUS, status, 0)) return false;
    if (status & INT_STATUS_WOM_INT_MASK) {
        out.fired |= m5::imu::MotionEventKind::AnyMotion;
        return true;
    }
    return false;
}

}  // namespace unit
}  // namespace m5
