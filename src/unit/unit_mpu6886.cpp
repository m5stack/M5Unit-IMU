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

constexpr uint8_t INT_ENABLE_WOM_XYZ{0xE0};           // WOM_X_INT_EN | WOM_Y_INT_EN | WOM_Z_INT_EN
constexpr uint8_t ACCEL_INTEL_CTRL_EN_MODE_ON{0xC0};  // ACCEL_INTEL_EN | ACCEL_INTEL_MODE
constexpr uint8_t PWR_MGMT_1_CYCLE_BIT{0x20};         // CYCLE = 1 (accel low-power mode)
constexpr uint8_t PWR_MGMT_1_SLEEP_BIT{0x40};         // SLEEP bit, cleared before enabling cycle mode
constexpr uint8_t PWR_MGMT_2_STBY_XG_YG_ZG{0x07};     // STBY_XG | STBY_YG | STBY_ZG (gyro standby)
constexpr uint8_t ACCEL_CONFIG2_LPF_218HZ{0x01};      // A_DLPF_CFG = 1 (218.1 Hz)
constexpr uint8_t INT_STATUS_WOM_INT_MASK{0xE0};      // WOM_X_INT | WOM_Y_INT | WOM_Z_INT
}  // namespace

bool UnitMPU6886::enableEvent(m5::imu::MotionEventKind kind, const void* cfg)
{
    if (kind != m5::imu::MotionEventKind::AnyMotion) return false;
    if (cfg == nullptr) return false;
    _wom_cfg = *static_cast<const m5::imu::AnyMotionConfig*>(cfg);

    // Snapshot the pre-WoM chip state so disableEvent() can restore it later
    if (!_pre_wom.captured) {
        if (!readRegister8(REG_PWR_MGMT_1, _pre_wom.pwr_mgmt_1, 0)) return false;
        if (!readRegister8(REG_PWR_MGMT_2, _pre_wom.pwr_mgmt_2, 0)) return false;
        if (!readRegister8(REG_ACCEL_CONFIG2, _pre_wom.accel_config2, 0)) return false;
        if (!readRegister8(REG_SMPLRT_DIV, _pre_wom.smplrt_div, 0)) return false;
        if (!readRegister8(REG_INT_ENABLE, _pre_wom.int_enable, 0)) return false;
        if (!readRegister8(REG_ACCEL_INTEL_CTRL, _pre_wom.accel_intel_ctrl, 0)) return false;
        _pre_wom.captured = true;
    }

    // Step 1: ensure the accelerometer is running (gyro may be put into standby)
    if (!writeRegister8(REG_PWR_MGMT_2, _pre_wom.pwr_mgmt_2 | PWR_MGMT_2_STBY_XG_YG_ZG)) return false;

    // Step 2: set accelerometer LPF bandwidth to 218.1 Hz
    if (!writeRegister8(REG_ACCEL_CONFIG2, ACCEL_CONFIG2_LPF_218HZ)) return false;

    // Step 3: enable the motion interrupt on all three axes
    if (!writeRegister8(REG_INT_ENABLE, INT_ENABLE_WOM_XYZ)) return false;

    // Step 4: set the motion threshold (mg -> LSB at 4 mg/LSB); disabled axes get the 0xFF
    // sentinel so they never trip the wake-on-motion condition
    const uint16_t thr_lsb = _wom_cfg.threshold_mg / 4;
    const uint8_t thr8     = static_cast<uint8_t>(thr_lsb > 0xFF ? 0xFF : thr_lsb);
    if (!writeRegister8(REG_ACCEL_WOM_X_THR, _wom_cfg.axis_x ? thr8 : 0xFF)) return false;
    if (!writeRegister8(REG_ACCEL_WOM_Y_THR, _wom_cfg.axis_y ? thr8 : 0xFF)) return false;
    if (!writeRegister8(REG_ACCEL_WOM_Z_THR, _wom_cfg.axis_z ? thr8 : 0xFF)) return false;

    // Step 5: enable the accelerometer hardware intelligence (compare-to-previous-sample mode)
    if (!writeRegister8(REG_ACCEL_INTEL_CTRL, ACCEL_INTEL_CTRL_EN_MODE_ON)) return false;

    // Step 7: set the wake-up frequency to ~20 Hz (SMPLRT_DIV = 1000/(1+div) - 1, div=49)
    if (!writeRegister8(REG_SMPLRT_DIV, 49)) return false;

    // Step 8: enable cycle mode (accelerometer low-power mode)
    if (!writeRegister8(REG_PWR_MGMT_1, (_pre_wom.pwr_mgmt_1 & ~PWR_MGMT_1_SLEEP_BIT) | PWR_MGMT_1_CYCLE_BIT))
        return false;

    _enabled_mask |= m5::imu::MotionEventKind::AnyMotion;
    return true;
}

bool UnitMPU6886::disableEvent(m5::imu::MotionEventKind kind)
{
    if (kind != m5::imu::MotionEventKind::AnyMotion) return false;

    // Turn off the WoM interrupt and accelerometer hardware intelligence first
    if (!writeRegister8(REG_INT_ENABLE, 0x00)) return false;
    if (!writeRegister8(REG_ACCEL_INTEL_CTRL, 0x00)) return false;

    // Restore the pre-WoM chip state; keep the snapshot on failure so the caller can retry
    if (_pre_wom.captured) {
        if (!writeRegister8(REG_PWR_MGMT_1, _pre_wom.pwr_mgmt_1)) return false;
        if (!writeRegister8(REG_PWR_MGMT_2, _pre_wom.pwr_mgmt_2)) return false;
        if (!writeRegister8(REG_ACCEL_CONFIG2, _pre_wom.accel_config2)) return false;
        if (!writeRegister8(REG_SMPLRT_DIV, _pre_wom.smplrt_div)) return false;
        if (!writeRegister8(REG_INT_ENABLE, _pre_wom.int_enable)) return false;
        if (!writeRegister8(REG_ACCEL_INTEL_CTRL, _pre_wom.accel_intel_ctrl)) return false;
        _pre_wom.captured = false;
    }

    _enabled_mask &= ~m5::imu::MotionEventKind::AnyMotion;
    return true;
}

bool UnitMPU6886::readEventStatus(m5::imu::MotionEventStatus& out)
{
    if (!m5::imu::any(_enabled_mask & m5::imu::MotionEventKind::AnyMotion)) return false;
    // Drain WoM bits latched by read_measurement() (INT_STATUS is read-clear, so the periodic
    // measurement task would otherwise consume them before we get here).
    uint8_t combined = _pending_wom_status;
    _pending_wom_status = 0;
    // Also try a direct read in case no periodic read has happened yet (e.g. WoM enabled after
    // stopPeriodicMeasurement, or on the very first call). This is harmless when combined with
    // the latched bits.
    uint8_t status{};
    if (readRegister8(REG_INT_STATUS, status, 0)) {
        combined |= static_cast<uint8_t>(status & INT_STATUS_WOM_INT_MASK);
        // Preserve any WoM bits we just consumed here in the latch too (paranoia; usually zero
        // because the periodic task races with us and clears them first).
    }
    if (combined & INT_STATUS_WOM_INT_MASK) {
        out.fired |= m5::imu::MotionEventKind::AnyMotion;
        return true;
    }
    return false;
}

}  // namespace unit
}  // namespace m5
