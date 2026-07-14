/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file mag_detect.hpp
  @brief Framework-agnostic detection of a classic M5Stack built-in magnetometer
  @details Classic M5Stack hosts (Basic/Gray/Fire and StickC) carry a
  version-dependent magnetometer that M5.getBoard() cannot distinguish: a BMM150
  at 0x10 (SH200Q/MPU6886 generations) or the AK8963 embedded in an MPU9250 at
  0x0C (original generation). These helpers probe the bus via injected callbacks
  so src/ stays independent of M5Unified / Arduino Wire (same pattern as
  m5::unit::bmi270::detectAddress). Note: m5::unit::imu here is distinct from the
  m5::imu namespace that holds the AHRS types.
*/
#ifndef M5_UNIT_IMU_IMU_MAG_DETECT_HPP
#define M5_UNIT_IMU_IMU_MAG_DETECT_HPP

#include <cstdint>

namespace m5 {
namespace unit {
namespace imu {

//! @brief Which built-in magnetometer a classic M5Stack host carries
enum class MagKind : uint8_t {
    None,    //!< @brief No magnetometer responded (e.g. an IMU-less M5Stack Basic)
    BMM150,  //!< @brief Bosch BMM150 at 0x10
    AK8963,  //!< @brief AKM AK8963 (MPU9250-internal) at 0x0C
};

/*!
  @brief Probe the internal bus for a built-in magnetometer
  @tparam ReadReg Callable `bool(uint8_t addr, uint8_t reg, uint8_t& out)` returning true on ACK
  @param read The bus read callable
  @return The detected magnetometer kind
  @details AK8963 is checked first by its WHO_AM_I (0x48, readable any time). BMM150 is then
  detected by presence at 0x10: its CHIP_ID reads 0x00 while the chip is suspended (power-on
  default), so an ACK at 0x10 is treated as BMM150 (UnitBMM150::begin() does the authoritative
  CHIP_ID check after powering the chip). For AK8963 to be visible at 0x0C behind an MPU9250,
  call enableAuxBypass() first.
 */
template <class ReadReg>
MagKind detectBuiltinMag(ReadReg read)
{
    uint8_t id{};
    if (read(static_cast<uint8_t>(0x0C), static_cast<uint8_t>(0x00), id) && id == 0x48) {
        return MagKind::AK8963;
    }
    if (read(static_cast<uint8_t>(0x10), static_cast<uint8_t>(0x40), id)) {
        return MagKind::BMM150;
    }
    return MagKind::None;
}

/*!
  @brief Open the MPU9250 I2C bypass so its internal AK8963 appears at 0x0C on the host bus
  @tparam ReadReg Callable `bool(uint8_t addr, uint8_t reg, uint8_t& out)`
  @tparam WriteReg Callable `bool(uint8_t addr, uint8_t reg, uint8_t val)`
  @param read The bus read callable
  @param write The bus write callable
  @return true if an MPU9250 was found and bypass enabled; false if no MPU9250 (a BMM150-direct
  board, or no IMU) -- in which case no registers are written
  @details Only the MPU9250 hides its magnetometer behind an aux I2C master; MPU6886/BMM150 boards
  expose the mag directly, so this is a no-op there. Requires the host IMU to not be concurrently
  owned (run with M5.config().internal_imu = false).
 */
template <class ReadReg, class WriteReg>
bool enableAuxBypass(ReadReg read, WriteReg write)
{
    uint8_t id{};
    if (!read(static_cast<uint8_t>(0x68), static_cast<uint8_t>(0x75), id) || id != 0x71) {
        return false;  // not an MPU9250 -> nothing to bypass
    }
    uint8_t user_ctrl{};
    if (!read(static_cast<uint8_t>(0x68), static_cast<uint8_t>(0x6A), user_ctrl)) {
        return false;
    }
    user_ctrl = static_cast<uint8_t>(user_ctrl & ~0x20);  // clear I2C_MST_EN (bit5)
    if (!write(static_cast<uint8_t>(0x68), static_cast<uint8_t>(0x6A), user_ctrl)) {
        return false;
    }
    if (!write(static_cast<uint8_t>(0x68), static_cast<uint8_t>(0x37), static_cast<uint8_t>(0x02))) {
        return false;  // INT_PIN_CFG BYPASS_EN (bit1)
    }
    return true;
}

}  // namespace imu
}  // namespace unit
}  // namespace m5
#endif
