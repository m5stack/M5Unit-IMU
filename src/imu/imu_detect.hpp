/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file imu_detect.hpp
  @brief Framework-agnostic detection of a classic/host built-in accel+gyro IMU
  @details Several M5Stack hosts carry a version-dependent accel/gyro chip that M5.getBoard()
  cannot distinguish: e.g. Core2 v1.1 has an MPU6886 while Core2 v1.3 has a BMI270, and ATOM Matrix
  switched from MPU6886 to BMI270 at v1.1 -- both report the same board id. detectBuiltinImu()
  probes the internal bus (via an injected read callback, so src/ stays free of M5Unified / Arduino
  Wire -- same pattern as m5::unit::bmi270::detectAddress and m5::unit::imu::detectBuiltinMag) and
  reports which accel/gyro chip is actually present, so the caller can construct the matching
  driver. The probe order mirrors M5Unified's IMU_Class::begin(): MPU family first (WHO_AM_I),
  then BMI270 (CHIP_ID at 0x68/0x69), then SH200Q. Note: m5::unit::imu here is distinct from the
  m5::imu namespace that holds the AHRS types.
*/
#ifndef M5_UNIT_IMU_IMU_IMU_DETECT_HPP
#define M5_UNIT_IMU_IMU_IMU_DETECT_HPP

#include <cstdint>
#include <initializer_list>

namespace m5 {
namespace unit {
namespace imu {

//! @brief Which built-in accel/gyro chip a host carries
enum class ImuKind : uint8_t {
    None,     //!< @brief No accel/gyro responded (e.g. an IMU-less M5Stack Basic)
    MPU6050,  //!< @brief InvenSense MPU6050 at 0x68 (WHO_AM_I 0x68)
    MPU6886,  //!< @brief InvenSense MPU6886 at 0x68 (WHO_AM_I 0x19)
    MPU9250,  //!< @brief InvenSense MPU9250 at 0x68 (WHO_AM_I 0x71), with an internal AK8963 mag
    BMI270,   //!< @brief Bosch BMI270 at 0x68 or 0x69 (CHIP_ID 0x24)
    SH200Q,   //!< @brief Senodia SH200Q at 0x6C (WHO_AM_I 0x18)
};

/*!
  @brief Probe the internal bus for a built-in accel/gyro chip
  @tparam ReadReg Callable `bool(uint8_t addr, uint8_t reg, uint8_t& out)` returning true on ACK
  @param read The bus read callable
  @return The detected accel/gyro chip kind (ImuKind::None if nothing answered)
  @details The MPU family shares WHO_AM_I register 0x75 at 0x68 (MPU6886=0x19, MPU6050=0x68,
  MPU9250=0x71) and is resolved first, so a BMI270 (whose 0x75 is undefined) is never mistaken for
  an MPU. Then the BMI270 CHIP_ID (0x24 at register 0x00) is checked at both candidate addresses
  0x68 and 0x69. Finally SH200Q is checked by its WHO_AM_I (0x18 at register 0x30, address 0x6C).
  This is detection only: it does not initialise or reset any chip.
 */
template <class ReadReg>
ImuKind detectBuiltinImu(ReadReg read)
{
    uint8_t id{};

    // MPU family: WHO_AM_I at 0x68 register 0x75.
    if (read(static_cast<uint8_t>(0x68), static_cast<uint8_t>(0x75), id)) {
        switch (id) {
            case 0x19:
                return ImuKind::MPU6886;
            case 0x68:
                return ImuKind::MPU6050;
            case 0x71:
                return ImuKind::MPU9250;
            default:
                break;  // not a known MPU id -> fall through to the BMI270 probe (BMI270 lives at 0x68 too)
        }
    }

    // BMI270: CHIP_ID 0x24 at register 0x00, on either candidate address.
    for (const uint8_t candidate : {static_cast<uint8_t>(0x68), static_cast<uint8_t>(0x69)}) {
        if (read(candidate, static_cast<uint8_t>(0x00), id) && id == 0x24) {
            return ImuKind::BMI270;
        }
    }

    // SH200Q: WHO_AM_I 0x18 at 0x6C register 0x30.
    if (read(static_cast<uint8_t>(0x6C), static_cast<uint8_t>(0x30), id) && id == 0x18) {
        return ImuKind::SH200Q;
    }

    return ImuKind::None;
}

}  // namespace imu
}  // namespace unit
}  // namespace m5
#endif
