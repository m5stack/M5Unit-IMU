/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_bmi270_data.hpp
  @brief BMI270 setting enums and measurement data (framework independent)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_BMI270_DATA_HPP
#define M5_UNIT_IMU_UNIT_UNIT_BMI270_DATA_HPP

#include <array>
#include <cstdint>
#include "unit_bmm150_data.hpp"

namespace m5 {
namespace unit {
namespace bmi270 {

/*!
  @enum AccelRange
  @brief Accelerometer full scale range (ACC_RANGE register 0x41)
 */
enum class AccelRange : uint8_t {
    G2,   //!< @brief +-2G (16384 LSB/G)
    G4,   //!< @brief +-4G (8192 LSB/G)
    G8,   //!< @brief +-8G (4096 LSB/G, power-on default)
    G16,  //!< @brief +-16G (2048 LSB/G)
};

/*!
  @enum GyroRange
  @brief Gyroscope full scale range (GYR_RANGE register 0x43)
  @note Unlike UnitMPU6886's GyroRange, value 0 is the *largest* range here
  (register encoding is chip-native, not renumbered to match MPU6886)
 */
enum class GyroRange : uint8_t {
    DPS2000,  //!< @brief +-2000 dps (power-on default)
    DPS1000,  //!< @brief +-1000 dps
    DPS500,   //!< @brief +-500 dps
    DPS250,   //!< @brief +-250 dps
    DPS125,   //!< @brief +-125 dps
};

/*!
  @enum AccelODR
  @brief Accelerometer output data rate (ACC_CONF register 0x40, bits[3:0])
 */
enum class AccelODR : uint8_t {
    Rate12_5Hz = 0x05,  //!< @brief 12.5Hz
    Rate25Hz,           //!< @brief 25Hz
    Rate50Hz,           //!< @brief 50Hz
    Rate100Hz,          //!< @brief 100Hz (power-on default)
    Rate200Hz,          //!< @brief 200Hz
    Rate400Hz,          //!< @brief 400Hz
    Rate800Hz,          //!< @brief 800Hz
    Rate1600Hz,         //!< @brief 1600Hz
};

/*!
  @enum GyroODR
  @brief Gyroscope output data rate (GYR_CONF register 0x42, bits[3:0])
 */
enum class GyroODR : uint8_t {
    Rate25Hz = 0x06,  //!< @brief 25Hz
    Rate50Hz,         //!< @brief 50Hz
    Rate100Hz,        //!< @brief 100Hz
    Rate200Hz,        //!< @brief 200Hz (power-on default)
    Rate400Hz,        //!< @brief 400Hz
    Rate800Hz,        //!< @brief 800Hz
    Rate1600Hz,       //!< @brief 1600Hz
    Rate3200Hz,       //!< @brief 3200Hz
};

//! @brief G per LSB for each AccelRange
constexpr float accel_res(const AccelRange r)
{
    return (2.0f * (1 << static_cast<uint8_t>(r))) / 32768;
}
//! @brief dps per LSB for each GyroRange (note: value 0 is the largest range, see GyroRange)
constexpr float gyro_res(const GyroRange r)
{
    return (2000.0f / (1 << static_cast<uint8_t>(r))) / 32768;
}

// Sampling interval (ms) per GyroODR raw register value, ceil(1000/Hz), indexed 0x00..0x0D
// (only 0x06..0x0D are valid GyroODR values; others are unused placeholders).
// constexpr namespace-scope array: internal linkage per translation unit (no ODR issue).
constexpr uint32_t GYRO_ODR_INTERVAL_MS[14] = {
    0,  0, 0, 0, 0, 0,  // 0x00..0x05 (unused)
    40,                 // 0x06 = 25Hz
    20,                 // 0x07 = 50Hz
    10,                 // 0x08 = 100Hz
    5,                  // 0x09 = 200Hz
    3,                  // 0x0A = 400Hz
    2,                  // 0x0B = 800Hz
    1,                  // 0x0C = 1600Hz
    1,                  // 0x0D = 3200Hz
};

//! @brief Sampling interval in milliseconds for the given GyroODR
constexpr uint32_t odr_interval_ms(const GyroODR odr)
{
    return GYRO_ODR_INTERVAL_MS[static_cast<uint8_t>(odr)];
}

/*!
  @struct Data
  @brief One measurement. Accel/gyro are always 12 bytes, little-endian
  (register 0x0C when the AUX/BMM150 bridge is not in use, or bytes [8..19] of
  the 20 byte burst from register 0x04 when it is; see UnitBMI270::read_measurement())
 */
struct Data {
    std::array<uint8_t, 12> raw{};  //!< @brief [0..5] accel XYZ, [6..11] gyro XYZ
    //! @brief AUX (on-chip BMM150) magnetometer sample, valid only when aux_mag is true.
    //! Reuses bmm150::Data's bit-packing and trim compensation (comp_x()/comp_y()/comp_z())
    //! so the AUX path and the standalone UnitBMM150 path share identical math
    bmm150::Data mag{};
    bool aux_mag{false};  //!< @brief True if mag holds a sample from the AUX (BMM150) bridge

    ///@name Raw ADC values (little-endian)
    ///@{
    //! @brief Raw accelerometer X
    int16_t accel_x() const
    {
        return static_cast<int16_t>((raw[1] << 8) | raw[0]);
    }
    //! @brief Raw accelerometer Y
    int16_t accel_y() const
    {
        return static_cast<int16_t>((raw[3] << 8) | raw[2]);
    }
    //! @brief Raw accelerometer Z
    int16_t accel_z() const
    {
        return static_cast<int16_t>((raw[5] << 8) | raw[4]);
    }
    //! @brief Raw gyroscope X
    int16_t gyro_x() const
    {
        return static_cast<int16_t>((raw[7] << 8) | raw[6]);
    }
    //! @brief Raw gyroscope Y
    int16_t gyro_y() const
    {
        return static_cast<int16_t>((raw[9] << 8) | raw[8]);
    }
    //! @brief Raw gyroscope Z
    int16_t gyro_z() const
    {
        return static_cast<int16_t>((raw[11] << 8) | raw[10]);
    }
    ///@}
};

}  // namespace bmi270
}  // namespace unit
}  // namespace m5
#endif
