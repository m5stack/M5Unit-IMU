/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6886_data.hpp
  @brief MPU6886 setting enums and measurement data (framework independent)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_MPU6886_DATA_HPP
#define M5_UNIT_IMU_UNIT_UNIT_MPU6886_DATA_HPP

#include <array>
#include <cstdint>

namespace m5 {
namespace unit {
namespace mpu6886 {

/*!
  @enum AccelRange
  @brief Accelerometer full scale range (register value = enum << 3)
 */
enum class AccelRange : uint8_t {
    G2,   //!< @brief +-2G (16384 LSB/G)
    G4,   //!< @brief +-4G (8192 LSB/G)
    G8,   //!< @brief +-8G (4096 LSB/G)
    G16,  //!< @brief +-16G (2048 LSB/G)
};

/*!
  @enum GyroRange
  @brief Gyroscope full scale range (register value = enum << 3)
 */
enum class GyroRange : uint8_t {
    DPS250,   //!< @brief +-250 dps
    DPS500,   //!< @brief +-500 dps
    DPS1000,  //!< @brief +-1000 dps
    DPS2000,  //!< @brief +-2000 dps
};

/*!
  @enum Dlpf
  @brief Gyro/temp digital low pass filter (CONFIG reg DLPF_CFG, valid when FCHOICE_B=00)
 */
enum class Dlpf : uint8_t {
    BW250,   //!< @brief 250Hz (gyro output 8kHz)
    BW176,   //!< @brief 176Hz (1kHz)
    BW92,    //!< @brief 92Hz
    BW41,    //!< @brief 41Hz
    BW20,    //!< @brief 20Hz
    BW10,    //!< @brief 10Hz
    BW5,     //!< @brief 5Hz
    BW3281,  //!< @brief 3281Hz (8kHz)
};

//! @brief G per LSB for each AccelRange
constexpr float accel_res(const AccelRange r)
{
    return (2.0f * (1 << static_cast<uint8_t>(r))) / 32768;
}
//! @brief dps per LSB for each GyroRange
constexpr float gyro_res(const GyroRange r)
{
    return (250.0f * (1 << static_cast<uint8_t>(r))) / 32768;
}

/*!
  @struct Data
  @brief One measurement (14 bytes burst from ACCEL_XOUT_H, big-endian)
 */
struct Data {
    std::array<uint8_t, 14> raw{};  //!< @brief [0..5] accel, [6..7] temp, [8..13] gyro

    ///@name Raw ADC values
    ///@{
    int16_t accel_x() const
    {
        return static_cast<int16_t>((raw[0] << 8) | raw[1]);
    }
    int16_t accel_y() const
    {
        return static_cast<int16_t>((raw[2] << 8) | raw[3]);
    }
    int16_t accel_z() const
    {
        return static_cast<int16_t>((raw[4] << 8) | raw[5]);
    }
    int16_t temp_adc() const
    {
        return static_cast<int16_t>((raw[6] << 8) | raw[7]);
    }
    int16_t gyro_x() const
    {
        return static_cast<int16_t>((raw[8] << 8) | raw[9]);
    }
    int16_t gyro_y() const
    {
        return static_cast<int16_t>((raw[10] << 8) | raw[11]);
    }
    int16_t gyro_z() const
    {
        return static_cast<int16_t>((raw[12] << 8) | raw[13]);
    }
    ///@}

    //! @brief Temperature in degC (T = adc/326.8 + 25.0)
    float temperature() const
    {
        return temp_adc() / 326.8f + 25.0f;
    }
};

}  // namespace mpu6886
}  // namespace unit
}  // namespace m5
#endif
