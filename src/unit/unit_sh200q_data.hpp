/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_sh200q_data.hpp
  @brief SH200Q setting enums and measurement data (framework independent)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_SH200Q_DATA_HPP
#define M5_UNIT_IMU_UNIT_UNIT_SH200Q_DATA_HPP

#include <array>
#include <cstdint>

namespace m5 {
namespace unit {
namespace sh200q {

/*!
  @enum AccelRange
  @brief Accelerometer full scale range (REG_ACC_RANGE bits[1:0], register value = enum)
  @note The SH200Q has no +-2G range
 */
enum class AccelRange : uint8_t {
    G4,   //!< @brief +-4G (8192 LSB/G)
    G8,   //!< @brief +-8G (4096 LSB/G)
    G16,  //!< @brief +-16G (2048 LSB/G)
};

/*!
  @enum GyroRange
  @brief Gyroscope full scale range (REG_GYRO_RANGE bits[2:0], register value = enum)
  @warning Ordered widest first (the register encoding), the REVERSE of the original M5StickC
  library's Gscale enum
 */
enum class GyroRange : uint8_t {
    DPS2000,  //!< @brief +-2000 dps (16.4 LSB/dps)
    DPS1000,  //!< @brief +-1000 dps (32.8 LSB/dps)
    DPS500,   //!< @brief +-500 dps (65.5 LSB/dps)
    DPS250,   //!< @brief +-250 dps (131 LSB/dps)
    DPS125,   //!< @brief +-125 dps (262 LSB/dps)
};

/*!
  @enum AccelOdr
  @brief Accelerometer output data rate (REG_ACC_CONFIG bits[4:3], normal mode)
 */
enum class AccelOdr : uint8_t {
    HZ1024,  //!< @brief 1024 Hz
    HZ512,   //!< @brief 512 Hz
    HZ256,   //!< @brief 256 Hz
    HZ128,   //!< @brief 128 Hz
};

/*!
  @enum GyroOdr
  @brief Gyroscope output data rate (REG_GYRO_CONFIG bits[3:1])
  @note The chip also offers 31.25 Hz and 8/16/32 kHz modes; they are not modeled here (the DLPF
  table differs for the high-speed modes)
 */
enum class GyroOdr : uint8_t {
    HZ1000,  //!< @brief 1000 Hz
    HZ500,   //!< @brief 500 Hz
    HZ250,   //!< @brief 250 Hz
};

/*!
  @enum GyroDlpf
  @brief Gyro digital low pass filter (REG_GYRO_DLPF). Bandwidths quoted at the default 500 Hz
  ODR; they scale with the selected ODR
 */
enum class GyroDlpf : uint8_t {
    BW250,  //!< @brief 250Hz @500Hz ODR
    BW200,  //!< @brief 200Hz
    BW100,  //!< @brief 100Hz
    BW50,   //!< @brief 50Hz
    BW25,   //!< @brief 25Hz
};

//! @brief G per LSB for each AccelRange
constexpr float accel_res(const AccelRange r)
{
    return (4.0f * (1 << static_cast<uint8_t>(r))) / 32768;
}
//! @brief dps per LSB for each GyroRange
constexpr float gyro_res(const GyroRange r)
{
    return (2000.0f / (1 << static_cast<uint8_t>(r))) / 32768;
}
//! @brief Gyroscope ODR in Hz (used to derive the periodic measurement interval)
constexpr uint32_t gyro_odr_hz(const GyroOdr r)
{
    return 1000U >> static_cast<uint8_t>(r);
}

/*!
  @struct Data
  @brief One measurement (14 bytes burst from REG_OUTPUT, little-endian -- NOT the MPU byte order)
 */
struct Data {
    std::array<uint8_t, 14> raw{};  //!< @brief [0..5] accel, [6..11] gyro, [12..13] temp

    ///@name Raw ADC values
    ///@{
    //! @brief Raw accelerometer X
    inline int16_t accel_x() const
    {
        return static_cast<int16_t>((raw[1] << 8) | raw[0]);
    }
    //! @brief Raw accelerometer Y
    inline int16_t accel_y() const
    {
        return static_cast<int16_t>((raw[3] << 8) | raw[2]);
    }
    //! @brief Raw accelerometer Z
    inline int16_t accel_z() const
    {
        return static_cast<int16_t>((raw[5] << 8) | raw[4]);
    }
    //! @brief Raw gyroscope X
    inline int16_t gyro_x() const
    {
        return static_cast<int16_t>((raw[7] << 8) | raw[6]);
    }
    //! @brief Raw gyroscope Y
    inline int16_t gyro_y() const
    {
        return static_cast<int16_t>((raw[9] << 8) | raw[8]);
    }
    //! @brief Raw gyroscope Z
    inline int16_t gyro_z() const
    {
        return static_cast<int16_t>((raw[11] << 8) | raw[10]);
    }
    //! @brief Raw temperature ADC
    inline int16_t temp_adc() const
    {
        return static_cast<int16_t>((raw[13] << 8) | raw[12]);
    }
    ///@}

    //! @brief Temperature in degC (T = adc/333.87 + 21.0)
    //! @warning The datasheet documents no conversion formula; these coefficients come from the
    //! original M5StickC library (identical to the MPU6500/6886 formula) -- treat the absolute
    //! value as approximate
    inline float temperature() const
    {
        return temp_adc() / 333.87f + 21.0f;
    }
};

}  // namespace sh200q
}  // namespace unit
}  // namespace m5
#endif
