/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file imu_types.hpp
  @brief Common types for M5Unit-IMU
*/
#ifndef M5_UNIT_IMU_IMU_IMU_TYPES_HPP
#define M5_UNIT_IMU_IMU_IMU_TYPES_HPP

#include <cstdint>

namespace m5 {
namespace imu {

/*!
  @enum Capability
  @brief Capability of the IMU data source (bitwise combinable)
 */
enum class Capability : uint8_t {
    None  = 0,       //!< @brief Provides nothing
    Accel = 1 << 0,  //!< @brief 3-axis accelerometer
    Gyro  = 1 << 1,  //!< @brief 3-axis gyroscope
    Mag   = 1 << 2,  //!< @brief 3-axis magnetometer
    Baro  = 1 << 3,  //!< @brief Barometric pressure (reserved for future)
    Temp  = 1 << 4,  //!< @brief Temperature output
};

//! @brief Bitwise OR
constexpr Capability operator|(const Capability a, const Capability b)
{
    return static_cast<Capability>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}
//! @brief Bitwise AND
constexpr Capability operator&(const Capability a, const Capability b)
{
    return static_cast<Capability>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}
//! @brief Bitwise OR assignment
inline Capability& operator|=(Capability& a, const Capability b)
{
    a = a | b;
    return a;
}
//! @brief Any bit set?
constexpr bool any(const Capability c)
{
    return c != Capability::None;
}

/*!
  @struct imu_raw_data_t
  @brief Unified raw data (chip-native axes, no remap applied)
 */
struct imu_raw_data_t {
    int16_t accel[3]{};  //!< @brief Raw accelerometer X/Y/Z
    int16_t gyro[3]{};   //!< @brief Raw gyroscope X/Y/Z
    int32_t mag[3]{};    //!< @brief Raw magnetometer X/Y/Z (32bit for BMM350)
    int32_t baro{};      //!< @brief Raw pressure (reserved)
    int16_t temp{};      //!< @brief Raw temperature
};

/*!
  @struct imu_convert_param_t
  @brief Raw to physical unit conversion coefficients
 */
struct imu_convert_param_t {
    float accel_res{8.0f / 32768};    //!< @brief G per LSB
    float gyro_res{2000.0f / 32768};  //!< @brief dps per LSB
    float mag_res{4912.0f / 32760};   //!< @brief uT per LSB
    float temp_res{1.0f};             //!< @brief degC per LSB
    float temp_offset{0.0f};          //!< @brief degC offset
};

/*!
  @struct imu_physical_data_t
  @brief Merged, converted physical-unit IMU sample (AhrsController::latest())
 */
struct imu_physical_data_t {
    float accel[3]{};  //!< @brief Accelerometer X/Y/Z (G)
    float gyro[3]{};   //!< @brief Gyroscope X/Y/Z (dps)
    float mag[3]{};    //!< @brief Magnetometer X/Y/Z (uT)
    float temp{};      //!< @brief Temperature (degC)
};

}  // namespace imu
}  // namespace m5
#endif
