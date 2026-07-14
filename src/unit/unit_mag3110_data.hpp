/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mag3110_data.hpp
  @brief MAG3110 setting enums and measurement data (framework independent)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_MAG3110_DATA_HPP
#define M5_UNIT_IMU_UNIT_UNIT_MAG3110_DATA_HPP

#include <array>
#include <cstdint>
#include <cmath>

namespace m5 {
namespace unit {
namespace mag3110 {

//! @brief Sampling interval (ms) for the 80Hz continuous ODR set in begin()
constexpr uint32_t CONT80HZ_INTERVAL_MS{13};

//! @brief Native sensitivity (µT per raw LSB); fixed, factory-calibrated
constexpr float RAW_UT_PER_LSB{0.10f};
//! @brief Library-wide magnetometer output scale (raw LSB per µT); see imu_convert_param_t
constexpr float OUTPUT_LSB_PER_UT{16.0f};

/*!
  @struct Data
  @brief One MAG3110 sample: a 7-byte burst from DR_STATUS (0x00)
  @details raw = {DR_STATUS, XMSB, XLSB, YMSB, YLSB, ZMSB, ZLSB}. Axes are big-endian. adj_*()
  rescale the native 0.10 µT/LSB output to the library-wide 16 LSB/µT convention (so
  imu_convert_param_t::mag_res stays a uniform 1/16, matching the other magnetometers).
 */
struct Data {
    std::array<uint8_t, 7> raw{};  //!< @brief DR_STATUS + X/Y/Z (big-endian)

    //! @brief Data ready (DR_STATUS bit3 ZYXDR: new data on one or more axes)
    inline bool drdy() const
    {
        return (raw[0] & 0x08) != 0;
    }
    //! @brief Raw X (big-endian, signed 16bit)
    inline int16_t x_raw() const
    {
        return static_cast<int16_t>((raw[1] << 8) | raw[2]);
    }
    //! @brief Raw Y
    inline int16_t y_raw() const
    {
        return static_cast<int16_t>((raw[3] << 8) | raw[4]);
    }
    //! @brief Raw Z
    inline int16_t z_raw() const
    {
        return static_cast<int16_t>((raw[5] << 8) | raw[6]);
    }

    //! @brief X rescaled to the library-wide 16 LSB/µT
    inline int32_t adj_x() const
    {
        return rescale(x_raw());
    }
    //! @brief Y rescaled to the library-wide 16 LSB/µT
    inline int32_t adj_y() const
    {
        return rescale(y_raw());
    }
    //! @brief Z rescaled to the library-wide 16 LSB/µT
    inline int32_t adj_z() const
    {
        return rescale(z_raw());
    }

private:
    static inline int32_t rescale(const int16_t raw_axis)
    {
        // Native 0.10µT/LSB -> library 16 LSB/µT: raw * 0.10 * 16
        return static_cast<int32_t>(lroundf(raw_axis * RAW_UT_PER_LSB * OUTPUT_LSB_PER_UT));
    }
};

}  // namespace mag3110
}  // namespace unit
}  // namespace m5
#endif
