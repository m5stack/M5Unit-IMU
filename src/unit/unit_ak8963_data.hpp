/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_ak8963_data.hpp
  @brief AK8963 setting enums and measurement data (framework independent)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_AK8963_DATA_HPP
#define M5_UNIT_IMU_UNIT_UNIT_AK8963_DATA_HPP

#include <array>
#include <cstdint>
#include <cmath>

namespace m5 {
namespace unit {
namespace ak8963 {

/*!
  @enum Mode
  @brief CNTL1 (0x0A) low-nibble operating mode
 */
enum class Mode : uint8_t {
    PowerDown = 0x00,  //!< @brief Power-down
    Single    = 0x01,  //!< @brief Single measurement
    Cont8Hz   = 0x02,  //!< @brief Continuous measurement, 8Hz
    Cont100Hz = 0x06,  //!< @brief Continuous measurement, 100Hz
    SelfTest  = 0x08,  //!< @brief Self test
    FuseRom   = 0x0F,  //!< @brief FUSE-ROM access (ASA readable only here)
};

//! @brief Sampling interval (ms) for Cont100Hz
constexpr uint32_t CONT100HZ_INTERVAL_MS{10};

//! @brief 16bit-mode sensitivity (µT per raw LSB) = 4912/32760
constexpr float RAW_UT_PER_LSB{4912.0f / 32760.0f};
//! @brief Library-wide magnetometer output scale (raw LSB per µT); see imu_convert_param_t
constexpr float OUTPUT_LSB_PER_UT{16.0f};

/*!
  @struct Data
  @brief One AK8963 sample: an 8-byte burst from ST1 (0x02) plus the device ASA trim
  @details raw = {ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2}. Axes are little-endian.
  adj_*() apply the per-axis sensitivity adjustment and rescale to the library-wide
  16 LSB/µT convention (so imu_convert_param_t::mag_res stays a uniform 1/16).
 */
struct Data {
    std::array<uint8_t, 8> raw{};   //!< @brief ST1 + X/Y/Z (LE) + ST2
    std::array<uint8_t, 3> asa{};   //!< @brief Per-axis sensitivity adjustment (device constant)

    //! @brief Data ready (ST1 bit0)
    inline bool drdy() const
    {
        return (raw[0] & 0x01) != 0;
    }
    //! @brief Magnetic overflow (ST2 bit3 HOFL); sample invalid if set
    inline bool overflow() const
    {
        return (raw[7] & 0x08) != 0;
    }
    //! @brief Raw X (little-endian, signed 16bit)
    inline int16_t x_raw() const
    {
        return static_cast<int16_t>((raw[2] << 8) | raw[1]);
    }
    //! @brief Raw Y
    inline int16_t y_raw() const
    {
        return static_cast<int16_t>((raw[4] << 8) | raw[3]);
    }
    //! @brief Raw Z
    inline int16_t z_raw() const
    {
        return static_cast<int16_t>((raw[6] << 8) | raw[5]);
    }

    //! @brief ASA-adjusted X in library-wide 16 LSB/µT
    inline int32_t adj_x() const
    {
        return adjust(x_raw(), asa[0]);
    }
    //! @brief ASA-adjusted Y
    inline int32_t adj_y() const
    {
        return adjust(y_raw(), asa[1]);
    }
    //! @brief ASA-adjusted Z
    inline int32_t adj_z() const
    {
        return adjust(z_raw(), asa[2]);
    }

private:
    static inline int32_t adjust(const int16_t raw_axis, const uint8_t asa_axis)
    {
        // Hadj = raw * (ASA + 128) / 256, then rescale 0.15µT/LSB -> 16 LSB/µT
        const float ut = raw_axis * ((asa_axis + 128) / 256.0f) * RAW_UT_PER_LSB;
        return static_cast<int32_t>(lroundf(ut * OUTPUT_LSB_PER_UT));
    }
};

}  // namespace ak8963
}  // namespace unit
}  // namespace m5
#endif
