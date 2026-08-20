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

/*!
  @enum DataRate
  @brief Output data rate select (REG_CTRL_REG1 DR[7:5])
  @details Each value is named after the output rate it gives at OverSampling::X16, which is how the
  datasheet labels them (Table 32); a larger oversampling ratio divides that down. DR on its own
  fixes the ADC rate and with it the supply current, from 900µA at Rate80Hz to 8.6µA at Rate0_63Hz
 */
enum class DataRate : uint8_t {
    Rate80Hz,    //!< @brief 80Hz at X16 (ADC 1280Hz, 900µA)
    Rate40Hz,    //!< @brief 40Hz at X16 (ADC 640Hz, 550µA)
    Rate20Hz,    //!< @brief 20Hz at X16 (ADC 320Hz, 275µA)
    Rate10Hz,    //!< @brief 10Hz at X16 (ADC 160Hz, 137.5µA)
    Rate5Hz,     //!< @brief 5Hz at X16 (ADC 80Hz, 68.8µA)
    Rate2_5Hz,   //!< @brief 2.5Hz at X16 (ADC 80Hz, 34.4µA)
    Rate1_25Hz,  //!< @brief 1.25Hz at X16 (ADC 80Hz, 17.2µA)
    Rate0_63Hz,  //!< @brief 0.63Hz at X16 (ADC 80Hz, 8.6µA)
};

/*!
  @enum OverSampling
  @brief Over sampling ratio (REG_CTRL_REG1 OS[4:3])
  @details OS on its own fixes the noise floor: 0.40µT rms at X16 down to 0.25µT rms at X128. It
  also divides the output rate, so most output rates are reachable from several DataRate/OverSampling
  pairs whose current and noise differ widely -- 10Hz for instance is available at 900/550/275/137.5µA
  with 0.25/0.30/0.35/0.40µT rms. That trade-off is why the two fields are exposed separately
 */
enum class OverSampling : uint8_t {
    X16,   //!< @brief 16x (0.40µT rms)
    X32,   //!< @brief 32x (0.35µT rms)
    X64,   //!< @brief 64x (0.30µT rms)
    X128,  //!< @brief 128x (0.25µT rms)
};

///@cond
// Output rate in milli-Hz at OverSampling::X16, indexed by DataRate. Milli-Hz rather than Hz so the
// oversampling division below stays exact enough at the slow end (0.63Hz / 128 is 4.9mHz)
constexpr uint32_t OUTPUT_RATE_MHZ[8] = {80000, 40000, 20000, 10000, 5000, 2500, 1250, 630};
///@endcond

//! @brief Effective output rate in milli-Hz
//! @details Oversampling divides the DataRate's rate, exactly as datasheet Table 32 tabulates it
constexpr uint32_t output_rate_mhz(const DataRate rate, const OverSampling os)
{
    return OUTPUT_RATE_MHZ[static_cast<uint8_t>(rate)] >> static_cast<uint8_t>(os);
}

//! @brief Sampling interval (ms) for a rate/oversampling pair, rounded up
//! @note Rate80Hz + X16 gives 13ms, the interval this driver used when the rate was fixed
constexpr uint32_t interval_ms(const DataRate rate, const OverSampling os)
{
    return (1000000U + output_rate_mhz(rate, os) - 1) / output_rate_mhz(rate, os);
}

//! @brief REG_CTRL_REG1 value entering continuous ACTIVE measurement (FR and TM left at 0)
constexpr uint8_t ctrl_reg1_active(const DataRate rate, const OverSampling os)
{
    return static_cast<uint8_t>((static_cast<uint8_t>(rate) << 5) | (static_cast<uint8_t>(os) << 3) | 0x01);
}

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
