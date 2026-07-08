/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_bmm350_data.hpp
  @brief BMM350 setting enums and measurement data (framework independent)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_BMM350_DATA_HPP
#define M5_UNIT_IMU_UNIT_UNIT_BMM350_DATA_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace m5 {
namespace unit {
namespace bmm350 {

/*!
  @enum DataRate
  @brief Output data rate (PMU_CMD_AGGR_SET register 0x04, bits[3:0])
 */
enum class DataRate : uint8_t {
    Rate400Hz    = 0x02,  //!< @brief 400Hz
    Rate200Hz    = 0x03,  //!< @brief 200Hz
    Rate100Hz    = 0x04,  //!< @brief 100Hz (power-on default)
    Rate50Hz     = 0x05,  //!< @brief 50Hz
    Rate25Hz     = 0x06,  //!< @brief 25Hz
    Rate12_5Hz   = 0x07,  //!< @brief 12.5Hz
    Rate6_25Hz   = 0x08,  //!< @brief 6.25Hz
    Rate3_125Hz  = 0x09,  //!< @brief 3.125Hz
    Rate1_5625Hz = 0x0A,  //!< @brief 1.5625Hz
};

/*!
  @enum Averaging
  @brief Averaging factor (PMU_CMD_AGGR_SET register 0x04, bits[5:4])
 */
enum class Averaging : uint8_t {
    None,  //!< @brief No averaging, 1 sample (LowPower)
    X2,    //!< @brief 2 samples averaged (Regular, power-on default)
    X4,    //!< @brief 4 samples averaged (LowNoise)
    X8,    //!< @brief 8 samples averaged (UltraLowNoise)
};

//! @brief Clamps avg to the combination the sensor accepts for the given rate
//! (400Hz allows None only, 200Hz allows up to X2, 100Hz allows up to X4,
//! X8 is only accepted at 50Hz or below)
constexpr Averaging clamp_averaging(const DataRate rate, const Averaging avg)
{
    return (rate == DataRate::Rate400Hz) ? Averaging::None
           : (rate == DataRate::Rate200Hz)
               ? (static_cast<uint8_t>(avg) > static_cast<uint8_t>(Averaging::X2) ? Averaging::X2 : avg)
           : (rate == DataRate::Rate100Hz)
               ? (static_cast<uint8_t>(avg) > static_cast<uint8_t>(Averaging::X4) ? Averaging::X4 : avg)
               : avg;
}

//! @brief Sign-extends a 24bit unsigned register value to a signed 32bit value
constexpr int32_t fix_sign24(const uint32_t raw)
{
    return (raw >= 0x800000u) ? static_cast<int32_t>(raw - 0x1000000u) : static_cast<int32_t>(raw);
}

// The following conversion coefficients are the values published in the
// BMM350 datasheet (BST-BMM350-DS001), and match the ones the Bosch
// BMM350_SensorAPI (BSD-3-Clause) derives at runtime from the sensor's
// analog front-end gains in update_default_coefiecents(): e.g. for X/Y,
// (1e6/2^20) / (14.55 * 19.46 * (1/1.5) * 0.714607238769531) = 0.0070700 uT/LSB.

//! @brief uT per LSB of the raw 24bit magnetometer value, X/Y axes
constexpr float lsb_to_ut_xy()
{
    return 0.00707f;
}
//! @brief uT per LSB of the raw 24bit magnetometer value, Z axis
constexpr float lsb_to_ut_z()
{
    return 0.0071750f;
}
//! @brief degC per LSB of the raw 24bit temperature ADC value
constexpr float temp_res_raw24()
{
    return 0.00098128f;
}
//! @brief Fixed offset (degC) applied to the primary temperature conversion
constexpr float TEMP_OFFSET = -25.49f;

/*!
  @struct otp_compensation_t
  @brief Per-chip OTP compensation coefficients (read out at boot from the
  sensor's one-time-programmable memory), consumed by compensate_temp() and
  compensate_mag(). All-zero (default-constructed, aside from t0) is a
  neutral / pass-through configuration.

  Formulas re-implemented (not copied) from the Bosch BMM350_SensorAPI
  (bmm350_get_compensated_mag_xyz_temp_data(), BSD-3-Clause), verified
  against https://github.com/boschsensortec/BMM350_SensorAPI/blob/main/bmm350.c
 */
struct otp_compensation_t {
    float t_sens{};     //!< @brief Temperature sensitivity coefficient
    float t_offs{};     //!< @brief Temperature offset coefficient
    float t0{23.0f};    //!< @brief Reference temperature (degC)
    float offset[3]{};  //!< @brief Per-axis (x,y,z) offset
    float sens[3]{};    //!< @brief Per-axis (x,y,z) sensitivity coefficient
    float tco[3]{};     //!< @brief Per-axis (x,y,z) temperature coefficient of offset
    float tcs[3]{};     //!< @brief Per-axis (x,y,z) temperature coefficient of sensitivity
    float cross[4]{};   //!< @brief Cross-axis coefficients: [0]=x_y [1]=y_x [2]=z_x [3]=z_y

    //! @brief Applies the OTP temperature compensation to a primary-converted temperature (degC)
    float compensate_temp(const float t_raw) const
    {
        return (1.0f + t_sens) * t_raw + t_offs;
    }

    //! @brief Applies per-axis offset/sensitivity/tco/tcs and cross-axis compensation (in-place) to
    //! primary-converted uT values, given the already temperature-compensated value from compensate_temp()
    void compensate_mag(float (&m)[3], const float t_comp) const
    {
        for (size_t i = 0; i < 3; ++i) {
            m[i] *= 1.0f + sens[i];
            m[i] += offset[i];
            m[i] += tco[i] * (t_comp - t0);
            m[i] /= 1.0f + tcs[i] * (t_comp - t0);
        }

        const float cross_xy = cross[0];
        const float cross_yx = cross[1];
        const float cross_zx = cross[2];
        const float cross_zy = cross[3];
        const float denom    = 1.0f - cross_yx * cross_xy;

        const float x       = (m[0] - cross_xy * m[1]) / denom;
        const float y       = (m[1] - cross_yx * m[0]) / denom;
        const float z_delta = m[0] * (cross_yx * cross_zy - cross_zx) - m[1] * (cross_zy - cross_xy * cross_zx);
        const float z       = m[2] + z_delta / denom;

        m[0] = x;
        m[1] = y;
        m[2] = z;
    }
};

/*!
  @struct Data
  @brief One measurement (12 bytes burst from MAG_X_XLSB, register 0x31): 9
  bytes magnetometer (X,Y,Z, 24bit little-endian each) followed by 3 bytes
  temperature (24bit little-endian)
 */
struct Data {
    std::array<uint8_t, 12> raw{};  //!< @brief [0..2] X, [3..5] Y, [6..8] Z, [9..11] temperature
    otp_compensation_t comp{};      //!< @brief OTP compensation coefficients used by x_ut()/y_ut()/z_ut()

    ///@name Raw ADC values (24bit signed, sign-extended)
    ///@{
    //! @brief Raw magnetometer X
    int32_t x_raw() const
    {
        return fix_sign24(assemble24(0));
    }
    //! @brief Raw magnetometer Y
    int32_t y_raw() const
    {
        return fix_sign24(assemble24(3));
    }
    //! @brief Raw magnetometer Z
    int32_t z_raw() const
    {
        return fix_sign24(assemble24(6));
    }
    //! @brief Raw temperature ADC (24bit, sign-extended)
    int32_t temp_raw24() const
    {
        return fix_sign24(assemble24(9));
    }
    //! @brief Raw temperature ADC normalized to 16bit (temp_raw24() >> 8)
    int16_t temp_adc() const
    {
        return static_cast<int16_t>(temp_raw24() >> 8);
    }
    ///@}

    ///@name Compensated values (primary conversion + OTP compensation applied, uT)
    ///@{
    //! @brief Compensated magnetometer X (uT)
    float x_ut() const
    {
        return compensated_mag()[0];
    }
    //! @brief Compensated magnetometer Y (uT)
    float y_ut() const
    {
        return compensated_mag()[1];
    }
    //! @brief Compensated magnetometer Z (uT)
    float z_ut() const
    {
        return compensated_mag()[2];
    }
    ///@}

private:
    // Assembles a little-endian 24bit unsigned value from raw[idx..idx+2]
    uint32_t assemble24(const size_t idx) const
    {
        return static_cast<uint32_t>(raw[idx]) | (static_cast<uint32_t>(raw[idx + 1]) << 8) |
               (static_cast<uint32_t>(raw[idx + 2]) << 16);
    }
    // Primary (pre-OTP) temperature conversion (degC)
    float primary_temp() const
    {
        return static_cast<float>(temp_raw24()) * temp_res_raw24() + TEMP_OFFSET;
    }
    // Primary conversion (uT) followed by full OTP compensation for all 3 axes
    std::array<float, 3> compensated_mag() const
    {
        std::array<float, 3> m{static_cast<float>(x_raw()) * lsb_to_ut_xy(),
                               static_cast<float>(y_raw()) * lsb_to_ut_xy(),
                               static_cast<float>(z_raw()) * lsb_to_ut_z()};
        float raw_arr[3] = {m[0], m[1], m[2]};
        comp.compensate_mag(raw_arr, comp.compensate_temp(primary_temp()));
        m[0] = raw_arr[0];
        m[1] = raw_arr[1];
        m[2] = raw_arr[2];
        return m;
    }
};

}  // namespace bmm350
}  // namespace unit
}  // namespace m5
#endif
