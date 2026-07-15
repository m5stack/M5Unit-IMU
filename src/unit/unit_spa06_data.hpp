/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_spa06_data.hpp
  @brief SPA06-003 measurement data + calibration/compensation (framework independent)
  @details Per datasheet/spa06-003.md. PSR/TMP are 24-bit 2's complement; 11 calibration coefficients
  (c0,c1 12-bit; c00,c10 20-bit; c01,c11,c20,c21,c30 16-bit; c31,c40 12-bit) drive a high-order
  compensation polynomial. Pressure is returned in Pa; the UnitSPA06 driver converts to hPa.
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_SPA06_DATA_HPP
#define M5_UNIT_IMU_UNIT_UNIT_SPA06_DATA_HPP

#include <array>
#include <cstdint>
#include <cmath>

namespace m5 {
namespace unit {
namespace spa06 {

//! @brief Compensation scale factor kP/kT for an oversampling rate (datasheet Table 4)
inline float scale_factor(const uint8_t oversampling)
{
    switch (oversampling) {
        case 1:
            return 524288.0f;
        case 2:
            return 1572864.0f;
        case 4:
            return 3670016.0f;
        case 8:
            return 7864320.0f;
        case 16:
            return 253952.0f;
        case 32:
            return 516096.0f;
        case 64:
            return 1040384.0f;
        case 128:
            return 2088960.0f;
        default:
            return 253952.0f;  // 16x default
    }
}

//! @brief Sign-extend the low `bits` of `v` (2's complement)
inline int32_t sign_extend(const uint32_t v, const uint8_t bits)
{
    const uint32_t m = static_cast<uint32_t>(1) << (bits - 1);
    return static_cast<int32_t>((v ^ m) - m);
}

//! @brief Factory calibration coefficients (all 2's complement, already sign-extended)
struct coeffs_t {
    int32_t c0{}, c1{};
    int32_t c00{}, c10{};
    int32_t c01{}, c11{}, c20{}, c21{}, c30{};
    int32_t c31{}, c40{};
};

//! @brief Parse the 21 COEF register bytes (0x10..0x24) into coeffs_t (datasheet Table 10)
inline coeffs_t parse_coeffs(const std::array<uint8_t, 21>& r)
{
    coeffs_t c{};
    c.c0  = sign_extend((static_cast<uint32_t>(r[0]) << 4) | (r[1] >> 4), 12);
    c.c1  = sign_extend(((static_cast<uint32_t>(r[1]) & 0x0F) << 8) | r[2], 12);
    c.c00 = sign_extend((static_cast<uint32_t>(r[3]) << 12) | (static_cast<uint32_t>(r[4]) << 4) | (r[5] >> 4), 20);
    c.c10 = sign_extend(((static_cast<uint32_t>(r[5]) & 0x0F) << 16) | (static_cast<uint32_t>(r[6]) << 8) | r[7], 20);
    c.c01 = sign_extend((static_cast<uint32_t>(r[8]) << 8) | r[9], 16);
    c.c11 = sign_extend((static_cast<uint32_t>(r[10]) << 8) | r[11], 16);
    c.c20 = sign_extend((static_cast<uint32_t>(r[12]) << 8) | r[13], 16);
    c.c21 = sign_extend((static_cast<uint32_t>(r[14]) << 8) | r[15], 16);
    c.c30 = sign_extend((static_cast<uint32_t>(r[16]) << 8) | r[17], 16);
    c.c31 = sign_extend((static_cast<uint32_t>(r[18]) << 4) | (r[19] >> 4), 12);
    c.c40 = sign_extend(((static_cast<uint32_t>(r[19]) & 0x0F) << 8) | r[20], 12);
    return c;
}

//! @brief Compensated temperature (degC) from a raw TMP reading and scale kT
inline float compensate_temperature(const int32_t tmp_raw, const coeffs_t& c, const float kt)
{
    const float traw_sc = tmp_raw / kt;
    return c.c0 * 0.5f + c.c1 * traw_sc;
}

//! @brief Compensated pressure (Pa) from raw PSR/TMP readings and scales kP/kT (datasheet 4.6.1)
inline float compensate_pressure(const int32_t psr_raw, const int32_t tmp_raw, const coeffs_t& c, const float kp,
                                  const float kt)
{
    const float praw_sc = psr_raw / kp;
    const float traw_sc = tmp_raw / kt;
    return c.c00 + praw_sc * (c.c10 + praw_sc * (c.c20 + praw_sc * (c.c30 + praw_sc * c.c40))) + traw_sc * c.c01 +
           traw_sc * praw_sc * (c.c11 + praw_sc * (c.c21 + praw_sc * c.c31));
}

//! @brief Relative altitude (m) from a pressure and a reference (both hPa), barometric formula
inline float altitude_m(const float pressure_hpa, const float sea_level_hpa)
{
    return 44330.0f * (1.0f - std::pow(pressure_hpa / sea_level_hpa, 1.0f / 5.255f));
}

/*!
  @struct Data
  @brief One SPA06 sample: a 6-byte burst from PSR_B2 (0x00) = PSR(24) + TMP(24), big-endian
 */
struct Data {
    std::array<uint8_t, 6> raw{};  //!< @brief PSR B2/B1/B0 + TMP B2/B1/B0

    //! @brief Raw pressure (24-bit 2's complement)
    inline int32_t psr_raw() const
    {
        return sign_extend((static_cast<uint32_t>(raw[0]) << 16) | (static_cast<uint32_t>(raw[1]) << 8) | raw[2], 24);
    }
    //! @brief Raw temperature (24-bit 2's complement)
    inline int32_t tmp_raw() const
    {
        return sign_extend((static_cast<uint32_t>(raw[3]) << 16) | (static_cast<uint32_t>(raw[4]) << 8) | raw[5], 24);
    }
};

}  // namespace spa06
}  // namespace unit
}  // namespace m5
#endif
