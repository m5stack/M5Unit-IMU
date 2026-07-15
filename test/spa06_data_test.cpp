/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <unit/unit_spa06_data.hpp>
#include <cmath>

using namespace m5::unit::spa06;

TEST(SPA06Data, Raw24BitSignExtend)
{
    Data d{};
    d.raw = {0x80, 0x00, 0x00, 0x7F, 0xFF, 0xFF};  // PSR = -2^23, TMP = +2^23-1
    EXPECT_EQ(d.psr_raw(), -8388608);
    EXPECT_EQ(d.tmp_raw(), 8388607);
}

TEST(SPA06Data, KpKtByOversampling)
{
    EXPECT_FLOAT_EQ(scale_factor(1), 524288.0f);
    EXPECT_FLOAT_EQ(scale_factor(16), 253952.0f);
    EXPECT_FLOAT_EQ(scale_factor(128), 2088960.0f);
}

TEST(SPA06Data, ParseCoeffsSignExtend)
{
    // c0 is 12-bit 2's complement in bytes 0x10=c0[11:4], 0x11 hi nibble=c0[3:0].
    std::array<uint8_t, 21> reg{};  // 0x10..0x24
    reg[0] = 0xFF;                  // c0[11:4] = 0xFF
    reg[1] = 0xF0;                  // c0[3:0] = 0xF  -> c0 = 0xFFF = -1 (12-bit)
    coeffs_t c = parse_coeffs(reg);
    EXPECT_EQ(c.c0, -1);
}

TEST(SPA06Data, CompensationMatchesReference)
{
    // Reference vector: all-zero raw, identity-ish coeffs -> pressure = c00, temp = c0*0.5.
    coeffs_t c{};
    c.c00 = 100000;  // Pa
    c.c0  = 40;      // -> temp 20 C
    EXPECT_NEAR(compensate_temperature(0, c, 253952.0f), 20.0f, 1e-3f);
    EXPECT_NEAR(compensate_pressure(0, 0, c, 253952.0f, 253952.0f), 100000.0f, 1e-3f);
}

TEST(SPA06Data, AltitudeFormula)
{
    // At sea-level pressure the relative altitude is ~0 m.
    EXPECT_NEAR(altitude_m(1013.25f, 1013.25f), 0.0f, 1e-3f);
    // A ~12 hPa drop is roughly +100 m near sea level.
    EXPECT_GT(altitude_m(1000.0f, 1013.25f), 80.0f);
    EXPECT_LT(altitude_m(1000.0f, 1013.25f), 130.0f);
}
