/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <unit/unit_bmm350_data.hpp>

#include <cmath>

using namespace m5::unit::bmm350;

namespace {

// Packs a signed 24bit value into raw[idx..idx+2] as little-endian
// (XLSB, LSB, MSB), the layout Data::x_raw()/y_raw()/z_raw()/temp_raw24()
// expect.
void set_raw24(Data& d, const size_t idx, const int32_t value)
{
    const uint32_t u = static_cast<uint32_t>(value) & 0xFFFFFFu;
    d.raw[idx]       = static_cast<uint8_t>(u & 0xFF);
    d.raw[idx + 1]   = static_cast<uint8_t>((u >> 8) & 0xFF);
    d.raw[idx + 2]   = static_cast<uint8_t>((u >> 16) & 0xFF);
}

}  // namespace

TEST(BMM350Data, FixSign24)
{
    EXPECT_EQ(fix_sign24(0x000000u), 0);
    EXPECT_EQ(fix_sign24(0x7FFFFFu), 8388607);
    EXPECT_EQ(fix_sign24(0x800000u), -8388608);
    EXPECT_EQ(fix_sign24(0xFFFFFFu), -1);
}

TEST(BMM350Data, LittleEndian24AssemblyX)
{
    Data d{};
    set_raw24(d, 0, 0x001234);
    EXPECT_EQ(d.x_raw(), 0x1234);

    set_raw24(d, 0, -8388608);
    EXPECT_EQ(d.x_raw(), -8388608);
}

TEST(BMM350Data, LittleEndian24AssemblyY)
{
    Data d{};
    set_raw24(d, 3, 0x00ABCD);
    EXPECT_EQ(d.y_raw(), 0x00ABCD);

    set_raw24(d, 3, -1);
    EXPECT_EQ(d.y_raw(), -1);
}

TEST(BMM350Data, LittleEndian24AssemblyZ)
{
    Data d{};
    set_raw24(d, 6, 0x654321);  // MSB=0x65, top bit clear -> positive
    EXPECT_EQ(d.z_raw(), 0x654321);

    set_raw24(d, 6, -12345);
    EXPECT_EQ(d.z_raw(), -12345);
}

TEST(BMM350Data, LittleEndian24AssemblyTemp)
{
    Data d{};
    set_raw24(d, 9, 300);
    EXPECT_EQ(d.temp_raw24(), 300);

    set_raw24(d, 9, -300);
    EXPECT_EQ(d.temp_raw24(), -300);
}

TEST(BMM350Data, ConversionCoefficients)
{
    EXPECT_NEAR(lsb_to_ut_xy(), 0.00707f, 1e-5f);
    EXPECT_NEAR(lsb_to_ut_z(), 0.0071750f, 1e-5f);
    EXPECT_NEAR(temp_res_raw24(), 0.00098128f, 1e-8f);
    EXPECT_NEAR(TEMP_OFFSET, -25.49f, 1e-6f);
}

TEST(BMM350Data, ClampAveraging)
{
    EXPECT_EQ(clamp_averaging(DataRate::Rate400Hz, Averaging::None), Averaging::None);
    EXPECT_EQ(clamp_averaging(DataRate::Rate400Hz, Averaging::X8), Averaging::None);

    EXPECT_EQ(clamp_averaging(DataRate::Rate200Hz, Averaging::None), Averaging::None);
    EXPECT_EQ(clamp_averaging(DataRate::Rate200Hz, Averaging::X2), Averaging::X2);
    EXPECT_EQ(clamp_averaging(DataRate::Rate200Hz, Averaging::X8), Averaging::X2);

    EXPECT_EQ(clamp_averaging(DataRate::Rate100Hz, Averaging::X4), Averaging::X4);
    EXPECT_EQ(clamp_averaging(DataRate::Rate100Hz, Averaging::X8), Averaging::X4);

    EXPECT_EQ(clamp_averaging(DataRate::Rate50Hz, Averaging::X8), Averaging::X8);
    EXPECT_EQ(clamp_averaging(DataRate::Rate1_5625Hz, Averaging::X8), Averaging::X8);
}

TEST(BMM350Data, OtpCompensationNeutralPassthrough)
{
    otp_compensation_t comp{};  // all-zero except t0

    EXPECT_FLOAT_EQ(comp.compensate_temp(10.0f), 10.0f);
    EXPECT_FLOAT_EQ(comp.compensate_temp(-40.0f), -40.0f);

    float m[3] = {100.0f, -50.0f, 12.5f};
    comp.compensate_mag(m, comp.compensate_temp(10.0f));
    EXPECT_FLOAT_EQ(m[0], 100.0f);
    EXPECT_FLOAT_EQ(m[1], -50.0f);
    EXPECT_FLOAT_EQ(m[2], 12.5f);
}

TEST(BMM350Data, OtpCompensationHandCalculatedTemperatureCoefficients)
{
    // T = (1+t_sens)*T_raw + t_offs = (1+0.01)*10.0 + 1.0 = 11.1
    // M_x += tco_x*(T-t0) = 100 + 0.5*(11.1-23) = 94.05
    // M_x /= 1+tcs_x*(T-t0) = 94.05 / (1+0.001*(11.1-23)) = 94.05/0.9881 = 95.1826738...
    otp_compensation_t comp{};
    comp.t_sens = 0.01f;
    comp.t_offs = 1.0f;
    comp.t0     = 23.0f;
    comp.tco[0] = 0.5f;
    comp.tcs[0] = 0.001f;

    const float t_comp = comp.compensate_temp(10.0f);
    EXPECT_NEAR(t_comp, 11.1f, 1e-5f);

    float m[3] = {100.0f, 0.0f, 0.0f};
    comp.compensate_mag(m, t_comp);
    EXPECT_NEAR(m[0], 95.1826738f, 1e-3f);
    EXPECT_NEAR(m[1], 0.0f, 1e-6f);
    EXPECT_NEAR(m[2], 0.0f, 1e-6f);
}

TEST(BMM350Data, OtpCompensationCrossAxis)
{
    // Verified against the Bosch BMM350_SensorAPI cross-axis formula
    // (bmm350_get_compensated_mag_xyz_temp_data, main/bmm350.c):
    //   denom = 1 - cross_yx*cross_xy
    //   x = (m0 - cross_xy*m1) / denom
    //   y = (m1 - cross_yx*m0) / denom
    //   z = m2 + (m0*(cross_yx*cross_zy - cross_zx) - m1*(cross_zy - cross_xy*cross_zx)) / denom
    otp_compensation_t comp{};
    comp.cross[0] = 0.01f;  // x_y
    comp.cross[1] = 0.02f;  // y_x
    comp.cross[2] = 0.03f;  // z_x
    comp.cross[3] = 0.04f;  // z_y

    float m[3] = {10.0f, 20.0f, 30.0f};
    comp.compensate_mag(m, comp.compensate_temp(23.0f));  // t_comp == t0 -> tco/tcs terms are no-ops
    EXPECT_NEAR(m[0], 9.801960392f, 1e-4f);
    EXPECT_NEAR(m[1], 19.803960792f, 1e-4f);
    EXPECT_NEAR(m[2], 28.913782757f, 1e-4f);
}

TEST(BMM350Data, TempAdcNormalizationWithinTolerance)
{
    const int32_t raw24_values[] = {0, 12345, -54321, 300, -300, 100000, -100000, 8388607, -8388608};
    for (const int32_t raw24 : raw24_values) {
        Data d{};
        set_raw24(d, 9, raw24);
        ASSERT_EQ(d.temp_raw24(), raw24);

        const float actual = static_cast<float>(raw24) * temp_res_raw24() + TEMP_OFFSET;
        const float approx = static_cast<float>(d.temp_adc()) * (temp_res_raw24() * 256.0f) + TEMP_OFFSET;
        const float error  = std::fabs(actual - approx);
        EXPECT_LT(error, 0.26f) << "raw24=" << raw24 << " actual=" << actual << " approx=" << approx;
    }
}
