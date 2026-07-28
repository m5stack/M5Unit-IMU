/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <unit/unit_mag3110_data.hpp>

using namespace m5::unit::mag3110;

namespace {
// Pack a signed 16bit value big-endian (MSB first) into raw[msb], raw[msb+1].
void set_be16(Data& d, const size_t msb, const int16_t v)
{
    d.raw[msb]     = static_cast<uint8_t>((v >> 8) & 0xFF);
    d.raw[msb + 1] = static_cast<uint8_t>(v & 0xFF);
}
}  // namespace

TEST(MAG3110Data, RawAxesBigEndian)
{
    Data d{};
    set_be16(d, 1, -1234);   // X at raw[1](MSB),[2](LSB)
    set_be16(d, 3, 5678);    // Y at raw[3],[4]
    set_be16(d, 5, -32768);  // Z at raw[5],[6]
    EXPECT_EQ(d.x_raw(), -1234);
    EXPECT_EQ(d.y_raw(), 5678);
    EXPECT_EQ(d.z_raw(), -32768);
}

TEST(MAG3110Data, DrdyBit)
{
    Data d{};
    d.raw[0] = 0x00;  // DR_STATUS
    EXPECT_FALSE(d.drdy());
    d.raw[0] = 0x08;  // ZYXDR
    EXPECT_TRUE(d.drdy());
}

TEST(MAG3110Data, RescaledTo16LsbPerMicroTesla)
{
    Data d{};
    set_be16(d, 1, 1000);  // X raw
    // out = 1000 * 0.10 µT/LSB * 16 LSB/µT = 1600
    EXPECT_EQ(d.adj_x(), 1600);

    set_be16(d, 3, -500);  // Y raw
    // out = -500 * 0.10 * 16 = -800
    EXPECT_EQ(d.adj_y(), -800);
}

TEST(MAG3110Setting, Ctrl1ValuePacksRateAndOverSampling)
{
    // DR is bits[7:5], OS bits[4:3], AC bit0. Rate80Hz + X16 is 0x01, the value this driver used
    // when both fields were hard-coded
    EXPECT_EQ(ctrl_reg1_active(DataRate::Rate80Hz, OverSampling::X16), 0x01);
    EXPECT_EQ(ctrl_reg1_active(DataRate::Rate80Hz, OverSampling::X128), 0x19);
    EXPECT_EQ(ctrl_reg1_active(DataRate::Rate0_63Hz, OverSampling::X16), 0xE1);
    EXPECT_EQ(ctrl_reg1_active(DataRate::Rate0_63Hz, OverSampling::X128), 0xF9);
    // FR (bit2) and TM (bit1) stay clear: FR would break the 7 byte burst read, TM is one-shot
    for (uint8_t r = 0; r < 8; ++r) {
        for (uint8_t o = 0; o < 4; ++o) {
            const uint8_t v = ctrl_reg1_active(static_cast<DataRate>(r), static_cast<OverSampling>(o));
            EXPECT_EQ(v & 0x06, 0) << "DR " << (int)r << " OS " << (int)o;
            EXPECT_EQ(v & 0x01, 0x01) << "DR " << (int)r << " OS " << (int)o;
        }
    }
}

TEST(MAG3110Setting, OutputRateMatchesDatasheetTable32)
{
    // Every row of datasheet Table 32 (DR x OS = 32 combinations), in milli-Hz. Reproducing the
    // whole table is what pins the "oversampling divides the rate" rule the interval derives from
    static const uint32_t expected[8][4] = {
        {80000, 40000, 20000, 10000},  // DR=000
        {40000, 20000, 10000, 5000},   // DR=001
        {20000, 10000, 5000, 2500},    // DR=010
        {10000, 5000, 2500, 1250},     // DR=011
        {5000, 2500, 1250, 630},       // DR=100 (datasheet 0.63Hz)
        {2500, 1250, 630, 310},        // DR=101 (0.63 / 0.31)
        {1250, 630, 310, 160},         // DR=110
        {630, 310, 160, 80},           // DR=111 (0.08Hz)
    };
    for (uint8_t r = 0; r < 8; ++r) {
        for (uint8_t o = 0; o < 4; ++o) {
            const auto got = output_rate_mhz(static_cast<DataRate>(r), static_cast<OverSampling>(o));
            // The table rounds to 2 decimals, so allow the integer division to differ in the last
            // milli-Hz (0.63/8 is 78.75mHz, tabulated as 0.08Hz)
            EXPECT_NEAR(static_cast<float>(got), static_cast<float>(expected[r][o]), expected[r][o] * 0.02f + 1.0f)
                << "DR " << (int)r << " OS " << (int)o;
        }
    }
}

TEST(MAG3110Setting, IntervalIsTheRoundedUpOutputPeriod)
{
    // Rate80Hz + X16 has to stay 13ms: that is the interval the driver used before the rate was
    // configurable, so the default configuration keeps its timing
    EXPECT_EQ(interval_ms(DataRate::Rate80Hz, OverSampling::X16), 13u);
    EXPECT_EQ(interval_ms(DataRate::Rate10Hz, OverSampling::X16), 100u);
    EXPECT_EQ(interval_ms(DataRate::Rate80Hz, OverSampling::X128), 100u);
    EXPECT_EQ(interval_ms(DataRate::Rate2_5Hz, OverSampling::X32), 800u);

    // Slowest setting: ~0.08Hz, i.e. one sample every 12.5 seconds. Without deriving the interval
    // update() would keep issuing a 7 byte read every 13ms and miss ~960 times per sample
    EXPECT_GT(interval_ms(DataRate::Rate0_63Hz, OverSampling::X128), 12000u);

    // Monotonic in both axes: a slower rate or a larger oversampling can never poll more often
    for (uint8_t r = 0; r < 8; ++r) {
        for (uint8_t o = 0; o < 4; ++o) {
            const auto here = interval_ms(static_cast<DataRate>(r), static_cast<OverSampling>(o));
            if (r + 1 < 8) {
                EXPECT_LE(here, interval_ms(static_cast<DataRate>(r + 1), static_cast<OverSampling>(o)));
            }
            if (o + 1 < 4) {
                EXPECT_LE(here, interval_ms(static_cast<DataRate>(r), static_cast<OverSampling>(o + 1)));
            }
        }
    }
}
