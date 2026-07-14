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
