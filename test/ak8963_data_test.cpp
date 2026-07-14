/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <unit/unit_ak8963_data.hpp>

using namespace m5::unit::ak8963;

namespace {
// Pack a signed 16bit value little-endian (LSB first) into raw[lsb], raw[lsb+1].
void set_le16(Data& d, const size_t lsb, const int16_t v)
{
    d.raw[lsb]     = static_cast<uint8_t>(v & 0xFF);
    d.raw[lsb + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
}  // namespace

TEST(AK8963Data, RawAxesLittleEndian)
{
    Data d{};
    set_le16(d, 1, -1234);  // X at raw[1],[2]
    set_le16(d, 3, 5678);   // Y at raw[3],[4]
    set_le16(d, 5, -32768); // Z at raw[5],[6]
    EXPECT_EQ(d.x_raw(), -1234);
    EXPECT_EQ(d.y_raw(), 5678);
    EXPECT_EQ(d.z_raw(), -32768);
}

TEST(AK8963Data, DrdyAndOverflowBits)
{
    Data d{};
    d.raw[0] = 0x00;  // ST1
    d.raw[7] = 0x00;  // ST2
    EXPECT_FALSE(d.drdy());
    EXPECT_FALSE(d.overflow());
    d.raw[0] = 0x01;  // DRDY
    d.raw[7] = 0x08;  // HOFL
    EXPECT_TRUE(d.drdy());
    EXPECT_TRUE(d.overflow());
}

TEST(AK8963Data, AdjustedOutputIs16LsbPerMicroTesla)
{
    Data d{};
    set_le16(d, 1, 1000);  // X raw
    d.asa = {128, 128, 128};  // ASA=128 -> adjustment factor exactly 1.0
    // out = 1000 * 1.0 * (4912/32760) * 16 = 1000 * 2.39902... = 2399
    EXPECT_EQ(d.adj_x(), 2399);
}

TEST(AK8963Data, AsaScalesPerAxis)
{
    Data d{};
    set_le16(d, 3, 1000);      // Y raw
    d.asa = {128, 256 - 1, 0}; // ASAY=255 -> factor (255+128)/256 = 1.49609375
    // out = 1000 * 1.49609375 * (4912/32760) * 16 = round(3589.2) = 3589
    EXPECT_EQ(d.adj_y(), 3589);
}
