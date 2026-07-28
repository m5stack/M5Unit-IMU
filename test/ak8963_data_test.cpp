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
// ST2 bit4: the width the sample was taken at. adj_*() reads it, so any 16bit sample built here
// has to carry it -- a zeroed Data is a 14bit sample.
constexpr uint8_t BITM{0x10};
}  // namespace

TEST(AK8963Data, RawAxesLittleEndian)
{
    Data d{};
    set_le16(d, 1, -1234);   // X at raw[1],[2]
    set_le16(d, 3, 5678);    // Y at raw[3],[4]
    set_le16(d, 5, -32768);  // Z at raw[5],[6]
    EXPECT_EQ(d.x_raw(), -1234);
    EXPECT_EQ(d.y_raw(), 5678);
    EXPECT_EQ(d.z_raw(), -32768);
}

TEST(AK8963Data, StatusBits)
{
    Data d{};
    d.raw[0] = 0x00;  // ST1
    d.raw[7] = 0x00;  // ST2
    EXPECT_FALSE(d.drdy());
    EXPECT_FALSE(d.dor());
    EXPECT_FALSE(d.overflow());
    EXPECT_FALSE(d.bit16());  // BITM clear = 14bit output
    d.raw[0] = 0x03;          // DRDY | DOR
    d.raw[7] = 0x18;          // HOFL | BITM
    EXPECT_TRUE(d.drdy());
    EXPECT_TRUE(d.dor());
    EXPECT_TRUE(d.overflow());
    EXPECT_TRUE(d.bit16());
}

TEST(AK8963Data, AdjustedOutputIs16LsbPerMicroTesla)
{
    Data d{};
    set_le16(d, 1, 1000);     // X raw
    d.raw[7] = BITM;          // ST2: sample was taken in 16bit mode
    d.asa    = {128, 128, 128};  // ASA=128 -> adjustment factor exactly 1.0
    // out = 1000 * 1.0 * (4912/32760) * 16 = 1000 * 2.39902... = 2399
    EXPECT_EQ(d.adj_x(), 2399);
}

TEST(AK8963Data, AsaScalesPerAxis)
{
    Data d{};
    set_le16(d, 3, 1000);       // Y raw
    d.raw[7] = BITM;
    d.asa    = {128, 256 - 1, 0};  // ASAY=255 -> factor (255+128)/256 = 1.49609375
    // out = 1000 * 1.49609375 * (4912/32760) * 16 = round(3589.2) = 3589
    EXPECT_EQ(d.adj_y(), 3589);
}

TEST(AK8963Data, SensitivityComesFromTheSamplesOwnBitmBit)
{
    // Full scale is +/-4912uT at either width, so 14bit output is exactly 4x coarser per LSB. BITM
    // mirrors CNTL1's BIT into every sample, which is what lets adj_*() pick the right sensitivity
    // without the driver having to remember what the chip was set to when the sample was taken
    Data d16{}, d14{};
    set_le16(d16, 1, 1000);
    set_le16(d14, 1, 1000);
    d16.raw[7] = BITM;
    d14.raw[7] = 0x00;
    d16.asa = d14.asa = {128, 128, 128};

    EXPECT_EQ(d16.adj_x(), 2399);
    // out = 1000 * 1.0 * (4912/8190) * 16 = round(9596.1) = 9596
    EXPECT_EQ(d14.adj_x(), 9596);
    // 14bit data is sign-extended and right-aligned, not shifted, so the ratio is purely the
    // sensitivity: (4912/8190) / (4912/32760) = 4
    EXPECT_NEAR(static_cast<float>(d14.adj_x()) / d16.adj_x(), 4.0f, 0.01f);
}

TEST(AK8963Data, FourteenBitFullScaleMatchesSixteenBit)
{
    // +/-4912uT either way: the extreme raw value of each width has to land on the same field
    Data d16{}, d14{};
    set_le16(d16, 1, 32760);  // 16bit full scale
    set_le16(d14, 1, 8190);   // 14bit full scale
    d16.raw[7] = BITM;
    d14.raw[7] = 0x00;
    d16.asa = d14.asa = {128, 128, 128};

    // 4912uT * 16 LSB/uT = 78592
    EXPECT_EQ(d16.adj_x(), 78592);
    EXPECT_EQ(d14.adj_x(), 78592);
}

TEST(AK8963Setting, IntervalFollowsTheContinuousRate)
{
    EXPECT_EQ(interval_ms(MeasurementMode::Cont100Hz), 10u);
    EXPECT_EQ(interval_ms(MeasurementMode::Cont8Hz), 125u);
}

TEST(AK8963Setting, Cntl1ValuePacksBitAndMode)
{
    // BIT is bit4, the mode is the low nibble. 16bit + Cont100Hz (0x16) is what this driver used
    // when both were hard-coded
    EXPECT_EQ(cntl1_value(MeasurementMode::Cont100Hz, OutputBit::Bit16), 0x16);
    EXPECT_EQ(cntl1_value(MeasurementMode::Cont8Hz, OutputBit::Bit16), 0x12);
    EXPECT_EQ(cntl1_value(MeasurementMode::Cont100Hz, OutputBit::Bit14), 0x06);
    EXPECT_EQ(cntl1_value(MeasurementMode::Cont8Hz, OutputBit::Bit14), 0x02);

    // The mode nibble has to stay the raw CNTL1 encoding shared with Mode
    EXPECT_EQ(static_cast<uint8_t>(MeasurementMode::Cont8Hz), static_cast<uint8_t>(Mode::Cont8Hz));
    EXPECT_EQ(static_cast<uint8_t>(MeasurementMode::Cont100Hz), static_cast<uint8_t>(Mode::Cont100Hz));
}
