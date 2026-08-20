/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <unit/unit_bmi270_data.hpp>

using namespace m5::unit::bmi270;

TEST(BMI270Data, LittleEndianAssembly)
{
    Data d{};
    // accel_x = 0x1234, gyro_z = 0x8000 (min, negative)
    d.raw[0]  = 0x34;
    d.raw[1]  = 0x12;
    d.raw[10] = 0x00;
    d.raw[11] = 0x80;

    EXPECT_EQ(d.accel_x(), 0x1234);
    EXPECT_EQ(d.gyro_z(), -32768);
}

TEST(BMI270Data, AllAccessorsByteOffsetAndEndianness)
{
    Data d{};
    // Each accessor's 2-byte slot is filled with a distinct value, LSB first, so a
    // wrong offset or a big-endian mix-up is caught (BMI270 data registers are
    // little-endian, unlike UnitMPU6886's big-endian burst)
    d.raw[0]  = 0x22;
    d.raw[1]  = 0x11;  // accel_x = 0x1122
    d.raw[2]  = 0x02;
    d.raw[3]  = 0x01;  // accel_y = 0x0102
    d.raw[4]  = 0x04;
    d.raw[5]  = 0x03;  // accel_z = 0x0304
    d.raw[6]  = 0x08;
    d.raw[7]  = 0x07;  // gyro_x = 0x0708
    d.raw[8]  = 0x0A;
    d.raw[9]  = 0x09;  // gyro_y = 0x090A
    d.raw[10] = 0x0C;
    d.raw[11] = 0x0B;  // gyro_z = 0x0B0C

    EXPECT_EQ(d.accel_x(), 0x1122);
    EXPECT_EQ(d.accel_y(), 0x0102);
    EXPECT_EQ(d.accel_z(), 0x0304);
    EXPECT_EQ(d.gyro_x(), 0x0708);
    EXPECT_EQ(d.gyro_y(), 0x090A);
    EXPECT_EQ(d.gyro_z(), 0x0B0C);
}

TEST(BMI270Data, AccelResAllRanges)
{
    EXPECT_FLOAT_EQ(accel_res(AccelRange::G2), 2.0f / 32768);
    EXPECT_FLOAT_EQ(accel_res(AccelRange::G4), 4.0f / 32768);
    EXPECT_FLOAT_EQ(accel_res(AccelRange::G8), 8.0f / 32768);
    EXPECT_FLOAT_EQ(accel_res(AccelRange::G16), 16.0f / 32768);
}

TEST(BMI270Data, GyroResAllRangesReversedOrder)
{
    // BMI270 encodes value 0 as the *largest* range (opposite of UnitMPU6886)
    EXPECT_EQ(static_cast<uint8_t>(GyroRange::DPS2000), 0);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS2000), 2000.0f / 32768);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS1000), 1000.0f / 32768);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS500), 500.0f / 32768);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS250), 250.0f / 32768);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS125), 125.0f / 32768);
}

TEST(BMI270Data, OdrIntervalMs)
{
    EXPECT_EQ(odr_interval_ms(GyroODR::Rate25Hz), 40u);
    EXPECT_EQ(odr_interval_ms(GyroODR::Rate50Hz), 20u);
    EXPECT_EQ(odr_interval_ms(GyroODR::Rate100Hz), 10u);
    EXPECT_EQ(odr_interval_ms(GyroODR::Rate200Hz), 5u);
    EXPECT_EQ(odr_interval_ms(GyroODR::Rate400Hz), 3u);
    EXPECT_EQ(odr_interval_ms(GyroODR::Rate800Hz), 2u);
    EXPECT_EQ(odr_interval_ms(GyroODR::Rate1600Hz), 1u);
    EXPECT_EQ(odr_interval_ms(GyroODR::Rate3200Hz), 1u);
}

TEST(BMI270Data, ConfValuesReproduceTheRegisterResetDefaults)
{
    // The datasheet gives ACC_CONF a 0xA8 reset value and GYR_CONF 0xA9, both of which decode to
    // Performance / normal bandwidth at the power-on ODR. Reproducing them is what pins the bit
    // packing (perf in bit7, bwp in bits[6:4] resp. [5:4], odr in bits[3:0])
    EXPECT_EQ(acc_conf_value(FilterPerf::Performance, AccelBwp::NormAvg4, AccelODR::Rate100Hz), 0xA8);
    EXPECT_EQ(gyr_conf_value(FilterPerf::Performance, GyroBwp::Norm, GyroODR::Rate200Hz), 0xA9);

    // gyr_noise_perf (bit6) stays at its 0 reset value, so the gyro register never sets it
    EXPECT_EQ(gyr_conf_value(FilterPerf::Performance, GyroBwp::Norm, GyroODR::Rate200Hz) & 0x40, 0);

    // PowerOptimized clears bit7; the averaging factors land in bits[6:4]
    EXPECT_EQ(acc_conf_value(FilterPerf::PowerOptimized, AccelBwp::Avg128, AccelODR::Rate100Hz), 0x78);
    EXPECT_EQ(acc_conf_value(FilterPerf::PowerOptimized, AccelBwp::Osr4Avg1, AccelODR::Rate12_5Hz), 0x05);
}

TEST(BMI270Data, AveragingBandwidthIsReservedInPerformanceMode)
{
    // Avg16 and up only mean anything while the filter averages, i.e. in PowerOptimized mode
    EXPECT_TRUE(bwp_fits(FilterPerf::Performance, AccelBwp::Osr4Avg1));
    EXPECT_TRUE(bwp_fits(FilterPerf::Performance, AccelBwp::NormAvg4));
    EXPECT_TRUE(bwp_fits(FilterPerf::Performance, AccelBwp::CicAvg8));
    EXPECT_FALSE(bwp_fits(FilterPerf::Performance, AccelBwp::Avg16));
    EXPECT_FALSE(bwp_fits(FilterPerf::Performance, AccelBwp::Avg128));

    // PowerOptimized accepts every value
    EXPECT_TRUE(bwp_fits(FilterPerf::PowerOptimized, AccelBwp::Osr4Avg1));
    EXPECT_TRUE(bwp_fits(FilterPerf::PowerOptimized, AccelBwp::Avg16));
    EXPECT_TRUE(bwp_fits(FilterPerf::PowerOptimized, AccelBwp::Avg128));
}
