/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <unit/unit_mpu6886_data.hpp>

using namespace m5::unit::mpu6886;

TEST(MPU6886Data, BigEndianAssembly)
{
    Data d{};
    // accel_x = 0x1234, temp = 0xFF38 (負値), gyro_z = 0x8000 (最小値)
    d.raw[0]  = 0x12;
    d.raw[1]  = 0x34;
    d.raw[6]  = 0xFF;
    d.raw[7]  = 0x38;
    d.raw[12] = 0x80;
    d.raw[13] = 0x00;

    EXPECT_EQ(d.accel_x(), 0x1234);
    EXPECT_EQ(d.temp_adc(), -200);
    EXPECT_EQ(d.gyro_z(), -32768);
}

TEST(MPU6886Data, Temperature)
{
    Data d{};
    // temp_adc = 0 -> 25.0 degC
    EXPECT_FLOAT_EQ(d.temperature(), 25.0f);
    // temp_adc = 3268 -> 25.0 + 10.0
    d.raw[6] = 0x0C;
    d.raw[7] = 0xC4;  // 3268 = 0x0CC4
    EXPECT_NEAR(d.temperature(), 35.0f, 0.01f);
}

TEST(MPU6886Data, AllAccessorsByteOffset)
{
    Data d{};
    // Each accessor's 2-byte slot is filled with a distinct value so a
    // wrong offset is caught even if adjacent accessors are swapped.
    d.raw[0]  = 0x11;
    d.raw[1]  = 0x22;  // accel_x = 0x1122
    d.raw[2]  = 0x01;
    d.raw[3]  = 0x02;  // accel_y = 0x0102
    d.raw[4]  = 0x03;
    d.raw[5]  = 0x04;  // accel_z = 0x0304
    d.raw[6]  = 0x05;
    d.raw[7]  = 0x06;  // temp_adc = 0x0506
    d.raw[8]  = 0x07;
    d.raw[9]  = 0x08;  // gyro_x = 0x0708
    d.raw[10] = 0x09;
    d.raw[11] = 0x0A;  // gyro_y = 0x090A
    d.raw[12] = 0x0B;
    d.raw[13] = 0x0C;  // gyro_z = 0x0B0C

    EXPECT_EQ(d.accel_x(), 0x1122);
    EXPECT_EQ(d.accel_y(), 0x0102);
    EXPECT_EQ(d.accel_z(), 0x0304);
    EXPECT_EQ(d.temp_adc(), 0x0506);
    EXPECT_EQ(d.gyro_x(), 0x0708);
    EXPECT_EQ(d.gyro_y(), 0x090A);
    EXPECT_EQ(d.gyro_z(), 0x0B0C);
}

TEST(MPU6886Data, AccelResAllRanges)
{
    EXPECT_FLOAT_EQ(accel_res(AccelRange::G2), 2.0f / 32768);
    EXPECT_FLOAT_EQ(accel_res(AccelRange::G4), 4.0f / 32768);
    EXPECT_FLOAT_EQ(accel_res(AccelRange::G8), 8.0f / 32768);
    EXPECT_FLOAT_EQ(accel_res(AccelRange::G16), 16.0f / 32768);
}

TEST(MPU6886Data, GyroResAllRanges)
{
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS250), 250.0f / 32768);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS500), 500.0f / 32768);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS1000), 1000.0f / 32768);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS2000), 2000.0f / 32768);
}
