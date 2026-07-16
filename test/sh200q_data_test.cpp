/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <unit/unit_sh200q_data.hpp>

using namespace m5::unit::sh200q;

TEST(SH200QData, LittleEndianAssembly)
{
    Data d{};
    // accel_x = 0x1234, gyro_x = 0x8000 (最小値), temp = 0xFF38 (負値) -- LSB が先 (MPU 系と逆)
    d.raw[0]  = 0x34;
    d.raw[1]  = 0x12;
    d.raw[6]  = 0x00;
    d.raw[7]  = 0x80;
    d.raw[12] = 0x38;
    d.raw[13] = 0xFF;

    EXPECT_EQ(d.accel_x(), 0x1234);
    EXPECT_EQ(d.gyro_x(), -32768);
    EXPECT_EQ(d.temp_adc(), -200);
}

TEST(SH200QData, Temperature)
{
    Data d{};
    // temp_adc = 0 -> 21.0 degC
    EXPECT_FLOAT_EQ(d.temperature(), 21.0f);
    // temp_adc = 3339 -> 21.0 + ~10.0
    d.raw[12] = 0x0B;
    d.raw[13] = 0x0D;  // 3339 = 0x0D0B (little-endian)
    EXPECT_NEAR(d.temperature(), 31.0f, 0.01f);
}

TEST(SH200QData, AllAccessorsByteOffset)
{
    Data d{};
    // Each accessor's 2-byte slot is filled with a distinct value so a
    // wrong offset is caught even if adjacent accessors are swapped.
    // Layout is accel6 + gyro6 + temp2 (temp LAST, unlike the MPU family).
    d.raw[0]  = 0x22;
    d.raw[1]  = 0x11;  // accel_x = 0x1122
    d.raw[2]  = 0x02;
    d.raw[3]  = 0x01;  // accel_y = 0x0102
    d.raw[4]  = 0x04;
    d.raw[5]  = 0x03;  // accel_z = 0x0304
    d.raw[6]  = 0x06;
    d.raw[7]  = 0x05;  // gyro_x = 0x0506
    d.raw[8]  = 0x08;
    d.raw[9]  = 0x07;  // gyro_y = 0x0708
    d.raw[10] = 0x0A;
    d.raw[11] = 0x09;  // gyro_z = 0x090A
    d.raw[12] = 0x0C;
    d.raw[13] = 0x0B;  // temp_adc = 0x0B0C

    EXPECT_EQ(d.accel_x(), 0x1122);
    EXPECT_EQ(d.accel_y(), 0x0102);
    EXPECT_EQ(d.accel_z(), 0x0304);
    EXPECT_EQ(d.gyro_x(), 0x0506);
    EXPECT_EQ(d.gyro_y(), 0x0708);
    EXPECT_EQ(d.gyro_z(), 0x090A);
    EXPECT_EQ(d.temp_adc(), 0x0B0C);
}

TEST(SH200QData, AccelResAllRanges)
{
    EXPECT_FLOAT_EQ(accel_res(AccelRange::G4), 4.0f / 32768);
    EXPECT_FLOAT_EQ(accel_res(AccelRange::G8), 8.0f / 32768);
    EXPECT_FLOAT_EQ(accel_res(AccelRange::G16), 16.0f / 32768);
}

TEST(SH200QData, GyroResAllRanges)
{
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS2000), 2000.0f / 32768);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS1000), 1000.0f / 32768);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS500), 500.0f / 32768);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS250), 250.0f / 32768);
    EXPECT_FLOAT_EQ(gyro_res(GyroRange::DPS125), 125.0f / 32768);
}

TEST(SH200QData, GyroOdrHz)
{
    EXPECT_EQ(gyro_odr_hz(GyroOdr::HZ1000), 1000U);
    EXPECT_EQ(gyro_odr_hz(GyroOdr::HZ500), 500U);
    EXPECT_EQ(gyro_odr_hz(GyroOdr::HZ250), 250U);
}
