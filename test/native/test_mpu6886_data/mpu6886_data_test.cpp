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
