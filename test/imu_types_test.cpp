/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/imu_types.hpp>

using namespace m5::imu;

TEST(ImuTypes, CapabilityBitwiseOr)
{
    const Capability c = Capability::Accel | Capability::Gyro;
    EXPECT_TRUE(any(c & Capability::Accel));
    EXPECT_TRUE(any(c & Capability::Gyro));
    EXPECT_FALSE(any(c & Capability::Mag));
    EXPECT_FALSE(any(c & Capability::Baro));
}

TEST(ImuTypes, CapabilityOrAssign)
{
    Capability c = Capability::None;
    EXPECT_FALSE(any(c));
    c |= Capability::Mag;
    EXPECT_TRUE(any(c & Capability::Mag));
    c |= Capability::Accel;
    EXPECT_TRUE(any(c & Capability::Mag));
    EXPECT_TRUE(any(c & Capability::Accel));
}

TEST(ImuTypes, RawDataDefaultZero)
{
    const imu_raw_data_t d{};
    EXPECT_EQ(d.accel[0], 0);
    EXPECT_EQ(d.gyro[2], 0);
    EXPECT_EQ(d.mag[1], 0);
    EXPECT_EQ(d.baro, 0);
    EXPECT_EQ(d.temp, 0);
}

TEST(ImuTypes, CapabilityTempBit)
{
    const Capability c = Capability::Accel | Capability::Temp;
    EXPECT_TRUE(any(c & Capability::Accel));
    EXPECT_TRUE(any(c & Capability::Temp));
    EXPECT_FALSE(any(c & Capability::Gyro));
    EXPECT_FALSE(any(c & Capability::Mag));
    EXPECT_FALSE(any(c & Capability::Baro));

    Capability t = Capability::None;
    t |= Capability::Temp;
    EXPECT_TRUE(any(t & Capability::Temp));
    EXPECT_FALSE(any(t & Capability::Accel));
    EXPECT_FALSE(any(t & Capability::Gyro));
    EXPECT_FALSE(any(t & Capability::Mag));
    EXPECT_FALSE(any(t & Capability::Baro));
}

TEST(ImuTypes, ConvertParamDefaults)
{
    const imu_convert_param_t p{};
    EXPECT_FLOAT_EQ(p.accel_res, 8.0f / 32768);
    EXPECT_FLOAT_EQ(p.gyro_res, 2000.0f / 32768);
    EXPECT_FLOAT_EQ(p.mag_res, 4912.0f / 32760);
    EXPECT_FLOAT_EQ(p.temp_res, 1.0f);
    EXPECT_FLOAT_EQ(p.temp_offset, 0.0f);
}
