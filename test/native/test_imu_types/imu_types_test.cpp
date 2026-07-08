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
