/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/motion_events.hpp>

using m5::imu::NoMotionConfig;

TEST(NoMotionConfig, defaults)
{
    NoMotionConfig c;
    EXPECT_EQ(c.threshold_mg, 70);
    EXPECT_EQ(c.duration_ms, 5000u);
    EXPECT_TRUE(c.axis_x && c.axis_y && c.axis_z);
}

TEST(NoMotionConfig, explicit_ctor)
{
    NoMotionConfig c{100, 1000, false, true, false};
    EXPECT_EQ(c.threshold_mg, 100);
    EXPECT_EQ(c.duration_ms, 1000u);
    EXPECT_FALSE(c.axis_x);
    EXPECT_TRUE(c.axis_y);
    EXPECT_FALSE(c.axis_z);
}
