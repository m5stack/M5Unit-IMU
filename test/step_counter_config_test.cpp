/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/motion_events.hpp>

using m5::imu::StepCounterConfig;

TEST(StepCounterConfig, defaults_to_counter_mode_watermark_1)
{
    StepCounterConfig c;
    EXPECT_EQ(c.mode, StepCounterConfig::Mode::Counter);
    EXPECT_EQ(c.watermark_steps, 1);
}

TEST(StepCounterConfig, explicit_detector_mode)
{
    StepCounterConfig c{StepCounterConfig::Mode::Detector, 0};
    EXPECT_EQ(c.mode, StepCounterConfig::Mode::Detector);
}
