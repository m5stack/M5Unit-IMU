/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/motion_events.hpp>

using m5::imu::MotionEventKind;
using m5::imu::MotionEventSource;
using m5::imu::MotionEventStatus;
using m5::imu::WristGestureConfig;

namespace {
// No tunable params yet; the recorder only verifies that the cfg pointer
// handed to enableEvent() arrives at the driver unchanged, matching the
// void*-cfg-per-kind contract declared on MotionEventSource.
class RecordingSource : public MotionEventSource {
public:
    MotionEventKind motionEventSpec() const override
    {
        return MotionEventKind::WristGesture;
    }
    MotionEventKind enabledEvents() const override
    {
        return MotionEventKind::None;
    }
    bool enableEvent(MotionEventKind, const void* cfg) override
    {
        last_cfg = cfg;
        return true;
    }
    bool disableEvent(MotionEventKind) override
    {
        return true;
    }
    bool readEventStatus(MotionEventStatus&) override
    {
        return false;
    }
    const void* last_cfg{nullptr};
};
}  // namespace

TEST(WristGestureConfig, default_constructible)
{
    WristGestureConfig c;
    (void)c;
}

TEST(WristGestureConfig, default_arm_is_left)
{
    WristGestureConfig c;
    EXPECT_EQ(c.arm, WristGestureConfig::Arm::Left);
}

TEST(WristGestureConfig, explicit_arm_right)
{
    WristGestureConfig c{WristGestureConfig::Arm::Right};
    EXPECT_EQ(c.arm, WristGestureConfig::Arm::Right);
}

TEST(WristGestureConfig, passed_through_enableEvent_as_void_ptr_round_trips)
{
    WristGestureConfig cfg;
    RecordingSource src;
    EXPECT_TRUE(src.enableEvent(MotionEventKind::WristGesture, &cfg));
    EXPECT_EQ(src.last_cfg, &cfg);
}
