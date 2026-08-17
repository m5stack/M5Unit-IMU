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
using m5::imu::PickUpGestureConfig;

namespace {
// No tunable params at this tier; the recorder only verifies that the cfg pointer
// handed to enableEvent() arrives at the driver unchanged, matching the
// void*-cfg-per-kind contract declared on MotionEventSource.
class RecordingSource : public MotionEventSource {
public:
    MotionEventKind motionEventSpec() const override
    {
        return MotionEventKind::PickUpGesture;
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

TEST(PickUpGestureConfig, default_constructible)
{
    PickUpGestureConfig c;
    (void)c;
}

TEST(PickUpGestureConfig, passed_through_enableEvent_as_void_ptr_round_trips)
{
    PickUpGestureConfig cfg;
    RecordingSource src;
    EXPECT_TRUE(src.enableEvent(MotionEventKind::PickUpGesture, &cfg));
    EXPECT_EQ(src.last_cfg, &cfg);
}
