/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include "imu/motion_events.hpp"

namespace {
using namespace m5::imu;

// Overrides only the pure-virtual API; used to check the default capability masks.
class MinimalMock : public MotionEventSource {
public:
    MotionEventKind motionEventSpec() const override
    {
        return MotionEventKind::None;
    }
    MotionEventKind enabledEvents() const override
    {
        return MotionEventKind::None;
    }
    bool enableEvent(MotionEventKind, const void*) override
    {
        return false;
    }
    bool disableEvent(MotionEventKind) override
    {
        return true;
    }
    bool readEventStatus(MotionEventStatus&) override
    {
        return false;
    }
};

// Overrides the capability masks and can be told to fire ActivityChange / WristGesture
// on the next readEventStatus() call, to exercise the typed-field aggregation path.
class CapabilityMock : public MotionEventSource {
public:
    MotionEventKind motionEventSpec() const override
    {
        return MotionEventKind::ActivityChange | MotionEventKind::WristGesture;
    }
    MotionEventKind enabledEvents() const override
    {
        return _enabled;
    }
    bool enableEvent(MotionEventKind k, const void*) override
    {
        if (!any(motionEventSpec() & k)) return false;
        _enabled |= k;
        return true;
    }
    bool disableEvent(MotionEventKind k) override
    {
        _enabled &= ~k;
        return true;
    }
    bool disableAllEvents() override
    {
        _enabled = MotionEventKind::None;
        return true;
    }
    bool readEventStatus(MotionEventStatus& out) override
    {
        if (_pending == MotionEventKind::None) return false;
        out.fired = _pending;
        if (any(_pending & MotionEventKind::ActivityChange)) out.activity_type = _next_activity;
        if (any(_pending & MotionEventKind::WristGesture)) out.gesture_type = _next_gesture;
        _pending = MotionEventKind::None;
        return true;
    }
    uint32_t supportedActivityTypes() const override
    {
        return (1u << static_cast<uint8_t>(ActivityType::Stationary)) |
               (1u << static_cast<uint8_t>(ActivityType::Walking)) |
               (1u << static_cast<uint8_t>(ActivityType::Running));
    }
    uint32_t supportedGestureTypes() const override
    {
        return (1u << static_cast<uint8_t>(GestureType::WristPushArmDown)) |
               (1u << static_cast<uint8_t>(GestureType::WristPivotUp)) |
               (1u << static_cast<uint8_t>(GestureType::WristShake)) |
               (1u << static_cast<uint8_t>(GestureType::WristFlickIn)) |
               (1u << static_cast<uint8_t>(GestureType::WristFlickOut));
    }

    void fire_activity(ActivityType t)
    {
        _pending |= MotionEventKind::ActivityChange;
        _next_activity = t;
    }
    void fire_gesture(GestureType t)
    {
        _pending |= MotionEventKind::WristGesture;
        _next_gesture = t;
    }

private:
    MotionEventKind _enabled{MotionEventKind::None};
    MotionEventKind _pending{MotionEventKind::None};
    ActivityType _next_activity{ActivityType::Unknown};
    GestureType _next_gesture{GestureType::None};
};
}  // namespace

TEST(MotionEventCapability, default_supported_masks_are_zero)
{
    MinimalMock m;
    EXPECT_EQ(m.supportedActivityTypes(), 0u);
    EXPECT_EQ(m.supportedGestureTypes(), 0u);
}

TEST(MotionEventCapability, override_reports_chip_specific_masks)
{
    CapabilityMock m;
    EXPECT_TRUE(m.supportedActivityTypes() & (1u << static_cast<uint8_t>(ActivityType::Walking)));
    EXPECT_FALSE(m.supportedActivityTypes() & (1u << static_cast<uint8_t>(ActivityType::Cycling)));
    EXPECT_TRUE(m.supportedGestureTypes() & (1u << static_cast<uint8_t>(GestureType::WristShake)));
}

TEST(MotionEventCapability, status_defaults_are_sentinel_values)
{
    MotionEventStatus s;
    EXPECT_EQ(s.activity_type, ActivityType::Unknown);
    EXPECT_EQ(s.gesture_type, GestureType::None);
}

TEST(MotionEventCapability, controller_aggregates_typed_fields_and_resets_each_tick)
{
    CapabilityMock m;
    MotionEventController c;
    ASSERT_TRUE(c.add(m));
    c.enable(MotionEventKind::ActivityChange, nullptr);
    c.enable(MotionEventKind::WristGesture, nullptr);

    m.fire_activity(ActivityType::Walking);
    c.update(0.02f);
    MotionEventStatus st;
    c.latest(st);
    EXPECT_EQ(st.activity_type, ActivityType::Walking);
    EXPECT_EQ(st.gesture_type, GestureType::None);

    m.fire_gesture(GestureType::WristShake);
    c.update(0.02f);
    c.latest(st);
    EXPECT_EQ(st.gesture_type, GestureType::WristShake);
    // Each update() call starts from a fresh MotionEventStatus{}: a field only
    // reflects what fired in *this* tick, not a stale value from a previous one.
    EXPECT_EQ(st.activity_type, ActivityType::Unknown);
}
