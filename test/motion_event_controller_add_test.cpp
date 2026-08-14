#include <gtest/gtest.h>
#include "imu/motion_events.hpp"

namespace {
using namespace m5::imu;

class MockSource : public MotionEventSource {
public:
    explicit MockSource(MotionEventKind spec) : _spec{spec}
    {
    }
    MotionEventKind motionEventSpec() const override
    {
        return _spec;
    }
    MotionEventKind enabledEvents() const override
    {
        return _enabled;
    }
    bool enableEvent(MotionEventKind k, const void*) override
    {
        if (!any(_spec & k)) return false;
        _enabled |= k;
        return true;
    }
    bool disableEvent(MotionEventKind k) override
    {
        _enabled &= ~k;
        return true;
    }
    bool readEventStatus(MotionEventStatus&) override
    {
        return false;
    }

private:
    MotionEventKind _spec;
    MotionEventKind _enabled{MotionEventKind::None};
};
}  // namespace

TEST(MotionEventController, single_source_supported_reflects_spec)
{
    MockSource s1{MotionEventKind::AnyMotion | MotionEventKind::NoMotion};
    MotionEventController c;
    EXPECT_TRUE(c.add(s1));
    EXPECT_EQ(c.supported(), MotionEventKind::AnyMotion | MotionEventKind::NoMotion);
}

TEST(MotionEventController, first_in_wins_claim_ordering)
{
    // A claims AnyMotion|NoMotion first; B requests NoMotion|StepCounter,
    // but NoMotion is already claimed by A so B only owns StepCounter.
    MockSource A{MotionEventKind::AnyMotion | MotionEventKind::NoMotion};
    MockSource B{MotionEventKind::NoMotion | MotionEventKind::StepCounter};
    MotionEventController c;
    EXPECT_TRUE(c.add(A));
    EXPECT_TRUE(c.add(B));
    EXPECT_EQ(c.supported(), MotionEventKind::AnyMotion | MotionEventKind::NoMotion | MotionEventKind::StepCounter);
    EXPECT_TRUE(c.enable(MotionEventKind::AnyMotion, nullptr));
    EXPECT_EQ(A.enabledEvents(), MotionEventKind::AnyMotion);
    EXPECT_EQ(B.enabledEvents(), MotionEventKind::None);
    EXPECT_TRUE(c.enable(MotionEventKind::NoMotion, nullptr));
    EXPECT_TRUE(any(A.enabledEvents() & MotionEventKind::NoMotion));
    EXPECT_FALSE(any(B.enabledEvents() & MotionEventKind::NoMotion));
    EXPECT_TRUE(c.enable(MotionEventKind::StepCounter, nullptr));
    EXPECT_TRUE(any(B.enabledEvents() & MotionEventKind::StepCounter));
}

TEST(MotionEventController, has_reflects_supported)
{
    MockSource A{MotionEventKind::AnyMotion};
    MotionEventController c;
    c.add(A);
    EXPECT_TRUE(c.has(MotionEventKind::AnyMotion));
    EXPECT_FALSE(c.has(MotionEventKind::NoMotion));
}
