#include <gtest/gtest.h>
#include "imu/motion_events.hpp"

namespace {
using namespace m5::imu;

class FiringSource : public MotionEventSource {
public:
    explicit FiringSource(MotionEventKind spec, MotionEventKind fire) : _spec{spec}, _fire{fire}
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
        _enabled |= (k & _spec);
        return any(k & _spec);
    }
    bool disableEvent(MotionEventKind k) override
    {
        _enabled &= ~k;
        return true;
    }
    bool readEventStatus(MotionEventStatus& out) override
    {
        if (any(_enabled & _fire)) {
            out.fired |= _fire;
            _fire = MotionEventKind::None;
            return true;
        }
        return false;
    }

private:
    MotionEventKind _spec;
    MotionEventKind _fire;
    MotionEventKind _enabled{MotionEventKind::None};
};
}  // namespace

TEST(MotionEventControllerUpdate, update_or_combines_fired_flags)
{
    FiringSource A{MotionEventKind::AnyMotion, MotionEventKind::AnyMotion};
    FiringSource B{MotionEventKind::NoMotion, MotionEventKind::NoMotion};
    MotionEventController c;
    c.add(A);
    c.add(B);
    c.enable(MotionEventKind::AnyMotion, nullptr);
    c.enable(MotionEventKind::NoMotion, nullptr);
    const auto fired = c.update(0.020f);
    EXPECT_EQ(fired, MotionEventKind::AnyMotion | MotionEventKind::NoMotion);
}

TEST(MotionEventControllerUpdate, latest_snapshots_last_update_result)
{
    FiringSource A{MotionEventKind::AnyMotion, MotionEventKind::AnyMotion};
    MotionEventController c;
    c.add(A);
    c.enable(MotionEventKind::AnyMotion, nullptr);
    c.update(0.010f);
    MotionEventStatus st;
    EXPECT_TRUE(c.latest(st));
    EXPECT_EQ(st.fired, MotionEventKind::AnyMotion);
}

TEST(MotionEventControllerUpdate, update_with_no_enabled_returns_none)
{
    MotionEventController c;
    EXPECT_EQ(c.update(0.010f), MotionEventKind::None);
}

TEST(MotionEventControllerUpdate, enabledEvents_ors_all_sources)
{
    FiringSource A{MotionEventKind::AnyMotion, MotionEventKind::None};
    FiringSource B{MotionEventKind::NoMotion, MotionEventKind::None};
    MotionEventController c;
    c.add(A);
    c.add(B);
    c.enable(MotionEventKind::AnyMotion, nullptr);
    EXPECT_EQ(c.enabledEvents(), MotionEventKind::AnyMotion);
    c.enable(MotionEventKind::NoMotion, nullptr);
    EXPECT_EQ(c.enabledEvents(), MotionEventKind::AnyMotion | MotionEventKind::NoMotion);
}
