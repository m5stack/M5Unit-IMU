#include <gtest/gtest.h>
#include "../../../examples/UnitUnified/MotionEvents/main/software_free_fall.hpp"

using example::FreeFallBit;
using example::SoftwareFreeFall;
using m5::imu::imu_physical_data_t;
using m5::imu::MotionEventStatus;

TEST(SoftwareFreeFall, fires_after_duration_of_low_g)
{
    SoftwareFreeFall ff{SoftwareFreeFall::Config{0.3f, 100}};
    ASSERT_TRUE(ff.enableEvent(FreeFallBit, nullptr));

    imu_physical_data_t s{};
    s.accel[0] = 0.05f;
    s.accel[1] = 0.05f;
    s.accel[2] = 0.05f;  // |a| ~= 0.087G, below 0.3G threshold
    // 10 ticks x 10 ms = 100 ms consecutive low-G
    for (int i = 0; i < 10; ++i) ff.update(0.010f, s);

    MotionEventStatus out{};
    EXPECT_TRUE(ff.readEventStatus(out));
    EXPECT_EQ(out.fired, FreeFallBit);
}

TEST(SoftwareFreeFall, does_not_fire_when_1G_static)
{
    SoftwareFreeFall ff;
    ff.enableEvent(FreeFallBit, nullptr);

    imu_physical_data_t s{};
    s.accel[0] = 0.0f;
    s.accel[1] = 0.0f;
    s.accel[2] = 1.0f;  // |a| = 1G
    for (int i = 0; i < 20; ++i) ff.update(0.010f, s);

    MotionEventStatus out{};
    EXPECT_FALSE(ff.readEventStatus(out));
}

TEST(SoftwareFreeFall, motion_interrupt_resets_still_timer)
{
    SoftwareFreeFall ff{SoftwareFreeFall::Config{0.3f, 100}};
    ff.enableEvent(FreeFallBit, nullptr);
    imu_physical_data_t low{};
    imu_physical_data_t high{};
    low.accel[2]  = 0.1f;
    high.accel[2] = 1.0f;

    // 50 ms low-G, then 1 tick high-G resets the timer, then 50 ms low-G again.
    for (int i = 0; i < 5; ++i) ff.update(0.010f, low);
    ff.update(0.010f, high);
    for (int i = 0; i < 5; ++i) ff.update(0.010f, low);

    MotionEventStatus out{};
    EXPECT_FALSE(ff.readEventStatus(out));  // Never reaches 100 ms cumulative
}

TEST(SoftwareFreeFall, disabled_ignores_input)
{
    SoftwareFreeFall ff;
    imu_physical_data_t low{};
    low.accel[2] = 0.1f;
    for (int i = 0; i < 30; ++i) ff.update(0.010f, low);
    MotionEventStatus out{};
    EXPECT_FALSE(ff.readEventStatus(out));
}
