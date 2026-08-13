#include <gtest/gtest.h>
#include <m5_utility/stl/extension.hpp>
#include "imu/motion_events.hpp"

using m5::imu::any;
using m5::imu::MotionEventKind;
using m5::stl::to_underlying;

TEST(MotionEventKindOps, or_combines_bits)
{
    const auto v = MotionEventKind::AnyMotion | MotionEventKind::NoMotion;
    EXPECT_EQ(to_underlying(v), 0x0003u);
}

TEST(MotionEventKindOps, and_masks_bits)
{
    const auto v = (MotionEventKind::AnyMotion | MotionEventKind::NoMotion) & MotionEventKind::AnyMotion;
    EXPECT_EQ(v, MotionEventKind::AnyMotion);
}

TEST(MotionEventKindOps, or_assign_accumulates)
{
    auto acc = MotionEventKind::None;
    acc |= MotionEventKind::AnyMotion;
    acc |= MotionEventKind::StepCounter;
    EXPECT_EQ(to_underlying(acc), 0x0001u | 0x0008u);
}

TEST(MotionEventKindOps, any_true_when_bit_set)
{
    EXPECT_TRUE(any(MotionEventKind::AnyMotion));
    EXPECT_FALSE(any(MotionEventKind::None));
}
