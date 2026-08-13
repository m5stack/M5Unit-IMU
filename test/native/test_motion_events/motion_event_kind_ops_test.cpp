#include <gtest/gtest.h>
#include "imu/motion_events.hpp"

using m5::imu::any;
using m5::imu::MotionEventKind;

TEST(MotionEventKindOps, or_combines_bits)
{
    const auto v = MotionEventKind::AnyMotion | MotionEventKind::NoMotion;
    EXPECT_EQ(static_cast<uint32_t>(v), 0x0003);
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
    EXPECT_EQ(static_cast<uint32_t>(acc), 0x0001 | 0x0008);
}

TEST(MotionEventKindOps, any_true_when_bit_set)
{
    EXPECT_TRUE(any(MotionEventKind::AnyMotion));
    EXPECT_FALSE(any(MotionEventKind::None));
}
