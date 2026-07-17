/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/axis_order.hpp>

using namespace m5::imu;

namespace {
void expect_vec(const float v[3], const float x, const float y, const float z)
{
    EXPECT_FLOAT_EQ(v[0], x);
    EXPECT_FLOAT_EQ(v[1], y);
    EXPECT_FLOAT_EQ(v[2], z);
}
}  // namespace

TEST(AxisOrder, IdentityIsNoOp)
{
    AxisOrder order;  // default: XYZ, no inversion
    float v[3] = {1.0f, 2.0f, 3.0f};
    order.apply(v);
    expect_vec(v, 1.0f, 2.0f, 3.0f);
}

TEST(AxisOrder, XZYSwapsYAndZ)
{
    AxisOrder order;
    order.permutation = AxisPermutation::XZY;
    float v[3]        = {1.0f, 2.0f, 3.0f};
    order.apply(v);
    expect_vec(v, 1.0f, 3.0f, 2.0f);
}

TEST(AxisOrder, YXZSwapsXAndY)
{
    AxisOrder order;
    order.permutation = AxisPermutation::YXZ;
    float v[3]        = {1.0f, 2.0f, 3.0f};
    order.apply(v);
    expect_vec(v, 2.0f, 1.0f, 3.0f);
}

TEST(AxisOrder, YZXRotatesLeft)
{
    AxisOrder order;
    order.permutation = AxisPermutation::YZX;
    float v[3]        = {1.0f, 2.0f, 3.0f};
    order.apply(v);
    expect_vec(v, 2.0f, 3.0f, 1.0f);
}

TEST(AxisOrder, ZXYRotatesRight)
{
    AxisOrder order;
    order.permutation = AxisPermutation::ZXY;
    float v[3]        = {1.0f, 2.0f, 3.0f};
    order.apply(v);
    expect_vec(v, 3.0f, 1.0f, 2.0f);
}

TEST(AxisOrder, ZYXReversesAxes)
{
    AxisOrder order;
    order.permutation = AxisPermutation::ZYX;
    float v[3]        = {1.0f, 2.0f, 3.0f};
    order.apply(v);
    expect_vec(v, 3.0f, 2.0f, 1.0f);
}

TEST(AxisOrder, AllAxesInvertedWithIdentityPermutation)
{
    AxisOrder order;
    order.invert_x = order.invert_y = order.invert_z = true;
    float v[3]                                       = {1.0f, 2.0f, 3.0f};
    order.apply(v);
    expect_vec(v, -1.0f, -2.0f, -3.0f);
}

TEST(AxisOrder, PermutationAndInversionCombine)
{
    // AtomS3R-style remap: order_yxz | invert_y (M5Unified's correction for this board)
    AxisOrder order;
    order.permutation = AxisPermutation::YXZ;
    order.invert_y    = true;
    float v[3]        = {1.0f, 2.0f, 3.0f};
    order.apply(v);
    // YXZ: (2,1,3), then invert output Y (index 1): (2,-1,3)
    expect_vec(v, 2.0f, -1.0f, 3.0f);
}
