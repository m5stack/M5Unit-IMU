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

namespace {

const AxisPermutation ALL_PERMUTATIONS[6] = {
    AxisPermutation::XYZ, AxisPermutation::XZY, AxisPermutation::YXZ,
    AxisPermutation::YZX, AxisPermutation::ZXY, AxisPermutation::ZYX,
};

//! @brief Build an AxisOrder from a permutation and a 3-bit sign mask (bit0=X, bit1=Y, bit2=Z)
AxisOrder make_order(const AxisPermutation permutation, const int sign_mask)
{
    AxisOrder order;
    order.permutation = permutation;
    order.invert_x    = (sign_mask & 1) != 0;
    order.invert_y    = (sign_mask & 2) != 0;
    order.invert_z    = (sign_mask & 4) != 0;
    return order;
}

//! @brief Expect two axis orders to transform every probe vector identically
void expect_same_transform(const AxisOrder& lhs, const AxisOrder& rhs)
{
    const float probes[4][3] = {
        {1.0f, 2.0f, 3.0f}, {-4.0f, 5.0f, -6.0f}, {7.0f, -8.0f, 9.0f}, {0.5f, -0.5f, 2.5f},
    };
    for (const auto& probe : probes) {
        float a[3] = {probe[0], probe[1], probe[2]};
        float b[3] = {probe[0], probe[1], probe[2]};
        lhs.apply(a);
        rhs.apply(b);
        expect_vec(a, b[0], b[1], b[2]);
    }
}

}  // namespace

TEST(AxisOrder, PermutationIndicesRoundTrip)
{
    for (const auto permutation : ALL_PERMUTATIONS) {
        const uint8_t* indices = axis_permutation_indices(permutation);
        const uint8_t copy[3]  = {indices[0], indices[1], indices[2]};
        EXPECT_EQ(permutation_from_indices(copy), permutation);
    }
}

TEST(AxisOrder, ComposeWithIdentityIsUnchanged)
{
    const AxisOrder identity;
    for (const auto permutation : ALL_PERMUTATIONS) {
        for (int sign_mask = 0; sign_mask < 8; ++sign_mask) {
            const AxisOrder order = make_order(permutation, sign_mask);
            expect_same_transform(compose(order, identity), order);
            expect_same_transform(compose(identity, order), order);
        }
    }
}

TEST(AxisOrder, ComposeMatchesSequentialApply)
{
    // Exhaustive: the composed order must equal applying `first` then `second`, for every
    // (permutation, sign) pair on both sides (6 * 8 = 48 orders each, 2304 combinations).
    for (const auto first_permutation : ALL_PERMUTATIONS) {
        for (int first_signs = 0; first_signs < 8; ++first_signs) {
            const AxisOrder first = make_order(first_permutation, first_signs);
            for (const auto second_permutation : ALL_PERMUTATIONS) {
                for (int second_signs = 0; second_signs < 8; ++second_signs) {
                    const AxisOrder second   = make_order(second_permutation, second_signs);
                    const AxisOrder composed = compose(first, second);

                    float sequential[3] = {1.0f, -2.0f, 3.0f};
                    first.apply(sequential);
                    second.apply(sequential);

                    float folded[3] = {1.0f, -2.0f, 3.0f};
                    composed.apply(folded);

                    expect_vec(folded, sequential[0], sequential[1], sequential[2]);
                }
            }
        }
    }
}

TEST(AxisOrder, ComposeIsOrderDependent)
{
    // Signed permutations do not commute in general; guards against composing the user order
    // and the fixed board order in the wrong direction.
    AxisOrder first;
    first.permutation = AxisPermutation::YXZ;
    AxisOrder second;
    second.permutation = AxisPermutation::YZX;

    float forward[3] = {1.0f, 2.0f, 3.0f};
    compose(first, second).apply(forward);
    expect_vec(forward, 1.0f, 3.0f, 2.0f);  // YXZ then YZX

    float reverse[3] = {1.0f, 2.0f, 3.0f};
    compose(second, first).apply(reverse);
    expect_vec(reverse, 3.0f, 2.0f, 1.0f);  // YZX then YXZ
}

TEST(AxisOrder, ComposeFoldsBoardCorrectionWithUserOrder)
{
    // CoreS3-style fixed mag correction (invert Y and Z) folded with a user X/Y swap.
    AxisOrder fixed;
    fixed.invert_y = true;
    fixed.invert_z = true;
    AxisOrder user;
    user.permutation = AxisPermutation::YXZ;

    float v[3] = {1.0f, 2.0f, 3.0f};
    compose(fixed, user).apply(v);
    // fixed: (1,-2,-3), then user swaps X and Y: (-2,1,-3)
    expect_vec(v, -2.0f, 1.0f, -3.0f);
}
