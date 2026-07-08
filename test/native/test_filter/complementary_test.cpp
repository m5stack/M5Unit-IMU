/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/filter/complementary.hpp>

#include "filter_scenario.hpp"

using namespace m5::imu;
using namespace m5::imu::test;

namespace {
constexpr float TEST_DT{0.01f};
}  // namespace

TEST(ComplementaryFilterTest, StaticConvergesToLevel)
{
    ComplementaryFilter filter;
    run_static(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 0.0f, 0.5f);
    EXPECT_NEAR(pitch, 0.0f, 0.5f);
}

TEST(ComplementaryFilterTest, TiltConvergesToRoll30Degrees)
{
    ComplementaryFilter filter;
    run_tilt_roll30(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 30.0f, 1.0f);
    EXPECT_NEAR(pitch, 0.0f, 1.0f);
}

TEST(ComplementaryFilterTest, GyroYawFollowsWithoutMag)
{
    ComplementaryFilter filter;
    run_static(filter, 1.0f, TEST_DT);
    run_yaw_rotation(filter, 90.0f, 1.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(yaw, 90.0f, 3.0f);
    EXPECT_NEAR(roll, 0.0f, 1.0f);
    EXPECT_NEAR(pitch, 0.0f, 1.0f);
}

TEST(ComplementaryFilterTest, MagCorrectsYawToStableHeading)
{
    ComplementaryFilter filter;
    // Introduce a yaw offset first (gyro-only, no mag), then let the fixed
    // north-pointing mag reading pull yaw back to a stable heading.
    run_yaw_rotation(filter, 45.0f, 1.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw_before_mag{};
    euler_of(filter, roll, pitch, yaw_before_mag);
    EXPECT_NEAR(yaw_before_mag, 45.0f, 3.0f);  // Sanity check on the induced offset

    run_with_mag(filter, 5.0f, TEST_DT);

    float yaw_converged{};
    euler_of(filter, roll, pitch, yaw_converged);
    EXPECT_NEAR(yaw_converged, 0.0f, 1.0f);

    // Confirm it has actually settled (further updates barely move it).
    run_with_mag(filter, 1.0f, TEST_DT);
    float yaw_after_more{};
    euler_of(filter, roll, pitch, yaw_after_more);
    EXPECT_NEAR(yaw_after_more, yaw_converged, 0.1f);
}

TEST(ComplementaryFilterTest, AlphaReadWriteRoundTrip)
{
    ComplementaryFilter filter;
    EXPECT_FLOAT_EQ(filter.readAlpha(), 0.98f);

    EXPECT_TRUE(filter.writeAlpha(0.5f));
    EXPECT_FLOAT_EQ(filter.readAlpha(), 0.5f);

    // Out of range: rejected, previous value kept.
    EXPECT_FALSE(filter.writeAlpha(0.0f));
    EXPECT_FALSE(filter.writeAlpha(1.0f));
    EXPECT_FALSE(filter.writeAlpha(-0.1f));
    EXPECT_FALSE(filter.writeAlpha(1.1f));
    EXPECT_FLOAT_EQ(filter.readAlpha(), 0.5f);
}

TEST(ComplementaryFilterTest, ResetReturnsToIdentity)
{
    ComplementaryFilter filter;
    run_tilt_roll30(filter, 5.0f, TEST_DT);
    filter.reset();

    float w{};
    float x{};
    float y{};
    float z{};
    filter.quaternion(w, x, y, z);
    EXPECT_NEAR(w, 1.0f, 1e-6f);
    EXPECT_NEAR(x, 0.0f, 1e-6f);
    EXPECT_NEAR(y, 0.0f, 1e-6f);
    EXPECT_NEAR(z, 0.0f, 1e-6f);
}
