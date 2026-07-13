/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <cmath>

#include <gtest/gtest.h>
#include <imu/filter/madgwick.hpp>
#include <math/quaternion.hpp>

#include "filter_scenario.hpp"

using namespace m5::imu;
using namespace m5::imu::test;

namespace {
constexpr float TEST_DT{0.01f};
}  // namespace

TEST(MadgwickFilterTest, StaticConvergesToLevel)
{
    MadgwickFilter filter;
    run_static(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 0.0f, 0.5f);
    EXPECT_NEAR(pitch, 0.0f, 0.5f);
}

TEST(MadgwickFilterTest, TiltConvergesToRoll30Degrees)
{
    MadgwickFilter filter;
    run_tilt_roll30(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 30.0f, 1.0f);
    EXPECT_NEAR(pitch, 0.0f, 1.0f);
}

TEST(MadgwickFilterTest, GyroYawFollowsWithoutMag)
{
    MadgwickFilter filter;
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

TEST(MadgwickFilterTest, MagCorrectsYawToStableHeading)
{
    MadgwickFilter filter;
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

TEST(MadgwickFilterTest, BetaReadWriteRoundTrip)
{
    MadgwickFilter filter;
    EXPECT_FLOAT_EQ(filter.readBeta(), 0.1f);

    EXPECT_TRUE(filter.writeBeta(0.5f));
    EXPECT_FLOAT_EQ(filter.readBeta(), 0.5f);

    // Out of range: rejected, previous value kept.
    EXPECT_FALSE(filter.writeBeta(0.0f));
    EXPECT_FALSE(filter.writeBeta(-0.1f));
    EXPECT_FLOAT_EQ(filter.readBeta(), 0.5f);
}

TEST(MadgwickFilterTest, ResetReturnsToIdentity)
{
    MadgwickFilter filter;
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

TEST(MadgwickFilterTest, HigherBetaConvergesFaster)
{
    // Seed both filters with the same initial roll error (level accel input, but the
    // attitude estimate starts at 40 deg roll) and compare the residual roll error
    // after 1 second of static (level) input: a larger beta should correct faster.
    const m5::math::Quaternion q_offset = m5::math::Quaternion::fromEuler(40.0f, 0.0f, 0.0f);

    MadgwickFilter fast;
    EXPECT_TRUE(fast.writeBeta(0.5f));
    EXPECT_TRUE(fast.writeQuaternion(q_offset.w, q_offset.x, q_offset.y, q_offset.z));

    MadgwickFilter slow;
    EXPECT_TRUE(slow.writeBeta(0.05f));
    EXPECT_TRUE(slow.writeQuaternion(q_offset.w, q_offset.x, q_offset.y, q_offset.z));

    run_static(fast, 1.0f, TEST_DT);
    run_static(slow, 1.0f, TEST_DT);

    float roll_fast{};
    float roll_slow{};
    float pitch{};
    float yaw{};
    euler_of(fast, roll_fast, pitch, yaw);
    euler_of(slow, roll_slow, pitch, yaw);

    EXPECT_LT(std::fabs(roll_fast), std::fabs(roll_slow));
}
