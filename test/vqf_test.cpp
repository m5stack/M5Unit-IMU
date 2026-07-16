/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <cmath>

#include <gtest/gtest.h>
#include <imu/filter/vqf.hpp>

#include "filter_scenario.hpp"

using namespace m5::imu;
using namespace m5::imu::test;

namespace {
constexpr float TEST_DT{0.01f};
}  // namespace

TEST(VQFFilterTest, StaticConvergesToLevel)
{
    VQFFilter filter;
    run_static(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 0.0f, 0.5f);
    EXPECT_NEAR(pitch, 0.0f, 0.5f);
}

TEST(VQFFilterTest, TiltConvergesToRoll30Degrees)
{
    VQFFilter filter;
    run_tilt_roll30(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 30.0f, 1.0f);
    EXPECT_NEAR(pitch, 0.0f, 1.0f);
}

TEST(VQFFilterTest, GyroYawFollowsWithoutMag)
{
    VQFFilter filter;
    run_yaw_rotation(filter, 90.0f, 1.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(yaw, 90.0f, 3.0f);
    EXPECT_NEAR(roll, 0.0f, 1.0f);
    EXPECT_NEAR(pitch, 0.0f, 1.0f);
}

// basicvqf's magnetometer heading reference converges the horizontal mag component onto
// world +Y (see BasicVQF::updateMag()'s atan2(magEarth[0], magEarth[1])), a fixed +90 degree
// offset from this repository's mag table (filter_base.hpp: mag horizontal along +X at
// yaw = 0). VQFFilter::quaternion() corrects this with a fixed world-Z -90 degree rotation
// once mag samples flow (see vqf.hpp), so -- like the other filters' MagCorrectsYaw tests --
// the absolute converged yaw must be 0, and it must be the same stable attractor regardless
// of where gyro drift/integration left the heading beforehand.
TEST(VQFFilterTest, MagCorrectsYawToStableHeadingRegardlessOfInitialOffset)
{
    VQFFilter filter_a;
    run_yaw_rotation(filter_a, 45.0f, 1.0f, TEST_DT);
    run_with_mag(filter_a, 60.0f, TEST_DT);

    VQFFilter filter_b;
    run_yaw_rotation(filter_b, -30.0f, 1.0f, TEST_DT);
    run_with_mag(filter_b, 60.0f, TEST_DT);

    float roll_a{};
    float pitch_a{};
    float yaw_a{};
    float roll_b{};
    float pitch_b{};
    float yaw_b{};
    euler_of(filter_a, roll_a, pitch_a, yaw_a);
    euler_of(filter_b, roll_b, pitch_b, yaw_b);

    EXPECT_NEAR(yaw_a, 0.0f, 1.0f);  // Same shared mag convention as the other five filters
    EXPECT_NEAR(yaw_b, 0.0f, 1.0f);
    EXPECT_NEAR(yaw_a, yaw_b, 1.0f);
    EXPECT_NEAR(roll_a, 0.0f, 1.0f);
    EXPECT_NEAR(pitch_a, 0.0f, 1.0f);
    EXPECT_NEAR(roll_b, 0.0f, 1.0f);
    EXPECT_NEAR(pitch_b, 0.0f, 1.0f);

    // Confirm it has actually settled (further updates barely move it).
    run_with_mag(filter_a, 5.0f, TEST_DT);
    float yaw_after_more{};
    float roll_after{};
    float pitch_after{};
    euler_of(filter_a, roll_after, pitch_after, yaw_after_more);
    EXPECT_NEAR(yaw_after_more, yaw_a, 0.2f);
}

TEST(VQFFilterTest, TauReadWriteRoundTrip)
{
    VQFFilter filter;
    // Defaults, per basicvqf's BasicVQFParams::BasicVQFParams().
    EXPECT_FLOAT_EQ(filter.readTauAcc(), 3.0f);
    EXPECT_FLOAT_EQ(filter.readTauMag(), 9.0f);

    EXPECT_TRUE(filter.writeTauAcc(1.0f));
    EXPECT_FLOAT_EQ(filter.readTauAcc(), 1.0f);
    EXPECT_TRUE(filter.writeTauMag(5.0f));
    EXPECT_FLOAT_EQ(filter.readTauMag(), 5.0f);

    // Out of range: rejected, previous value kept.
    EXPECT_FALSE(filter.writeTauAcc(0.0f));
    EXPECT_FLOAT_EQ(filter.readTauAcc(), 1.0f);
    EXPECT_FALSE(filter.writeTauMag(-1.0f));
    EXPECT_FLOAT_EQ(filter.readTauMag(), 5.0f);
}

TEST(VQFFilterTest, ResetReturnsToIdentity)
{
    VQFFilter filter;
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

    // Re-running the same scenario from a freshly reset filter must reproduce the same
    // result as a brand-new filter (i.e. reset() must also clear basicvqf's internal
    // low-pass/heading state, not just the quaternion).
    VQFFilter fresh;
    run_tilt_roll30(fresh, 5.0f, TEST_DT);

    run_tilt_roll30(filter, 5.0f, TEST_DT);

    float w1{}, x1{}, y1{}, z1{};
    float w2{}, x2{}, y2{}, z2{};
    filter.quaternion(w1, x1, y1, z1);
    fresh.quaternion(w2, x2, y2, z2);
    EXPECT_NEAR(w1, w2, 1e-4f);
    EXPECT_NEAR(x1, x2, 1e-4f);
    EXPECT_NEAR(y1, y2, 1e-4f);
    EXPECT_NEAR(z1, z2, 1e-4f);
}

// A dt drifting by more than the ~20% threshold documented in vqf.hpp forces a rebuild of
// the underlying BasicVQF instance -- this must not crash, and the filter must keep
// producing a sane (normalized) quaternion afterwards.
TEST(VQFFilterTest, SurvivesLargeSamplePeriodChange)
{
    VQFFilter filter;
    run_static(filter, 2.0f, TEST_DT);  // Built at the default 0.01s nominal period.
    run_static(filter, 2.0f, 0.02f);    // >20% drift: triggers a rebuild.
    run_static(filter, 2.0f, TEST_DT);  // >20% drift back: triggers another rebuild.

    float w{};
    float x{};
    float y{};
    float z{};
    filter.quaternion(w, x, y, z);
    const float norm = std::sqrt(w * w + x * x + y * y + z * z);
    EXPECT_NEAR(norm, 1.0f, 1e-3f);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 0.5f);
    EXPECT_NEAR(pitch, 0.0f, 0.5f);
}
