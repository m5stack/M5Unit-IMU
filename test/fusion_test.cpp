/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <cmath>

#include <gtest/gtest.h>
#include <imu/filter/fusion.hpp>
#include <math/quaternion.hpp>

#include "filter_scenario.hpp"

using namespace m5::imu;
using namespace m5::imu::test;

namespace {
constexpr float TEST_DT{0.01f};

// Fusion always starts (FusionAhrsInitialise()/FusionAhrsRestart(), i.e. also after
// reset()) with a high gain that ramps down to the configured gain over a fixed
// 3-second window (see fusion.hpp), and forces the yaw heading to 0 on every
// no-magnetometer update() while that window is open. Scenarios that rely on gyro-only
// yaw integration therefore need to run past this warm-up window first.
constexpr float FUSION_STARTUP_WARMUP_S{3.5f};
}  // namespace

TEST(FusionFilterTest, StaticConvergesToLevel)
{
    FusionFilter filter;
    run_static(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 0.0f, 0.5f);
    EXPECT_NEAR(pitch, 0.0f, 0.5f);
}

TEST(FusionFilterTest, TiltConvergesToRoll30Degrees)
{
    FusionFilter filter;
    run_tilt_roll30(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 30.0f, 1.0f);
    EXPECT_NEAR(pitch, 0.0f, 1.0f);
}

TEST(FusionFilterTest, GyroYawFollowsWithoutMag)
{
    FusionFilter filter;
    // Clear the startup warm-up window first (see FUSION_STARTUP_WARMUP_S) so the
    // subsequent no-mag yaw rotation actually accumulates instead of being zeroed.
    run_static(filter, FUSION_STARTUP_WARMUP_S, TEST_DT);
    run_yaw_rotation(filter, 90.0f, 1.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(yaw, 90.0f, 3.0f);
    EXPECT_NEAR(roll, 0.0f, 1.0f);
    EXPECT_NEAR(pitch, 0.0f, 1.0f);
}

TEST(FusionFilterTest, MagCorrectsYawToStableHeading)
{
    FusionFilter filter;
    // Same startup warm-up concern as GyroYawFollowsWithoutMag: induce the yaw offset
    // only after the no-mag heading-zeroing window has closed.
    run_static(filter, FUSION_STARTUP_WARMUP_S, TEST_DT);
    run_yaw_rotation(filter, 45.0f, 1.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw_before_mag{};
    euler_of(filter, roll, pitch, yaw_before_mag);
    EXPECT_NEAR(yaw_before_mag, 45.0f, 3.0f);  // Sanity check on the induced offset

    run_with_mag(filter, 60.0f, TEST_DT);

    float yaw_converged{};
    euler_of(filter, roll, pitch, yaw_converged);
    EXPECT_NEAR(yaw_converged, 0.0f, 1.0f);

    // Confirm it has actually settled (further updates barely move it).
    run_with_mag(filter, 5.0f, TEST_DT);
    float yaw_after_more{};
    euler_of(filter, roll, pitch, yaw_after_more);
    EXPECT_NEAR(yaw_after_more, yaw_converged, 0.1f);
}

TEST(FusionFilterTest, GainReadWriteRoundTrip)
{
    FusionFilter filter;
    EXPECT_FLOAT_EQ(filter.readGain(), 0.5f);

    EXPECT_TRUE(filter.writeGain(1.0f));
    EXPECT_FLOAT_EQ(filter.readGain(), 1.0f);

    // Zero is a valid gain (disables outlier rejection, per Fusion's own semantics).
    EXPECT_TRUE(filter.writeGain(0.0f));
    EXPECT_FLOAT_EQ(filter.readGain(), 0.0f);

    // Out of range: rejected, previous value kept.
    EXPECT_FALSE(filter.writeGain(-0.1f));
    EXPECT_FLOAT_EQ(filter.readGain(), 0.0f);
}

TEST(FusionFilterTest, ResetReturnsToIdentity)
{
    FusionFilter filter;
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
    // result as a brand-new filter (i.e. reset() must also clear Fusion's internal
    // feedback/startup-ramp state, not just the quaternion).
    FusionFilter fresh;
    run_tilt_roll30(fresh, 5.0f, TEST_DT);

    run_tilt_roll30(filter, 5.0f, TEST_DT);

    float w1{}, x1{}, y1{}, z1{};
    float w2{}, x2{}, y2{}, z2{};
    filter.quaternion(w1, x1, y1, z1);
    fresh.quaternion(w2, x2, y2, z2);
    EXPECT_NEAR(w1, w2, 1e-5f);
    EXPECT_NEAR(x1, x2, 1e-5f);
    EXPECT_NEAR(y1, y2, 1e-5f);
    EXPECT_NEAR(z1, z2, 1e-5f);
}

TEST(FusionFilterTest, WriteQuaternionRejectsNearZero)
{
    FusionFilter filter;
    EXPECT_FALSE(filter.writeQuaternion(0.0f, 0.0f, 0.0f, 0.0f));

    const m5::math::Quaternion q_offset = m5::math::Quaternion::fromEuler(20.0f, 0.0f, 0.0f);
    EXPECT_TRUE(filter.writeQuaternion(q_offset.w, q_offset.x, q_offset.y, q_offset.z));

    float w{};
    float x{};
    float y{};
    float z{};
    filter.quaternion(w, x, y, z);
    EXPECT_NEAR(w, q_offset.w, 1e-6f);
    EXPECT_NEAR(x, q_offset.x, 1e-6f);
    EXPECT_NEAR(y, q_offset.y, 1e-6f);
    EXPECT_NEAR(z, q_offset.z, 1e-6f);
}
