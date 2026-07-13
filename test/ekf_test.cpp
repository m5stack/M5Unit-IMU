/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <cmath>

#include <gtest/gtest.h>
#include <imu/filter/ekf.hpp>
#include <math/quaternion.hpp>

#include "filter_scenario.hpp"

using namespace m5::imu;
using namespace m5::imu::test;

namespace {
constexpr float TEST_DT{0.01f};
}  // namespace

TEST(EKFFilterTest, StaticConvergesToLevel)
{
    EKFFilter filter;
    run_static(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 0.0f, 0.5f);
    EXPECT_NEAR(pitch, 0.0f, 0.5f);
}

TEST(EKFFilterTest, TiltConvergesToRoll30Degrees)
{
    EKFFilter filter;
    run_tilt_roll30(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 30.0f, 1.0f);
    EXPECT_NEAR(pitch, 0.0f, 1.0f);
}

TEST(EKFFilterTest, GyroYawFollowsWithoutMag)
{
    EKFFilter filter;
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

TEST(EKFFilterTest, MagCorrectsYawToStableHeading)
{
    EKFFilter filter;
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
    EXPECT_NEAR(yaw_after_more, yaw_converged, 0.2f);
}

TEST(EKFFilterTest, NoiseParamsReadWriteRoundTrip)
{
    EKFFilter filter;
    EXPECT_FLOAT_EQ(filter.readGyroNoise(), 0.5f);
    EXPECT_FLOAT_EQ(filter.readAccelNoise(), 0.05f);
    EXPECT_FLOAT_EQ(filter.readMagNoise(), 2.0f);
    EXPECT_FLOAT_EQ(filter.readBiasRandomWalk(), 0.01f);

    EXPECT_TRUE(filter.writeNoiseParams(1.0f, 0.1f, 5.0f, 0.02f));
    EXPECT_FLOAT_EQ(filter.readGyroNoise(), 1.0f);
    EXPECT_FLOAT_EQ(filter.readAccelNoise(), 0.1f);
    EXPECT_FLOAT_EQ(filter.readMagNoise(), 5.0f);
    EXPECT_FLOAT_EQ(filter.readBiasRandomWalk(), 0.02f);

    // Out of range: rejected, previous values kept.
    EXPECT_FALSE(filter.writeNoiseParams(0.0f, 0.1f, 5.0f, 0.02f));
    EXPECT_FALSE(filter.writeNoiseParams(1.0f, -0.1f, 5.0f, 0.02f));
    EXPECT_FALSE(filter.writeNoiseParams(1.0f, 0.1f, 0.0f, 0.02f));
    EXPECT_FALSE(filter.writeNoiseParams(1.0f, 0.1f, 5.0f, -0.02f));
    EXPECT_FLOAT_EQ(filter.readGyroNoise(), 1.0f);
    EXPECT_FLOAT_EQ(filter.readAccelNoise(), 0.1f);
    EXPECT_FLOAT_EQ(filter.readMagNoise(), 5.0f);
    EXPECT_FLOAT_EQ(filter.readBiasRandomWalk(), 0.02f);
}

TEST(EKFFilterTest, WriteQuaternionSeamOverridesAttitudeOnly)
{
    EKFFilter filter;
    const m5::math::Quaternion q_offset = m5::math::Quaternion::fromEuler(40.0f, 0.0f, 0.0f);
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

    // Rejected: near-zero quaternion, previous attitude kept.
    EXPECT_FALSE(filter.writeQuaternion(0.0f, 0.0f, 0.0f, 0.0f));
    filter.quaternion(w, x, y, z);
    EXPECT_NEAR(w, q_offset.w, 1e-6f);
}

TEST(EKFFilterTest, ResetReturnsToIdentityAndClearsBias)
{
    EKFFilter filter;
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

    const m5::math::Vector3 bias = filter.readGyroBias();
    EXPECT_NEAR(bias.x, 0.0f, 1e-6f);
    EXPECT_NEAR(bias.y, 0.0f, 1e-6f);
    EXPECT_NEAR(bias.z, 0.0f, 1e-6f);

    // Covariance must also be reset: re-running the same tilt scenario from
    // identity should reproduce the same result as a fresh filter (i.e. no
    // leftover covariance shrink from the previous run).
    EKFFilter fresh;
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

TEST(EKFFilterTest, GyroBiasEstimationConvergesWhileStationary)
{
    // A constant gyro bias about the roll axis with level (static) accel: the
    // EKF should estimate the bias (via the 7-state augmentation) and keep the
    // attitude estimate accurate despite the biased gyro input.
    constexpr float BIAS_DPS{2.0f};
    constexpr float DURATION_S{30.0f};

    EKFFilter filter;
    const float accel[3]       = {0.0f, 0.0f, 1.0f};
    const float biased_gyro[3] = {BIAS_DPS, 0.0f, 0.0f};
    const int steps            = static_cast<int>(DURATION_S / TEST_DT + 0.5f);
    for (int i = 0; i < steps; ++i) {
        filter.update(TEST_DT, accel, biased_gyro, nullptr);
    }

    const m5::math::Vector3 bias = filter.readGyroBias();
    EXPECT_NEAR(bias.x, BIAS_DPS, 0.5f);
    EXPECT_NEAR(bias.y, 0.0f, 0.5f);
    EXPECT_NEAR(bias.z, 0.0f, 0.5f);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1.0f);
}
