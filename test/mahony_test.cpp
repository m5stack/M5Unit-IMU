/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <cmath>

#include <gtest/gtest.h>
#include <imu/filter/mahony.hpp>
#include <math/quaternion.hpp>

#include "filter_scenario.hpp"

using namespace m5::imu;
using namespace m5::imu::test;

namespace {
constexpr float TEST_DT{0.01f};
}  // namespace

TEST(MahonyFilterTest, StaticConvergesToLevel)
{
    MahonyFilter filter;
    run_static(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 0.0f, 0.5f);
    EXPECT_NEAR(pitch, 0.0f, 0.5f);
}

TEST(MahonyFilterTest, TiltConvergesToRoll30Degrees)
{
    MahonyFilter filter;
    run_tilt_roll30(filter, 5.0f, TEST_DT);

    float roll{};
    float pitch{};
    float yaw{};
    euler_of(filter, roll, pitch, yaw);

    EXPECT_NEAR(roll, 30.0f, 1.0f);
    EXPECT_NEAR(pitch, 0.0f, 1.0f);
}

TEST(MahonyFilterTest, GyroYawFollowsWithoutMag)
{
    MahonyFilter filter;
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

TEST(MahonyFilterTest, MagCorrectsYawToStableHeading)
{
    MahonyFilter filter;
    // Introduce a yaw offset first (gyro-only, no mag), then let the fixed
    // north-pointing mag reading pull yaw back to a stable heading.
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

TEST(MahonyFilterTest, KpKiReadWriteRoundTrip)
{
    MahonyFilter filter;
    EXPECT_FLOAT_EQ(filter.readKp(), 1.0f);
    EXPECT_FLOAT_EQ(filter.readKi(), 0.0f);

    EXPECT_TRUE(filter.writeKp(2.5f));
    EXPECT_FLOAT_EQ(filter.readKp(), 2.5f);
    EXPECT_TRUE(filter.writeKi(0.2f));
    EXPECT_FLOAT_EQ(filter.readKi(), 0.2f);

    // Out of range: rejected, previous value kept.
    EXPECT_FALSE(filter.writeKp(0.0f));
    EXPECT_FALSE(filter.writeKp(-0.1f));
    EXPECT_FLOAT_EQ(filter.readKp(), 2.5f);

    EXPECT_FALSE(filter.writeKi(-0.01f));
    EXPECT_FLOAT_EQ(filter.readKi(), 0.2f);

    // Zero is a valid Ki (integral action disabled).
    EXPECT_TRUE(filter.writeKi(0.0f));
    EXPECT_FLOAT_EQ(filter.readKi(), 0.0f);
}

TEST(MahonyFilterTest, ResetReturnsToIdentity)
{
    MahonyFilter filter;
    EXPECT_TRUE(filter.writeKi(0.1f));
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

    // The integral term must also be cleared: re-running the same tilt scenario
    // from identity should reproduce the same result as a fresh filter (i.e. no
    // leftover integral windup from the previous run).
    MahonyFilter fresh;
    EXPECT_TRUE(fresh.writeKi(0.1f));
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

TEST(MahonyFilterTest, IntegralGainReducesGyroBiasDrift)
{
    // A constant gyro bias about the roll axis with level (static) accel: pure
    // proportional feedback settles to a nonzero steady-state roll error
    // (bias / Kp), while adding integral feedback should drive that residual
    // error toward zero.
    constexpr float BIAS_DPS{2.0f};
    constexpr float DURATION_S{30.0f};

    const float accel[3]       = {0.0f, 0.0f, 1.0f};
    const float biased_gyro[3] = {BIAS_DPS, 0.0f, 0.0f};
    const int steps            = static_cast<int>(DURATION_S / TEST_DT + 0.5f);

    MahonyFilter without_ki;
    EXPECT_TRUE(without_ki.writeKi(0.0f));
    for (int i = 0; i < steps; ++i) {
        without_ki.update(TEST_DT, accel, biased_gyro, nullptr);
    }

    MahonyFilter with_ki;
    EXPECT_TRUE(with_ki.writeKi(0.1f));
    for (int i = 0; i < steps; ++i) {
        with_ki.update(TEST_DT, accel, biased_gyro, nullptr);
    }

    float roll_without{};
    float roll_with{};
    float pitch{};
    float yaw{};
    euler_of(without_ki, roll_without, pitch, yaw);
    euler_of(with_ki, roll_with, pitch, yaw);

    EXPECT_LT(std::fabs(roll_with), std::fabs(roll_without));
    EXPECT_LT(std::fabs(roll_with), 2.0f);
}
