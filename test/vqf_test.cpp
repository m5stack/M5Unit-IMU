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

// VQF's magnetometer heading reference converges the horizontal mag component onto
// world +Y (see VQF::updateMag()'s atan2(magEarth[0], magEarth[1])), a fixed +90 degree
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
    // Defaults, per VQFParams::VQFParams().
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
    // result as a brand-new filter (i.e. reset() must also clear VQF's internal
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
// the underlying VQF instance -- this must not crash, and the filter must keep
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

// ---------------------------------------------------------------------------------------
// Full-VQF features: bias estimation and magnetic disturbance rejection
// ---------------------------------------------------------------------------------------

namespace {

// Stationary body whose gyroscope carries a constant bias about Z: the accelerometer still
// reads gravity, so only the heading is corrupted -- exactly the 6-axis yaw drift case.
void run_static_with_gyro_bias(AhrsFilterBase& filter, const float bias_dps, const float seconds, const float dt)
{
    const float accel[3] = {0.0f, 0.0f, 1.0f};
    const float gyro[3]  = {0.0f, 0.0f, bias_dps};
    const int steps      = scenario_steps(seconds, dt);
    for (int i = 0; i < steps; ++i) {
        filter.update(dt, accel, gyro, nullptr);
    }
}

}  // namespace

TEST(VQFFilterTest, FeatureFlagsDefaultOnAndRoundTrip)
{
    VQFFilter filter;
    // Upstream's defaults; turning all three off makes this equivalent to the "basic" variant
    EXPECT_TRUE(filter.readRestBiasEstimation());
    EXPECT_TRUE(filter.readMotionBiasEstimation());
    EXPECT_TRUE(filter.readMagDisturbanceRejection());

    filter.writeRestBiasEstimation(false);
    filter.writeMotionBiasEstimation(false);
    filter.writeMagDisturbanceRejection(false);
    EXPECT_FALSE(filter.readRestBiasEstimation());
    EXPECT_FALSE(filter.readMotionBiasEstimation());
    EXPECT_FALSE(filter.readMagDisturbanceRejection());

    filter.writeRestBiasEstimation(true);
    EXPECT_TRUE(filter.readRestBiasEstimation());
}

TEST(VQFFilterTest, RestBiasEstimationSuppressesYawDrift)
{
    constexpr float dt{0.01f};
    constexpr float bias_dps{1.0f};  // well inside VQFParams::biasClip (2 dps)
    constexpr float seconds{30.0f};

    // Bias estimation off == the old "basic" behaviour: the bias integrates into the heading
    VQFFilter basic;
    basic.writeRestBiasEstimation(false);
    basic.writeMotionBiasEstimation(false);
    run_static_with_gyro_bias(basic, bias_dps, seconds, dt);
    float roll{}, pitch{}, yaw_basic{};
    euler_of(basic, roll, pitch, yaw_basic);

    // Defaults (rest bias estimation on): the bias is measured while resting and removed
    VQFFilter full;
    run_static_with_gyro_bias(full, bias_dps, seconds, dt);
    float yaw_full{};
    euler_of(full, roll, pitch, yaw_full);

    // 1 dps for 30 s is ~30 degrees of uncorrected heading error...
    EXPECT_GT(std::fabs(yaw_basic), 20.0f);
    // ...and estimating the bias must cut that down substantially
    EXPECT_LT(std::fabs(yaw_full), std::fabs(yaw_basic) * 0.5f);
}

TEST(VQFFilterTest, RestBiasEstimateConvergesToTheInjectedBias)
{
    constexpr float dt{0.01f};
    constexpr float bias_dps{1.0f};

    VQFFilter filter;
    // Longer than VQFParams::restMinT (1.5 s) so rest detection has time to settle
    run_static_with_gyro_bias(filter, bias_dps, 30.0f, dt);

    EXPECT_TRUE(filter.restDetected());
    const m5::math::Vector3 bias = filter.readGyroBias();
    EXPECT_NEAR(bias.z, bias_dps, 0.3f);  // reported in dps, like EKFFilter::readGyroBias()
    EXPECT_NEAR(bias.x, 0.0f, 0.3f);
    EXPECT_NEAR(bias.y, 0.0f, 0.3f);
}

TEST(VQFFilterTest, NoBiasIsReportedWhileEstimationIsDisabled)
{
    constexpr float dt{0.01f};

    VQFFilter filter;
    filter.writeRestBiasEstimation(false);
    filter.writeMotionBiasEstimation(false);
    run_static_with_gyro_bias(filter, 1.0f, 30.0f, dt);

    const m5::math::Vector3 bias = filter.readGyroBias();
    EXPECT_NEAR(bias.x, 0.0f, 1e-3f);
    EXPECT_NEAR(bias.y, 0.0f, 1e-3f);
    EXPECT_NEAR(bias.z, 0.0f, 1e-3f);
}

TEST(VQFFilterTest, StationaryDeviceCannotEstablishAMagneticReference)
{
    constexpr float dt{0.01f};

    VQFFilter filter;
    EXPECT_TRUE(filter.readMagDisturbanceRejection());
    // VQF only accepts a magnetic reference once the field has been observed from several
    // orientations (VQFParams::magNewMinGyr, 20 deg/s) -- proof that it is a homogeneous
    // earth field rather than a local distortion. A device that never moves therefore keeps
    // reporting "disturbed" no matter how long it runs.
    run_with_mag(filter, 20.0f, dt);
    EXPECT_TRUE(filter.magDisturbanceDetected());
}

TEST(VQFFilterTest, RotatingInAConsistentFieldEstablishesTheMagneticReference)
{
    constexpr float dt{0.01f};
    constexpr float dps{30.0f};  // above VQFParams::magNewMinGyr (20 deg/s)

    VQFFilter filter;
    // Level body yawing at a constant rate through a fixed world field: the body-frame
    // reading rotates with it, so norm and dip angle stay constant (an undisturbed field)
    const float accel[3] = {0.0f, 0.0f, 1.0f};
    const float gyro[3]  = {0.0f, 0.0f, dps};
    const int steps      = scenario_steps(10.0f, dt);  // > VQFParams::magNewFirstTime (5 s)
    for (int i = 0; i < steps; ++i) {
        const float yaw_rad = dps * SCENARIO_DEG2RAD * dt * static_cast<float>(i);
        const float mag[3]  = {20.0f * std::cos(yaw_rad), -20.0f * std::sin(yaw_rad), -40.0f};
        filter.update(dt, accel, gyro, mag);
    }
    EXPECT_FALSE(filter.magDisturbanceDetected());
}

TEST(VQFFilterTest, DisturbanceFlagIsOnlyMeaningfulWhileRejectionIsEnabled)
{
    constexpr float dt{0.01f};

    VQFFilter filter;
    filter.writeMagDisturbanceRejection(false);
    run_with_mag(filter, 20.0f, dt);
    // With rejection off the detector never runs, so the flag keeps whatever value reset()
    // left it at (true) instead of tracking the field. It must not be read as "a disturbance
    // is present" in this mode -- the heading correction is applied unconditionally, which is
    // the behaviour of upstream's "basic" variant.
    EXPECT_TRUE(filter.magDisturbanceDetected());
}

TEST(VQFFilterTest, BiasClipRoundTrip)
{
    VQFFilter filter;
    EXPECT_FLOAT_EQ(filter.readBiasClip(), 2.0f);  // upstream's default

    EXPECT_TRUE(filter.writeBiasClip(6.0f));
    EXPECT_FLOAT_EQ(filter.readBiasClip(), 6.0f);
    // The other tuning parameters survive the rebuild writeBiasClip() has to perform
    EXPECT_TRUE(filter.readRestBiasEstimation());

    EXPECT_FALSE(filter.writeBiasClip(0.0f));
    EXPECT_FALSE(filter.writeBiasClip(-1.0f));
    EXPECT_FLOAT_EQ(filter.readBiasClip(), 6.0f);
}

TEST(VQFFilterTest, BiasLargerThanTheClipSuppressesRestDetectionEntirely)
{
    constexpr float dt{0.01f};
    // Above the 2 dps default; an MPU6886's zero-rate output can reach this
    constexpr float bias_dps{3.5f};
    constexpr float seconds{30.0f};

    VQFFilter filter;
    run_static_with_gyro_bias(filter, bias_dps, seconds, dt);

    // Upstream refuses to call a sample "at rest" while any low-passed gyro axis exceeds
    // biasClip, so rest detection never fires and no bias is ever estimated...
    EXPECT_FALSE(filter.restDetected());
    const m5::math::Vector3 bias = filter.readGyroBias();
    EXPECT_NEAR(bias.z, 0.0f, 0.1f);

    // ...leaving the full bias to integrate into the heading (3.5 dps * 30 s, wrapped)
    float roll{}, pitch{}, yaw{};
    euler_of(filter, roll, pitch, yaw);
    EXPECT_GT(std::fabs(yaw), 20.0f);
}

TEST(VQFFilterTest, RaisingTheBiasClipRestoresEstimationForALargeBias)
{
    constexpr float dt{0.01f};
    constexpr float bias_dps{3.5f};
    constexpr float seconds{30.0f};

    VQFFilter filter;
    EXPECT_TRUE(filter.writeBiasClip(6.0f));  // now above the sensor's actual bias
    run_static_with_gyro_bias(filter, bias_dps, seconds, dt);

    EXPECT_TRUE(filter.restDetected());
    const m5::math::Vector3 bias = filter.readGyroBias();
    EXPECT_NEAR(bias.z, bias_dps, 0.3f);

    float roll{}, pitch{}, yaw{};
    euler_of(filter, roll, pitch, yaw);
    EXPECT_LT(std::fabs(yaw), 20.0f);  // drift is suppressed again
}
