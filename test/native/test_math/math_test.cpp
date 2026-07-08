/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <cmath>
#include <math/quaternion.hpp>
#include <math/vector3.hpp>

using namespace m5::math;

namespace {
constexpr float PI{3.14159265358979323846f};
constexpr float DEG2RAD{PI / 180.0f};
}  // namespace

//
// Vector3
//
TEST(Vector3Test, ConstructAndAccessMembers)
{
    const Vector3 v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v.x, 1.0f);
    EXPECT_FLOAT_EQ(v.y, 2.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);

    const Vector3 zero{};
    EXPECT_FLOAT_EQ(zero.x, 0.0f);
    EXPECT_FLOAT_EQ(zero.y, 0.0f);
    EXPECT_FLOAT_EQ(zero.z, 0.0f);
}

TEST(Vector3Test, AddAndSubtract)
{
    const Vector3 a(1.0f, 2.0f, 3.0f);
    const Vector3 b(4.0f, -1.0f, 2.0f);

    const Vector3 sum = a + b;
    EXPECT_FLOAT_EQ(sum.x, 5.0f);
    EXPECT_FLOAT_EQ(sum.y, 1.0f);
    EXPECT_FLOAT_EQ(sum.z, 5.0f);

    const Vector3 diff = a - b;
    EXPECT_FLOAT_EQ(diff.x, -3.0f);
    EXPECT_FLOAT_EQ(diff.y, 3.0f);
    EXPECT_FLOAT_EQ(diff.z, 1.0f);
}

TEST(Vector3Test, ScalarMultiply)
{
    const Vector3 a(1.0f, -2.0f, 3.0f);
    const Vector3 r = a * 2.0f;
    EXPECT_FLOAT_EQ(r.x, 2.0f);
    EXPECT_FLOAT_EQ(r.y, -4.0f);
    EXPECT_FLOAT_EQ(r.z, 6.0f);
}

TEST(Vector3Test, Dot)
{
    const Vector3 a(1.0f, 2.0f, 3.0f);
    const Vector3 b(4.0f, -5.0f, 6.0f);
    EXPECT_FLOAT_EQ(a.dot(b), 1.0f * 4.0f + 2.0f * -5.0f + 3.0f * 6.0f);
}

TEST(Vector3Test, Cross)
{
    const Vector3 x_axis(1.0f, 0.0f, 0.0f);
    const Vector3 y_axis(0.0f, 1.0f, 0.0f);
    const Vector3 r = x_axis.cross(y_axis);
    EXPECT_FLOAT_EQ(r.x, 0.0f);
    EXPECT_FLOAT_EQ(r.y, 0.0f);
    EXPECT_FLOAT_EQ(r.z, 1.0f);
}

TEST(Vector3Test, Norm)
{
    const Vector3 v(3.0f, 4.0f, 0.0f);
    EXPECT_FLOAT_EQ(v.norm(), 5.0f);
}

TEST(Vector3Test, NormalizeInPlace)
{
    Vector3 v(3.0f, 4.0f, 0.0f);
    const bool ok = v.normalize();
    EXPECT_TRUE(ok);
    EXPECT_NEAR(v.x, 0.6f, 1e-6f);
    EXPECT_NEAR(v.y, 0.8f, 1e-6f);
    EXPECT_NEAR(v.z, 0.0f, 1e-6f);
    EXPECT_NEAR(v.norm(), 1.0f, 1e-6f);
}

TEST(Vector3Test, NormalizeZeroVectorIsSafeNoOp)
{
    Vector3 v(0.0f, 0.0f, 0.0f);
    const bool ok = v.normalize();
    EXPECT_FALSE(ok);
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vector3Test, NormalizedReturnsValueWithoutMutatingSource)
{
    const Vector3 v(3.0f, 4.0f, 0.0f);
    const Vector3 n = v.normalized();
    EXPECT_NEAR(n.x, 0.6f, 1e-6f);
    EXPECT_NEAR(n.y, 0.8f, 1e-6f);
    // Source unchanged
    EXPECT_FLOAT_EQ(v.x, 3.0f);
    EXPECT_FLOAT_EQ(v.y, 4.0f);
}

TEST(Vector3Test, NormalizedZeroVectorReturnsZero)
{
    const Vector3 v(0.0f, 0.0f, 0.0f);
    const Vector3 n = v.normalized();
    EXPECT_FLOAT_EQ(n.x, 0.0f);
    EXPECT_FLOAT_EQ(n.y, 0.0f);
    EXPECT_FLOAT_EQ(n.z, 0.0f);
}

//
// Quaternion
//
TEST(QuaternionTest, DefaultIsIdentity)
{
    const Quaternion q;
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);

    const Quaternion id = Quaternion::identity();
    EXPECT_FLOAT_EQ(id.w, 1.0f);
    EXPECT_FLOAT_EQ(id.x, 0.0f);
    EXPECT_FLOAT_EQ(id.y, 0.0f);
    EXPECT_FLOAT_EQ(id.z, 0.0f);
}

TEST(QuaternionTest, MultiplyTwo90DegXRotationsEquals180DegXRotation)
{
    const Quaternion q90  = Quaternion::fromAxisAngle(Vector3(1.0f, 0.0f, 0.0f), 90.0f * DEG2RAD);
    const Quaternion q180 = Quaternion::fromAxisAngle(Vector3(1.0f, 0.0f, 0.0f), 180.0f * DEG2RAD);

    const Quaternion product = q90 * q90;
    EXPECT_NEAR(product.w, q180.w, 1e-5f);
    EXPECT_NEAR(product.x, q180.x, 1e-5f);
    EXPECT_NEAR(product.y, q180.y, 1e-5f);
    EXPECT_NEAR(product.z, q180.z, 1e-5f);
}

TEST(QuaternionTest, Conjugate)
{
    const Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    const Quaternion c = q.conjugated();
    EXPECT_FLOAT_EQ(c.w, 0.5f);
    EXPECT_FLOAT_EQ(c.x, -0.5f);
    EXPECT_FLOAT_EQ(c.y, -0.5f);
    EXPECT_FLOAT_EQ(c.z, -0.5f);
}

TEST(QuaternionTest, Normalize)
{
    Quaternion q(2.0f, 0.0f, 0.0f, 0.0f);
    const bool ok = q.normalize();
    EXPECT_TRUE(ok);
    EXPECT_NEAR(q.w, 1.0f, 1e-6f);
    EXPECT_NEAR(q.x, 0.0f, 1e-6f);

    const Quaternion normalized = Quaternion(0.0f, 3.0f, 4.0f, 0.0f).normalized();
    EXPECT_NEAR(normalized.norm(), 1.0f, 1e-6f);
}

TEST(QuaternionTest, RotateVectorAroundXBy90Degrees)
{
    const Quaternion q = Quaternion::fromAxisAngle(Vector3(1.0f, 0.0f, 0.0f), 90.0f * DEG2RAD);
    const Vector3 r    = q.rotate(Vector3(0.0f, 0.0f, 1.0f));
    EXPECT_NEAR(r.x, 0.0f, 1e-5f);
    EXPECT_NEAR(r.y, -1.0f, 1e-5f);
    EXPECT_NEAR(r.z, 0.0f, 1e-5f);
}

TEST(QuaternionTest, FromAxisAngleUnitNorm)
{
    const Quaternion q = Quaternion::fromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 45.0f * DEG2RAD);
    EXPECT_NEAR(q.norm(), 1.0f, 1e-6f);
    EXPECT_NEAR(q.w, std::cos(45.0f * DEG2RAD * 0.5f), 1e-6f);
    EXPECT_NEAR(q.y, std::sin(45.0f * DEG2RAD * 0.5f), 1e-6f);
}

TEST(QuaternionTest, FromAxisAngleZeroAxisReturnsIdentity)
{
    // Regression test for zero axis + non-zero angle bug
    const Quaternion q = Quaternion::fromAxisAngle(Vector3(0.0f, 0.0f, 0.0f), 90.0f * DEG2RAD);
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(QuaternionTest, EulerRoundTrip)
{
    const float roll_in  = 30.0f;
    const float pitch_in = 45.0f;
    const float yaw_in   = 60.0f;

    const Quaternion q = Quaternion::fromEuler(roll_in, pitch_in, yaw_in);
    float roll_out{};
    float pitch_out{};
    float yaw_out{};
    q.toEuler(roll_out, pitch_out, yaw_out);

    EXPECT_NEAR(roll_out, roll_in, 1e-2f);
    EXPECT_NEAR(pitch_out, pitch_in, 1e-2f);
    EXPECT_NEAR(yaw_out, yaw_in, 1e-2f);
}

TEST(QuaternionTest, NearGimbalLockDoesNotProduceNaN)
{
    const Quaternion q = Quaternion::fromEuler(10.0f, 89.9f, 20.0f);
    float roll{};
    float pitch{};
    float yaw{};
    q.toEuler(roll, pitch, yaw);

    EXPECT_FALSE(std::isnan(roll));
    EXPECT_FALSE(std::isnan(pitch));
    EXPECT_FALSE(std::isnan(yaw));
    EXPECT_NEAR(pitch, 89.9f, 0.5f);
}

TEST(QuaternionTest, ToEulerClampsSinPToPreventNaN)
{
    // Regression test for asin clamping: intentionally unnormalized quaternion
    // where sinp = 2(w*y - z*x) exceeds 1.0
    const Quaternion q(0.7072f, 0.0f, 0.7072f, 0.0f);
    float roll{};
    float pitch{};
    float yaw{};
    q.toEuler(roll, pitch, yaw);

    EXPECT_FALSE(std::isnan(roll));
    EXPECT_FALSE(std::isnan(pitch));
    EXPECT_FALSE(std::isnan(yaw));
    EXPECT_NEAR(pitch, 90.0f, 1.0f);
}

TEST(QuaternionTest, IntegrateConstantYawRateReachesExpectedYaw)
{
    Quaternion q = Quaternion::identity();
    const Vector3 gyro(0.0f, 0.0f, 90.0f * DEG2RAD);  // 90 dps about Z, in rad/s
    const float dt{0.01f};
    for (int i = 0; i < 100; ++i) {
        q.integrate(gyro, dt);
    }

    float roll{};
    float pitch{};
    float yaw{};
    q.toEuler(roll, pitch, yaw);

    EXPECT_NEAR(yaw, 90.0f, 0.5f);
    EXPECT_NEAR(roll, 0.0f, 0.5f);
    EXPECT_NEAR(pitch, 0.0f, 0.5f);
}
