/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file mahony.cpp
  @brief Explicit complementary filter with PI feedback (Mahony et al. 2008)
*/
#include "mahony.hpp"

#include <cmath>

namespace m5 {
namespace imu {

namespace {
constexpr float MAHONY_PI{3.14159265358979323846f};
constexpr float MAHONY_DEG2RAD{MAHONY_PI / 180.0f};
//! @brief Anti-windup clamp applied to each integral accumulator component
constexpr float MAHONY_INTEGRAL_LIMIT{0.5f};

//! @brief Clamp value to +-limit
inline float clamp_symmetric(const float value, const float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}
}  // namespace

void MahonyFilter::reset()
{
    _q              = m5::math::Quaternion::identity();
    _integral_error = m5::math::Vector3();
}

void MahonyFilter::update(const float dt, const float accel[3], const float gyro[3], const float* mag)
{
    if (dt <= 0.0f) {
        return;
    }

    const float gx = gyro[0] * MAHONY_DEG2RAD;
    const float gy = gyro[1] * MAHONY_DEG2RAD;
    const float gz = gyro[2] * MAHONY_DEG2RAD;

    const float w = _q.w;
    const float x = _q.x;
    const float y = _q.y;
    const float z = _q.z;

    // Body-frame PI feedback error, accumulated below from every available
    // reference direction (gravity, and mag when supplied).
    float ex{};
    float ey{};
    float ez{};

    const float a_norm = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (a_norm > 0.0f) {
        const float ax = accel[0] / a_norm;
        const float ay = accel[1] / a_norm;
        const float az = accel[2] / a_norm;

        // Predicted gravity direction in the body frame: the world reference (0,0,1)
        // (see filter_base.hpp: a static, level unit reads accel = (0,0,+1) G) rotated
        // world->body, i.e. by the inverse (conjugate) of the body->world rotation
        // that q.rotate() implements. Same expansion as MadgwickFilter::update().
        const float vx = 2.0f * (x * z - w * y);
        const float vy = 2.0f * (w * x + y * z);
        const float vz = 1.0f - 2.0f * (x * x + y * y);

        // Body-frame error term: measured direction cross predicted direction. For
        // a small attitude error this cross product points along the (small-angle)
        // rotation axis that would carry the predicted direction onto the measured
        // one, so feeding Kp*e (and Ki*integral(e)) into the gyro rate steers the
        // attitude estimate toward the measurement (verified against this
        // repository's sign convention: for a level unit read as accel =
        // (0, sin30, cos30) G -- i.e. +30 deg roll per filter_base.hpp -- with the
        // estimate still at identity, this yields e = (sin30, 0, 0), a positive
        // rotation about body X that converges the estimate toward +30 deg roll).
        ex += ay * vz - az * vy;
        ey += az * vx - ax * vz;
        ez += ax * vy - ay * vx;

        if (mag != nullptr) {
            const float m_norm = std::sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
            if (m_norm > 0.0f) {
                const float mx = mag[0] / m_norm;
                const float my = mag[1] / m_norm;
                const float mz = mag[2] / m_norm;

                // Rotate the measured (normalized) mag reading body->world with the
                // current attitude estimate, then flatten its horizontal component
                // onto world +X to get this step's world-frame mag reference
                // (bx, 0, bz) -- recomputed every step, per Madgwick 2010 (reused
                // here for the same reason: a fixed world reference would fight the
                // yaw estimate whenever the body is not exactly level).
                const m5::math::Vector3 h = _q.rotate(m5::math::Vector3(mx, my, mz));
                const float bx            = std::sqrt(h.x * h.x + h.y * h.y);
                const float bz            = h.z;

                // Predicted mag direction in the body frame: (bx, 0, bz) rotated
                // world->body, same expansion as the accel term above.
                const float mvx = bx * (1.0f - 2.0f * (y * y + z * z)) + 2.0f * bz * (x * z - w * y);
                const float mvy = 2.0f * bx * (x * y - w * z) + 2.0f * bz * (y * z + w * x);
                const float mvz = 2.0f * bx * (x * z + w * y) + bz * (1.0f - 2.0f * (x * x + y * y));

                ex += my * mvz - mz * mvy;
                ey += mz * mvx - mx * mvz;
                ez += mx * mvy - my * mvx;
            }
        }
    }

    if (_ki > 0.0f) {
        _integral_error.x = clamp_symmetric(_integral_error.x + ex * dt, MAHONY_INTEGRAL_LIMIT);
        _integral_error.y = clamp_symmetric(_integral_error.y + ey * dt, MAHONY_INTEGRAL_LIMIT);
        _integral_error.z = clamp_symmetric(_integral_error.z + ez * dt, MAHONY_INTEGRAL_LIMIT);
    }

    const float corrected_gx = gx + _kp * ex + _ki * _integral_error.x;
    const float corrected_gy = gy + _kp * ey + _ki * _integral_error.y;
    const float corrected_gz = gz + _kp * ez + _ki * _integral_error.z;

    _q.integrate(m5::math::Vector3(corrected_gx, corrected_gy, corrected_gz), dt);
}

void MahonyFilter::quaternion(float& w, float& x, float& y, float& z) const
{
    w = _q.w;
    x = _q.x;
    y = _q.y;
    z = _q.z;
}

bool MahonyFilter::writeKp(const float kp)
{
    if (!(kp > 0.0f)) {
        return false;
    }
    _kp = kp;
    return true;
}

bool MahonyFilter::writeKi(const float ki)
{
    if (ki < 0.0f) {
        return false;
    }
    _ki = ki;
    return true;
}

bool MahonyFilter::writeQuaternion(const float w, const float x, const float y, const float z)
{
    m5::math::Quaternion q(w, x, y, z);
    if (!q.normalize()) {
        return false;
    }
    _q = q;
    return true;
}

}  // namespace imu
}  // namespace m5
