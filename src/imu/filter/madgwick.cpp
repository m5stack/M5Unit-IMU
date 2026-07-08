/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file madgwick.cpp
  @brief Gradient-descent AHRS orientation filter (Madgwick 2010)
*/
#include "madgwick.hpp"

#include <cmath>

namespace m5 {
namespace imu {

namespace {
constexpr float MADGWICK_PI{3.14159265358979323846f};
constexpr float MADGWICK_DEG2RAD{MADGWICK_PI / 180.0f};
}  // namespace

void MadgwickFilter::reset()
{
    _q = m5::math::Quaternion::identity();
}

void MadgwickFilter::update(const float dt, const float accel[3], const float gyro[3], const float* mag)
{
    if (dt <= 0.0f) {
        return;
    }

    const float gx = gyro[0] * MADGWICK_DEG2RAD;
    const float gy = gyro[1] * MADGWICK_DEG2RAD;
    const float gz = gyro[2] * MADGWICK_DEG2RAD;

    const float w = _q.w;
    const float x = _q.x;
    const float y = _q.y;
    const float z = _q.z;

    // Gyro-only quaternion rate: qDot = 0.5 * q (x) (0, gx, gy, gz).
    const m5::math::Quaternion omega(0.0f, gx, gy, gz);
    const m5::math::Quaternion qdot_gyro = _q * omega;
    float qdot_w                         = 0.5f * qdot_gyro.w;
    float qdot_x                         = 0.5f * qdot_gyro.x;
    float qdot_y                         = 0.5f * qdot_gyro.y;
    float qdot_z                         = 0.5f * qdot_gyro.z;

    const float a_norm = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (a_norm > 0.0f) {
        const float ax = accel[0] / a_norm;
        const float ay = accel[1] / a_norm;
        const float az = accel[2] / a_norm;

        // Predicted gravity direction in the body frame: the world reference (0,0,1)
        // (see filter_base.hpp: a static, level unit reads accel = (0,0,+1) G) rotated
        // world->body, i.e. by the inverse (conjugate) of the body->world rotation
        // that q.rotate() implements.
        const float f1 = 2.0f * (x * z - w * y) - ax;
        const float f2 = 2.0f * (w * x + y * z) - ay;
        const float f3 = 1.0f - 2.0f * (x * x + y * y) - az;

        float grad_w = -2.0f * y * f1 + 2.0f * x * f2;
        float grad_x = 2.0f * z * f1 + 2.0f * w * f2 - 4.0f * x * f3;
        float grad_y = -2.0f * w * f1 + 2.0f * z * f2 - 4.0f * y * f3;
        float grad_z = 2.0f * x * f1 + 2.0f * y * f2;

        if (mag != nullptr) {
            const float m_norm = std::sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
            if (m_norm > 0.0f) {
                const float mx = mag[0] / m_norm;
                const float my = mag[1] / m_norm;
                const float mz = mag[2] / m_norm;

                // Rotate the measured (normalized) mag reading body->world with the
                // current attitude estimate, then flatten its horizontal component
                // onto world +X to get this step's world-frame mag reference
                // (bx, 0, bz) -- recomputed every step, per Madgwick 2010.
                const m5::math::Vector3 h = _q.rotate(m5::math::Vector3(mx, my, mz));
                const float bx            = std::sqrt(h.x * h.x + h.y * h.y);
                const float bz            = h.z;

                const float f4 = bx * (1.0f - 2.0f * (y * y + z * z)) + 2.0f * bz * (x * z - w * y) - mx;
                const float f5 = 2.0f * bx * (x * y - w * z) + 2.0f * bz * (y * z + w * x) - my;
                const float f6 = 2.0f * bx * (x * z + w * y) + bz * (1.0f - 2.0f * (x * x + y * y)) - mz;

                grad_w += -2.0f * bz * y * f4 + (-2.0f * bx * z + 2.0f * bz * x) * f5 + 2.0f * bx * y * f6;
                grad_x +=
                    2.0f * bz * z * f4 + (2.0f * bx * y + 2.0f * bz * w) * f5 + (2.0f * bx * z - 4.0f * bz * x) * f6;
                grad_y += (-4.0f * bx * y - 2.0f * bz * w) * f4 + (2.0f * bx * x + 2.0f * bz * z) * f5 +
                          (2.0f * bx * w - 4.0f * bz * y) * f6;
                grad_z +=
                    (-4.0f * bx * z + 2.0f * bz * x) * f4 + (-2.0f * bx * w + 2.0f * bz * y) * f5 + 2.0f * bx * x * f6;
            }
        }

        const float grad_norm = std::sqrt(grad_w * grad_w + grad_x * grad_x + grad_y * grad_y + grad_z * grad_z);
        if (grad_norm > 0.0f) {
            const float inv = 1.0f / grad_norm;
            qdot_w -= _beta * grad_w * inv;
            qdot_x -= _beta * grad_x * inv;
            qdot_y -= _beta * grad_y * inv;
            qdot_z -= _beta * grad_z * inv;
        }
    }

    _q.w += qdot_w * dt;
    _q.x += qdot_x * dt;
    _q.y += qdot_y * dt;
    _q.z += qdot_z * dt;
    _q.normalize();
}

void MadgwickFilter::quaternion(float& w, float& x, float& y, float& z) const
{
    w = _q.w;
    x = _q.x;
    y = _q.y;
    z = _q.z;
}

bool MadgwickFilter::writeBeta(const float beta)
{
    if (!(beta > 0.0f)) {
        return false;
    }
    _beta = beta;
    return true;
}

bool MadgwickFilter::writeQuaternion(const float w, const float x, const float y, const float z)
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
