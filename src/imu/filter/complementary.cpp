/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file complementary.cpp
  @brief Angle-based first-order complementary AHRS filter
*/
#include "complementary.hpp"

#include <cmath>

#include <math/quaternion.hpp>

namespace m5 {
namespace imu {

namespace {
constexpr float COMPLEMENTARY_PI{3.14159265358979323846f};
constexpr float COMPLEMENTARY_DEG2RAD{COMPLEMENTARY_PI / 180.0f};
constexpr float COMPLEMENTARY_RAD2DEG{180.0f / COMPLEMENTARY_PI};
}  // namespace

void ComplementaryFilter::reset()
{
    _roll = _pitch = _yaw = 0.0f;
}

void ComplementaryFilter::update(const float dt, const float accel[3], const float gyro[3], const float* mag)
{
    if (dt <= 0.0f) {
        return;
    }

    const float roll_gyro  = _roll + gyro[0] * dt;
    const float pitch_gyro = _pitch + gyro[1] * dt;
    const float yaw_gyro   = _yaw + gyro[2] * dt;

    // Roll/pitch from the accelerometer (see filter_base.hpp for the coordinate
    // convention and its derived sign table: body X forward / Y left / Z up,
    // static level reading = (0,0,+1) G; +roll = right side down, +pitch = nose down).
    const float roll_acc = std::atan2(accel[1], accel[2]) * COMPLEMENTARY_RAD2DEG;
    const float pitch_acc =
        std::atan2(-accel[0], std::sqrt(accel[1] * accel[1] + accel[2] * accel[2])) * COMPLEMENTARY_RAD2DEG;

    _roll  = _alpha * roll_gyro + (1.0f - _alpha) * roll_acc;
    _pitch = _alpha * pitch_gyro + (1.0f - _alpha) * pitch_acc;

    if (mag) {
        const float roll_rad  = _roll * COMPLEMENTARY_DEG2RAD;
        const float pitch_rad = _pitch * COMPLEMENTARY_DEG2RAD;
        const float cr        = std::cos(roll_rad);
        const float sr        = std::sin(roll_rad);
        const float cp        = std::cos(pitch_rad);
        const float sp        = std::sin(pitch_rad);

        // Tilt-compensated horizontal magnetic field components (standard
        // roll/pitch-compensated compass heading formula).
        const float xh      = mag[0] * cp + mag[1] * sr * sp + mag[2] * cr * sp;
        const float yh      = mag[1] * cr - mag[2] * sr;
        const float yaw_mag = std::atan2(-yh, xh) * COMPLEMENTARY_RAD2DEG;

        _yaw = _alpha * yaw_gyro + (1.0f - _alpha) * yaw_mag;
    } else {
        _yaw = yaw_gyro;
    }
}

void ComplementaryFilter::quaternion(float& w, float& x, float& y, float& z) const
{
    const m5::math::Quaternion q = m5::math::Quaternion::fromEuler(_roll, _pitch, _yaw);
    w                            = q.w;
    x                            = q.x;
    y                            = q.y;
    z                            = q.z;
}

bool ComplementaryFilter::writeAlpha(const float alpha)
{
    if (!(alpha > 0.0f && alpha < 1.0f)) {
        return false;
    }
    _alpha = alpha;
    return true;
}

}  // namespace imu
}  // namespace m5
