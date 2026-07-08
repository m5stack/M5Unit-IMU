/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
// Filter-independent scenario helpers shared by the AhrsFilterBase implementations'
// native tests. Each helper only drives the filter (no assertions) so individual
// filter tests stay in charge of tolerances via EXPECT_NEAR.
#ifndef M5_UNIT_IMU_TEST_FILTER_FILTER_SCENARIO_HPP
#define M5_UNIT_IMU_TEST_FILTER_FILTER_SCENARIO_HPP

#include <cmath>
#include <imu/filter/filter_base.hpp>
#include <math/quaternion.hpp>

namespace m5 {
namespace imu {
namespace test {

namespace {
constexpr float SCENARIO_PI{3.14159265358979323846f};
constexpr float SCENARIO_DEG2RAD{SCENARIO_PI / 180.0f};
}  // namespace

// Number of update() steps to cover the given duration at the given step size.
inline int scenario_steps(const float seconds, const float dt)
{
    return static_cast<int>(seconds / dt + 0.5f);
}

// Body lying flat and stationary: accel = (0,0,1) G, gyro = 0, no mag.
inline void run_static(AhrsFilterBase& filter, const float seconds, const float dt)
{
    const float accel[3] = {0.0f, 0.0f, 1.0f};
    const float gyro[3]  = {0.0f, 0.0f, 0.0f};
    const int steps      = scenario_steps(seconds, dt);
    for (int i = 0; i < steps; ++i) {
        filter.update(dt, accel, gyro, nullptr);
    }
}

// Body stationary, tilted 30 degrees about the roll (X) axis: accel = (0, sin30, cos30) G.
// Per filter_base.hpp's derived sign table this is +30 deg roll (right side down).
inline void run_tilt_roll30(AhrsFilterBase& filter, const float seconds, const float dt)
{
    const float accel[3] = {0.0f, std::sin(30.0f * SCENARIO_DEG2RAD), std::cos(30.0f * SCENARIO_DEG2RAD)};
    const float gyro[3]  = {0.0f, 0.0f, 0.0f};
    const int steps      = scenario_steps(seconds, dt);
    for (int i = 0; i < steps; ++i) {
        filter.update(dt, accel, gyro, nullptr);
    }
}

// Body level, rotating about the yaw (Z) axis at a constant rate (no mag).
// Per filter_base.hpp's derived sign table, a positive rate turns left (counter-clockwise
// viewed from above).
inline void run_yaw_rotation(AhrsFilterBase& filter, const float dps, const float seconds, const float dt)
{
    const float accel[3] = {0.0f, 0.0f, 1.0f};
    const float gyro[3]  = {0.0f, 0.0f, dps};
    const int steps      = scenario_steps(seconds, dt);
    for (int i = 0; i < steps; ++i) {
        filter.update(dt, accel, gyro, nullptr);
    }
}

// Body level and stationary with a fixed, north-pointing magnetometer reading: horizontal
// component along body/world +X (heading reference), vertical component pointing down
// (-Z), i.e. a Northern-hemisphere-like field inclination (see filter_base.hpp).
inline void run_with_mag(AhrsFilterBase& filter, const float seconds, const float dt)
{
    const float accel[3] = {0.0f, 0.0f, 1.0f};
    const float gyro[3]  = {0.0f, 0.0f, 0.0f};
    const float mag[3]   = {20.0f, 0.0f, -40.0f};
    const int steps      = scenario_steps(seconds, dt);
    for (int i = 0; i < steps; ++i) {
        filter.update(dt, accel, gyro, mag);
    }
}

// Decompose the filter's current attitude into aerospace ZYX Euler angles (degrees).
inline void euler_of(const AhrsFilterBase& filter, float& roll_deg, float& pitch_deg, float& yaw_deg)
{
    float w{};
    float x{};
    float y{};
    float z{};
    filter.quaternion(w, x, y, z);
    const m5::math::Quaternion q(w, x, y, z);
    q.toEuler(roll_deg, pitch_deg, yaw_deg);
}

}  // namespace test
}  // namespace imu
}  // namespace m5
#endif
