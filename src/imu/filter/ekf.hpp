/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file ekf.hpp
  @brief Quaternion extended Kalman filter AHRS orientation filter
*/
#ifndef M5_UNIT_IMU_IMU_FILTER_EKF_HPP
#define M5_UNIT_IMU_IMU_FILTER_EKF_HPP

#include <math/matrix.hpp>
#include <math/quaternion.hpp>
#include <math/vector3.hpp>

#include "filter_base.hpp"

namespace m5 {
namespace imu {

/*!
  @class EKFFilter
  @brief Quaternion extended Kalman filter (EKF) with gyro bias estimation
  @details A standard additive EKF with a 7-element state x = [q (4), gyro bias
  b (3, rad/s internally)] and a 7x7 covariance, re-derived for this
  repository's coordinate/quaternion convention (see filter_base.hpp): body
  Z-up, `v_world = q.rotate(v_body)`. Each update() runs:
  - Predict: propagate q via the gyro-minus-bias body rate (reusing
    m5::math::Quaternion::integrate(), which also renormalizes q -- the
    quaternion is over-parameterized for the 3 rotational degrees of freedom,
    so per-step renormalization after the linear propagation step is the
    standard way to keep it on the unit sphere), bias held constant, and
    propagate the covariance with a first-order state transition Jacobian.
  - Update (accel): sequential 3-axis observation of the predicted gravity
    direction in the body frame (same expansion as MadgwickFilter/MahonyFilter).
  - Update (mag, when supplied): a second, separate sequential 3-axis
    observation of the predicted horizontal magnetic field direction, with the
    world-frame reference (bx, 0, bz) recomputed every step from the current
    attitude estimate (same "flatten mag to horizontal" trick as
    MadgwickFilter/MahonyFilter).
  Both observation updates reuse a single 3x3 (not 6x6) Kalman gain computation
  (see src/math/matrix.hpp's invert3x3()), which is lighter than solving a
  combined 6x6 system and keeps the accel and mag corrections independent.
 */
class EKFFilter : public AhrsFilterBase {
public:
    EKFFilter()
    {
        reset_state();
    }

    virtual void reset() override;
    virtual void update(const float dt, const float accel[3], const float gyro[3], const float* mag) override;
    virtual void quaternion(float& w, float& x, float& y, float& z) const override;

    /*!
      @brief Set the process/observation noise parameters
      @param gyro_noise_dps Gyro measurement noise (1-sigma), degrees per second, must be > 0
      @param accel_noise_g Accelerometer measurement noise (1-sigma), G, must be > 0
      @param mag_noise_ut Magnetometer measurement noise (1-sigma), microtesla, must be > 0
      @param bias_random_walk_dps Gyro bias random walk (1-sigma), degrees per second, must be > 0
      @return False (and keeps the previous values) if any parameter is not positive
      @note The gyro_noise_dps parameter is internally scaled by EKF_PROCESS_NOISE_SCALE
      (currently 100.0x) when building the process noise covariance Q. This means the
      actual filter response will be stronger (more responsive to references) than the
      datasheet raw values might suggest; use this method iteratively with your sensor
      and reference characteristics to tune the desired tracking behavior.
     */
    bool writeNoiseParams(const float gyro_noise_dps, const float accel_noise_g, const float mag_noise_ut,
                          const float bias_random_walk_dps);
    //! @brief Current gyro measurement noise (dps)
    inline float readGyroNoise() const
    {
        return _gyro_noise_dps;
    }
    //! @brief Current accelerometer measurement noise (G)
    inline float readAccelNoise() const
    {
        return _accel_noise_g;
    }
    //! @brief Current magnetometer measurement noise (uT)
    inline float readMagNoise() const
    {
        return _mag_noise_ut;
    }
    //! @brief Current gyro bias random walk (dps)
    inline float readBiasRandomWalk() const
    {
        return _bias_random_walk_dps;
    }

    //! @brief Current estimated gyro bias (dps, body frame)
    m5::math::Vector3 readGyroBias() const;

    /*!
      @brief Overwrite the current attitude estimate directly
      @details Not part of AhrsFilterBase; intended for seeding a known starting
      attitude (e.g. tests, or resuming from a previously stored orientation).
      Does not touch the bias estimate or covariance.
      @param w W component of the quaternion
      @param x X component of the quaternion
      @param y Y component of the quaternion
      @param z Z component of the quaternion
      @return False (and keeps the previous value) if the resulting quaternion is
      (near) zero
     */
    bool writeQuaternion(const float w, const float x, const float y, const float z);

private:
    // Non-virtual state initializer shared by ctor and reset(); safe to call from ctor
    void reset_state();
    void predict(const float dt, const m5::math::Vector3& gyro_rad_s);
    bool apply_axis3_update(const m5::math::Matrix<3, 7>& h, const m5::math::Vector3& innovation,
                            const float noise_var);

    m5::math::Quaternion _q{};        // Current attitude estimate (identity by default)
    m5::math::Vector3 _bias_rad_s{};  // Gyro bias estimate, rad/s, body frame
    m5::math::Matrix<7, 7> _p{};      // State covariance (state = [q(4), bias(3)])

    float _gyro_noise_dps{0.5f};         // Gyro measurement noise, 1-sigma, dps
    float _accel_noise_g{0.05f};         // Accelerometer measurement noise, 1-sigma, G
    float _mag_noise_ut{2.0f};           // Magnetometer measurement noise, 1-sigma, uT
    float _bias_random_walk_dps{0.01f};  // Gyro bias random walk, 1-sigma, dps
};

}  // namespace imu
}  // namespace m5
#endif
