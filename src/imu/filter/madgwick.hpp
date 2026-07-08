/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file madgwick.hpp
  @brief Gradient-descent AHRS orientation filter (Madgwick 2010)
*/
#ifndef M5_UNIT_IMU_IMU_FILTER_MADGWICK_HPP
#define M5_UNIT_IMU_IMU_FILTER_MADGWICK_HPP

#include <math/quaternion.hpp>

#include "filter_base.hpp"

namespace m5 {
namespace imu {

/*!
  @class MadgwickFilter
  @brief Gradient-descent orientation filter (Madgwick 2010)
  @details Implements the gradient descent algorithm from S. Madgwick's "An efficient
  orientation filter for inertial and inertial/magnetic sensor arrays" (2010), re-derived
  for this repository's coordinate/quaternion convention (see filter_base.hpp): body
  Z-up, `v_world = q.rotate(v_body)`. Each update() blends a gyro-integrated quaternion
  rate with a gradient-descent correction step that drives the predicted reference
  direction(s) (gravity, and -- when a magnetometer sample is supplied -- the horizontal
  magnetic field recomputed every step from the current attitude estimate) toward the
  measured accel/mag directions.
 */
class MadgwickFilter : public AhrsFilterBase {
public:
    MadgwickFilter() = default;

    virtual void reset() override;
    virtual void update(const float dt, const float accel[3], const float gyro[3], const float* mag) override;
    virtual void quaternion(float& w, float& x, float& y, float& z) const override;

    /*!
      @brief Set the gradient descent gain
      @param beta Step size applied to the normalized gradient correction, must be > 0
      @return False (and keeps the previous value) if beta is not positive
     */
    bool writeBeta(const float beta);
    //! @brief Current gradient descent gain
    inline float readBeta() const
    {
        return _beta;
    }

    /*!
      @brief Overwrite the current attitude estimate directly
      @details Not part of AhrsFilterBase; intended for seeding a known starting
      attitude (e.g. tests, or resuming from a previously stored orientation).
      @return False (and keeps the previous value) if the resulting quaternion is
      (near) zero
     */
    bool writeQuaternion(const float w, const float x, const float y, const float z);

private:
    m5::math::Quaternion _q{};  // Current attitude estimate (identity by default)
    float _beta{0.1f};          // Gradient descent gain
};

}  // namespace imu
}  // namespace m5
#endif
