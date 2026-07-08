/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file mahony.hpp
  @brief Explicit complementary filter with PI feedback (Mahony et al. 2008)
*/
#ifndef M5_UNIT_IMU_IMU_FILTER_MAHONY_HPP
#define M5_UNIT_IMU_IMU_FILTER_MAHONY_HPP

#include <math/quaternion.hpp>
#include <math/vector3.hpp>

#include "filter_base.hpp"

namespace m5 {
namespace imu {

/*!
  @class MahonyFilter
  @brief Explicit complementary filter with proportional-integral feedback (Mahony et al. 2008)
  @details Implements the explicit complementary filter on SO(3) from Mahony, Hamel and
  Pflimlin's "Nonlinear Complementary Filters on the Special Orthogonal Group" (2008),
  re-derived for this repository's coordinate/quaternion convention (see filter_base.hpp):
  body Z-up, `v_world = q.rotate(v_body)`. Each update() integrates the gyro rate after
  correcting it with a proportional-integral term driven by the body-frame cross-product
  error between the measured accel/mag directions and the predicted reference
  direction(s) (gravity, and -- when a magnetometer sample is supplied -- the horizontal
  magnetic field recomputed every step from the current attitude estimate).
 */
class MahonyFilter : public AhrsFilterBase {
public:
    MahonyFilter() = default;

    virtual void reset() override;
    virtual void update(const float dt, const float accel[3], const float gyro[3], const float* mag) override;
    virtual void quaternion(float& w, float& x, float& y, float& z) const override;

    /*!
      @brief Set the proportional feedback gain
      @param kp Gain applied to the instantaneous cross-product error, must be > 0
      @return False (and keeps the previous value) if kp is not positive
     */
    bool writeKp(const float kp);
    //! @brief Current proportional feedback gain
    inline float readKp() const
    {
        return _kp;
    }

    /*!
      @brief Set the integral feedback gain
      @param ki Gain applied to the accumulated cross-product error, must be >= 0
      (0 disables integral action; the accumulator itself is left untouched but
      contributes nothing while ki is 0)
      @return False (and keeps the previous value) if ki is negative
     */
    bool writeKi(const float ki);
    //! @brief Current integral feedback gain
    inline float readKi() const
    {
        return _ki;
    }

    /*!
      @brief Overwrite the current attitude estimate directly
      @details Not part of AhrsFilterBase; intended for seeding a known starting
      attitude (e.g. tests, or resuming from a previously stored orientation).
      @param w W component of the quaternion
      @param x X component of the quaternion
      @param y Y component of the quaternion
      @param z Z component of the quaternion
      @return False (and keeps the previous value) if the resulting quaternion is
      (near) zero
     */
    bool writeQuaternion(const float w, const float x, const float y, const float z);

private:
    m5::math::Quaternion _q{};            // Current attitude estimate (identity by default)
    m5::math::Vector3 _integral_error{};  // Integral feedback accumulator (sum of e * dt)
    float _kp{1.0f};                      // Proportional feedback gain
    float _ki{0.0f};                      // Integral feedback gain
};

}  // namespace imu
}  // namespace m5
#endif
