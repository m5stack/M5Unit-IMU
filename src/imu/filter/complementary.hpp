/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file complementary.hpp
  @brief Angle-based first-order complementary AHRS filter
*/
#ifndef M5_UNIT_IMU_IMU_FILTER_COMPLEMENTARY_HPP
#define M5_UNIT_IMU_IMU_FILTER_COMPLEMENTARY_HPP

#include "filter_base.hpp"

namespace m5 {
namespace imu {

/*!
  @class ComplementaryFilter
  @brief Angle-based first-order complementary filter
  @details Blends the gyro-integrated angle with a reference angle independently for
  roll, pitch and yaw: `angle = alpha * (angle + gyro * dt) + (1 - alpha) * angle_ref`.
  Roll/pitch reference angles come from the accelerometer; the yaw reference comes from
  a tilt-compensated magnetometer heading when a magnetometer sample is supplied (pure
  gyro integration otherwise). See filter_base.hpp for the unit and coordinate
  convention this filter assumes.
 */
class ComplementaryFilter : public AhrsFilterBase {
public:
    ComplementaryFilter() = default;

    virtual void reset() override;
    virtual void update(const float dt, const float accel[3], const float gyro[3], const float* mag) override;
    virtual void quaternion(float& w, float& x, float& y, float& z) const override;

    /*!
      @brief Set the complementary blend factor
      @param alpha Weight given to the gyro-integrated angle, must be in the open
      interval (0, 1)
      @return False (and keeps the previous value) if alpha is out of range
     */
    bool writeAlpha(const float alpha);
    //! @brief Current complementary blend factor
    inline float readAlpha() const
    {
        return _alpha;
    }

private:
    float _alpha{0.98f};  // Blend factor (weight of the gyro-integrated angle)
    float _roll{};        // Current roll estimate (deg)
    float _pitch{};       // Current pitch estimate (deg)
    float _yaw{};         // Current yaw estimate (deg)
};

}  // namespace imu
}  // namespace m5
#endif
