/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file filter_base.hpp
  @brief Common contract for AHRS attitude filters
  @details
  ## Unit contract (applies to every AhrsFilterBase implementation)
  - accel: G (specific force including gravity; a stationary, level unit reads (0,0,+1) G)
  - gyro: dps (degrees per second, body frame)
  - mag: microtesla (uT); pass `nullptr` to run 6-axis (accel+gyro only)
  - dt: seconds elapsed since the previous update() call

  ## Coordinate convention
  Body frame is right-handed: X forward, Y left, Z up (a ROS REP-103-style body frame;
  note that (forward, right, up) would be left-handed -- Y must be left to keep the
  triad right-handed with Z up). This matches what M5Unified and most consumer IMUs
  report as-is -- a unit lying flat and stationary reads accel = (0, 0, +1) G (the
  sensor's raw output, not the true gravity vector, which points the other way).

  This ("sensor output as-is", Z-up) convention is used instead of the classic aerospace
  NED (Z-down) convention because it needs no sign flip between raw sensor samples and
  filter input, keeping the AhrsController -> filter boundary a straight physical-unit
  passthrough.

  The quaternion returned by quaternion() rotates a body-frame vector into the reference
  (world) frame: `v_world = q.rotate(v_body)` (see m5::math::Quaternion). Euler angles use
  the aerospace ZYX (yaw-pitch-roll) convention in degrees, matching
  m5::math::Quaternion::fromEuler() / toEuler().

  The physical meaning of each sign below is not asserted; it is derived from
  `m5::math::Quaternion::fromEuler()` + `rotate()` themselves (see
  test/native/test_filter for the scenarios this table is checked against):

  | Signal                                 | Derived physical meaning                                     |
  |-----------------------------------------|----------------------------------------------------------------|
  | accel = (0, 0, +1) G, static and level  | Body Z (up) aligned with world Z (up)                           |
  | +roll, e.g. `fromEuler(+30, 0, 0)`      | Right side down (roll right); body Y (left) tilts toward +Z     |
  | +pitch, e.g. `fromEuler(0, +30, 0)`     | Nose down; body X (forward) tilts toward world -Z               |
  | +yaw, e.g. `fromEuler(0, 0, +30)`       | Turn left (CCW viewed from above); body X tilts toward world +Y |
  | mag = (20, 0, -40) uT, level, yaw = 0   | Horizontal along +X (north-ish heading); vertical -Z (down):    |
  | (world frame at yaw = 0)                | Northern-hemisphere-like field inclination                      |

  Concretely, a body held level and pitched/rolled by 30 degrees per the table above
  reproduces exactly the accelerometer readings used by
  `test/native/test_filter/filter_scenario.hpp` (e.g. `run_tilt_roll30()` uses
  accel = (0, sin30, cos30) G for +30 deg roll), so `ComplementaryFilter`'s
  `atan2(accel[1], accel[2])` (roll) and `atan2(-accel[0], ...)` (pitch) formulas in
  complementary.cpp recover these same signs.
*/
#ifndef M5_UNIT_IMU_IMU_FILTER_FILTER_BASE_HPP
#define M5_UNIT_IMU_IMU_FILTER_FILTER_BASE_HPP

#include <cstdint>

namespace m5 {
namespace imu {

/*!
  @enum FilterType
  @brief Built-in AHRS filter implementations selectable via AhrsController::selectFilter
 */
enum class FilterType : uint8_t {
    Complementary,  //!< @brief Angle-based first-order complementary filter
    Madgwick,       //!< @brief Gradient-descent orientation filter (Madgwick 2010)
    Mahony,         //!< @brief Explicit complementary filter with PI feedback (Mahony)
    EKF,            //!< @brief Quaternion extended Kalman filter
    Fusion,         //!< @brief Vendored x-io Fusion AHRS
    VQF,            //!< @brief Vendored basicvqf (Versatile Quaternion-based Filter)
};

/*!
  @class AhrsFilterBase
  @brief Framework-independent contract shared by every AHRS attitude filter
  @details Implementations consume physical-unit IMU samples and expose the fused
  attitude as a quaternion. See the unit and coordinate convention documented above
  (@ref filter_base.hpp).
 */
class AhrsFilterBase {
public:
    virtual ~AhrsFilterBase() = default;

    //! @brief Reset the internal attitude estimate (and any filter state) to identity
    virtual void reset() = 0;

    /*!
      @brief Fuse one IMU sample into the attitude estimate
      @param dt Time elapsed since the previous update(), in seconds
      @param accel Accelerometer reading (G), body frame, X/Y/Z
      @param gyro Gyroscope reading (dps), body frame, X/Y/Z
      @param mag Magnetometer reading (uT), body frame, X/Y/Z; pass `nullptr` for
      6-axis (accel+gyro only) operation
     */
    virtual void update(const float dt, const float accel[3], const float gyro[3], const float* mag) = 0;

    /*!
      @brief Current fused attitude
      @param[out] w Quaternion scalar part
      @param[out] x Quaternion vector part X
      @param[out] y Quaternion vector part Y
      @param[out] z Quaternion vector part Z
     */
    virtual void quaternion(float& w, float& x, float& y, float& z) const = 0;
};

}  // namespace imu
}  // namespace m5
#endif
