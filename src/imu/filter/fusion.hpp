/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file fusion.hpp
  @brief Adapter around the vendored x-io Fusion AHRS algorithm
*/
#ifndef M5_UNIT_IMU_IMU_FILTER_FUSION_HPP
#define M5_UNIT_IMU_IMU_FILTER_FUSION_HPP

#include "filter_base.hpp"

// The vendored sources are plain C (see src/imu/filter/third_party/fusion, MIT license,
// unmodified from https://github.com/xioTechnologies/Fusion) and FusionAhrs.h itself has
// no extern "C" guard of its own, so this adapter supplies one at the include site to
// give FusionAhrs.c's symbols C linkage when called from this C++ translation unit.
extern "C" {
#include "third_party/fusion/FusionAhrs.h"
}

namespace m5 {
namespace imu {

/*!
  @class FusionFilter
  @brief Adapter around the vendored x-io Fusion AHRS algorithm (x-io Technologies,
  MIT license; see src/imu/filter/third_party/fusion)
  @details Fusion's own attitude/feedback algorithm is used as-is (this adapter performs
  no re-derivation of the underlying math, unlike ComplementaryFilter/MadgwickFilter/
  MahonyFilter/EKFFilter). The adapter's job is limited to the AhrsFilterBase <-> Fusion
  API boundary:
  - Units: Fusion's `FusionAhrsUpdate()`/`FusionAhrsUpdateNoMagnetometer()` already take
    gyroscope in dps, accelerometer in G and magnetometer in "any calibrated units" (see
    FusionAhrs.h), matching this repository's unit contract (filter_base.hpp) exactly --
    no scaling is applied here.
  - Earth axes convention: `FusionAhrsSettings.convention` is fixed to
    `FusionConventionNwu` (North(X)/West(Y)/Up(Z)) in the constructor, which is the only
    one of Fusion's three conventions (Nwu/Enu/Ned) consistent with this repository's
    world frame (filter_base.hpp): FusionConventionNed's earth Z axis is "down", which
    contradicts a static/level unit reading accel = (0,0,+1) G; that leaves Nwu and Enu,
    both earth-Z-up, differing only in which world axis the horizontal magnetometer
    reference aligns to (X for Nwu, Y for Enu) -- filter_base.hpp's mag table places that
    reference along world +X at yaw = 0, which is Nwu's "North" (X) axis. (This is also
    Fusion's own library default, `fusionAhrsDefaultSettings.convention`.) The Fusion
    quaternion is read back via `FusionAhrsGetQuaternion()` component-for-component
    (w,x,y,z) with no reordering or sign flip: Fusion's own `HalfGravity()` formula for
    Nwu/Enu (see FusionAhrs.c) is algebraically identical to the predicted-gravity-in-
    body-frame expansion this repository's other filters derive from
    `m5::math::Quaternion::rotate()`/`fromEuler()` (see e.g. MahonyFilter::update()), so
    the two quaternion representations already agree.
  - Startup ramp: Fusion always starts (on `FusionAhrsInitialise()`/`FusionAhrsRestart()`,
    i.e. also after `reset()`) with a high gain (10.0) that ramps down to the configured
    gain over a fixed 3-second window (`STARTUP_PERIOD` in FusionAhrs.c) -- this is
    Fusion's own behavior, not something this adapter can tune. It also means
    `FusionAhrsUpdateNoMagnetometer()` forces the yaw heading to 0 on every call while
    that window is open, so gyro-only yaw integration only becomes observable once the
    filter has been running (real, cumulative dt) for more than 3 seconds; see
    test/native/test_filter/fusion_test.cpp for how the shared scenarios account for this.
 */
class FusionFilter : public AhrsFilterBase {
public:
    FusionFilter();

    virtual void reset() override;
    virtual void update(const float dt, const float accel[3], const float gyro[3], const float* mag) override;
    virtual void quaternion(float& w, float& x, float& y, float& z) const override;

    /*!
      @brief Set the AHRS feedback gain (FusionAhrsSettings.gain)
      @param gain Feedback gain, must be >= 0 (0 disables accelerometer/magnetometer
      outlier rejection as well as feedback correction speed limiting -- see Fusion's own
      `FusionAhrsSetSettings()`)
      @return False (and keeps the previous value) if gain is negative
     */
    bool writeGain(const float gain);
    //! @brief Current feedback gain
    inline float readGain() const
    {
        return _settings.gain;
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
    ::FusionAhrsSettings _settings{};  // Cached settings (Fusion keeps its own copy internally too)
    ::FusionAhrs _ahrs{};              // Fusion's AHRS state (quaternion + internal feedback state)
};

}  // namespace imu
}  // namespace m5
#endif
