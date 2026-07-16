/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file vqf.hpp
  @brief Adapter around the vendored basicvqf (Versatile Quaternion-based Filter) algorithm
*/
#ifndef M5_UNIT_IMU_IMU_FILTER_VQF_HPP
#define M5_UNIT_IMU_IMU_FILTER_VQF_HPP

#include <memory>

#include "filter_base.hpp"

// The vendored basicvqf.hpp (see src/imu/filter/third_party/vqf, MIT license, unmodified
// from https://github.com/dlaidig/vqf) picks its floating-point type `vqf_real_t` (used for
// every BasicVQF member, including its in-memory layout) based on whether
// VQF_SINGLE_PRECISION is defined *at the point basicvqf.hpp is included* -- defaulting to
// `double` otherwise. Defining it here, right before the only include of basicvqf.hpp in
// this adapter's public header, makes every translation unit that includes vqf.hpp (this
// adapter's .cpp, and its tests) see `vqf_real_t == float`, matching this repository's
// float-based AhrsFilterBase. This must stay consistent with
// third_party/vqf/basicvqf.cpp, which defines the same macro before compiling the vendored
// implementation itself (see that file for why both sides need to agree).
#ifndef VQF_SINGLE_PRECISION
#define VQF_SINGLE_PRECISION
#endif
#include "third_party/vqf/basicvqf.hpp"

namespace m5 {
namespace imu {

/*!
  @class VQFFilter
  @brief Adapter around the vendored basicvqf algorithm (Daniel Laidig, MIT license; see
  src/imu/filter/third_party/vqf)
  @details basicvqf is the "basic" (no rest/motion bias estimation, no magnetic disturbance
  rejection) C++ reference implementation of the VQF algorithm described in D. Laidig and
  T. Seel, "VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and
  Magnetic Disturbance Rejection", Information Fusion 2023. This adapter performs no
  re-derivation of the underlying math (like FusionFilter, and unlike
  ComplementaryFilter/MadgwickFilter/MahonyFilter/EKFFilter); its job is limited to the
  AhrsFilterBase <-> BasicVQF API boundary:

  - Units: BasicVQF's `update()` takes gyroscope in **rad/s** and accelerometer in
    **m/s^2** (`mag` is documented as "arbitrary units" and is passed through unscaled), which
    differs from this repository's dps/G unit contract (filter_base.hpp) -- update() converts
    gyro (dps -> rad/s, x pi/180) and accel (G -> m/s^2, x standard gravity) accordingly.
  - Earth axes convention: basicvqf's world/earth frame is Z-up like this repository's (a
    static, level unit's accelerometer reading is rotated toward (0,0,+1) by the inclination
    correction in BasicVQF::updateAcc(), the same sign as filter_base.hpp), so no flip is
    needed there. Its magnetometer heading reference, however, converges the *horizontal*
    mag component onto world **+Y**, not +X (see BasicVQF::updateMag()'s
    `atan2(magEarth[0], magEarth[1])`, i.e. atan2(x, y) rather than this repository's
    atan2(y, x)-style convention) -- a fixed +90 degree yaw offset from this repository's mag
    table (filter_base.hpp: mag horizontal along +X at yaw = 0). quaternion() corrects this
    by composing a fixed world-Z rotation of -90 degrees on top of getQuat9D() once any mag
    sample has been consumed (`_mag_seen`); a pure heading rotation, so roll/pitch (which
    already agree with this repository) are untouched. In the mag-less case no correction is
    applied (delta stays 0 and the 6D quaternion already matches the shared convention);
    reset() and a sample-period rebuild clear `_mag_seen` along with basicvqf's delta.
  - Sample period: BasicVQF is constructed with a fixed nominal sampling period (`gyrTs`)
    and has no public API to change it afterwards (`BasicVQF::setup()`, which derives the
    internal filter coefficients from it, is only called from the constructors). Since
    AhrsFilterBase::update() receives an explicit, potentially-varying `dt` every call, this
    adapter tracks the `dt` the underlying BasicVQF was last (re)built with and, when a new
    `dt` differs from it by more than 20%, reconstructs a fresh BasicVQF for the new period
    -- carrying over the current tauAcc/tauMag tuning parameters (see writeTauAcc/writeTauMag)
    but losing the accumulated filter state (gyrQuat/accQuat/delta and the acc/mag low-pass
    histories), as BasicVQF exposes no API to resample that state either. This is expected to
    be rare in practice (real IMU sample periods are effectively constant); small jitter
    around the nominal period is absorbed without a rebuild.
  - basicvqf has no public API to overwrite the attitude estimate directly (unlike
    Fusion/Complementary/Madgwick/Mahony's optional `writeQuaternion()`): `BasicVQF::setState()`
    only accepts a full `BasicVQFState`, whose `gyrQuat`/`accQuat`/`delta` split has no
    general inverse from a single desired output quaternion, so this adapter does not offer
    an equivalent setter.
 */
class VQFFilter : public AhrsFilterBase {
public:
    VQFFilter();

    virtual void reset() override;
    virtual void update(const float dt, const float accel[3], const float gyro[3], const float* mag) override;
    virtual void quaternion(float& w, float& x, float& y, float& z) const override;

    /*!
      @brief Set the accelerometer low-pass time constant (BasicVQFParams::tauAcc)
      @param tau_acc Time constant in seconds, must be > 0
      @return False (and keeps the previous value) if tau_acc is not positive
     */
    bool writeTauAcc(const float tau_acc);
    //! @brief Current accelerometer low-pass time constant
    inline float readTauAcc() const
    {
        return static_cast<float>(_vqf->getParams().tauAcc);
    }

    /*!
      @brief Set the magnetometer heading correction time constant (BasicVQFParams::tauMag)
      @param tau_mag Time constant in seconds, must be > 0
      @return False (and keeps the previous value) if tau_mag is not positive
     */
    bool writeTauMag(const float tau_mag);
    //! @brief Current magnetometer heading correction time constant
    inline float readTauMag() const
    {
        return static_cast<float>(_vqf->getParams().tauMag);
    }

private:
    //! @brief (Re)build _vqf for the given dt if none exists yet, or if dt has drifted from
    //! the period it was last built with by more than VQF_RESAMPLE_THRESHOLD_RATIO
    void ensureSamplePeriod(const float dt);

    std::unique_ptr<::BasicVQF> _vqf{};  // basicvqf's filter instance (see VQFFilter()/ensureSamplePeriod())
    float _built_dt{};                   // Sample period _vqf was (re)built with, in seconds
    bool _mag_seen{};                    // Any mag sample consumed? Gates the heading convention fix
};

}  // namespace imu
}  // namespace m5
#endif
