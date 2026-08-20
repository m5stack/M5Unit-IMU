/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file vqf.hpp
  @brief Adapter around the vendored VQF (Versatile Quaternion-based Filter) algorithm
*/
#ifndef M5_UNIT_IMU_IMU_FILTER_VQF_HPP
#define M5_UNIT_IMU_IMU_FILTER_VQF_HPP

#include <memory>

#include <math/vector3.hpp>

#include "filter_base.hpp"

// The vendored vqf.hpp (see src/imu/filter/third_party/vqf, MIT license, unmodified from
// https://github.com/dlaidig/vqf) picks its floating-point type `vqf_real_t` (used for every
// VQF member, including its in-memory layout) based on whether VQF_SINGLE_PRECISION is
// defined *at the point vqf.hpp is included* -- defaulting to `double` otherwise. Defining it
// here, right before the only include of the vendored header in this adapter's public header,
// makes every translation unit that includes this file (the adapter's .cpp, and its tests)
// see `vqf_real_t == float`, matching this repository's float-based AhrsFilterBase. This must
// stay consistent with third_party/vqf/vqf_impl.cpp, which defines the same macro before
// compiling the vendored implementation itself (see that file for why both sides need to agree).
#ifndef VQF_SINGLE_PRECISION
#define VQF_SINGLE_PRECISION
#endif
#include "third_party/vqf/vqf.hpp"

namespace m5 {
namespace imu {

/*!
  @class VQFFilter
  @brief Adapter around the vendored VQF algorithm (Daniel Laidig, MIT license; see
  src/imu/filter/third_party/vqf)
  @details The vendored code is the full C++ reference implementation of the VQF algorithm
  described in D. Laidig and T. Seel, "VQF: Highly Accurate IMU Orientation Estimation with
  Bias Estimation and Magnetic Disturbance Rejection", Information Fusion 2023 -- i.e. it
  includes the three mechanisms the paper is named after, each individually switchable here.
  The two bias estimators are on by default; magnetic disturbance rejection is off (see
  writeMagDisturbanceRejection() for why this one deviates from upstream):

  - **Rest bias estimation** (writeRestBiasEstimation()): while the device is detected as
    resting, the gyroscope output *is* its bias, so it can be measured and removed directly.
    This is the dominant defence against 6-axis yaw drift.
  - **Motion bias estimation** (writeMotionBiasEstimation()): estimates the gyroscope bias
    while the device is moving, which a stillness-based scheme (AhrsController's
    writeAutoCalibration()) cannot do by construction.
  - **Magnetic disturbance rejection** (writeMagDisturbanceRejection()): detects when the
    magnetic field's magnitude/dip angle departs from the learned undisturbed reference and
    stops trusting the magnetometer for heading until the field settles again.

  Turning all three off makes this filter equivalent to upstream's "basic" variant, which is
  what this adapter used to vendor.

  This adapter performs no re-derivation of the underlying math (like FusionFilter, and unlike
  ComplementaryFilter/MadgwickFilter/MahonyFilter/EKFFilter); its job is limited to the
  AhrsFilterBase <-> VQF API boundary:

  - Units: VQF's `update()` takes gyroscope in **rad/s** and accelerometer in
    **m/s^2** (`mag` is documented as "arbitrary units" and is passed through unscaled), which
    differs from this repository's dps/G unit contract (filter_base.hpp) -- update() converts
    gyro (dps -> rad/s, x pi/180) and accel (G -> m/s^2, x standard gravity) accordingly.
  - Earth axes convention: VQF's world/earth frame is Z-up like this repository's (a
    static, level unit's accelerometer reading is rotated toward (0,0,+1) by the inclination
    correction in VQF::updateAcc(), the same sign as filter_base.hpp), so no flip is
    needed there. Its magnetometer heading reference, however, converges the *horizontal*
    mag component onto world **+Y**, not +X (see VQF::updateMag()'s
    `atan2(magEarth[0], magEarth[1])`, i.e. atan2(x, y) rather than this repository's
    atan2(y, x)-style convention) -- a fixed +90 degree yaw offset from this repository's mag
    table (filter_base.hpp: mag horizontal along +X at yaw = 0). quaternion() corrects this
    by composing a fixed world-Z rotation of -90 degrees on top of getQuat9D() once any mag
    sample has been consumed (`_mag_seen`); a pure heading rotation, so roll/pitch (which
    already agree with this repository) are untouched. In the mag-less case no correction is
    applied (delta stays 0 and the 6D quaternion already matches the shared convention);
    reset() clears `_mag_seen` along with VQF's delta; a sample-period rebuild keeps both,
    since the state is carried across (see below).
  - Sample period: VQF is constructed with a fixed nominal sampling period (`gyrTs`)
    and has no public API to change it afterwards (`VQF::setup()`, which derives the
    internal filter coefficients from it, is only called from the constructors). Since
    AhrsFilterBase::update() receives an explicit, potentially-varying `dt` every call, this
    adapter tracks the `dt` the underlying VQF was last (re)built with and, when a new
    `dt` differs from it by more than 20%, reconstructs a fresh VQF for the new period,
    carrying over both the tuning parameters (see writeTauAcc/writeTauMag) and the filter state
    (getState()/setState()). Preserving the state is essential rather than merely nice: rest
    detection has to accumulate 1.5s uninterrupted before it arms, and it shares the state
    struct with the gyro bias estimate, so a caller whose loop drains a varying number of
    samples per update() -- which makes the per-sample dt jitter past the threshold repeatedly
    -- would otherwise never get any bias estimation at all. The low-pass histories inside the
    state were built with the previous coefficients and upstream offers no way to resample
    them, so this is an approximation, but a much smaller one than restarting the estimators.
  - VQF has no public API to overwrite the attitude estimate directly (unlike
    Fusion/Complementary/Madgwick/Mahony's optional `writeQuaternion()`): `VQF::setState()`
    only accepts a full `VQFState`, whose `gyrQuat`/`accQuat`/`delta` split has no
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
      @brief Set the accelerometer low-pass time constant (VQFParams::tauAcc)
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
      @brief Set the magnetometer heading correction time constant (VQFParams::tauMag)
      @param tau_mag Time constant in seconds, must be > 0
      @return False (and keeps the previous value) if tau_mag is not positive
     */
    bool writeTauMag(const float tau_mag);
    //! @brief Current magnetometer heading correction time constant
    inline float readTauMag() const
    {
        return static_cast<float>(_vqf->getParams().tauMag);
    }

    /*!
      @brief Enable/disable gyroscope bias estimation while the device is at rest
      @param enabled True to estimate (the default)
      @details While rest is detected the gyroscope output is its own bias, so it can be
      measured directly. This is the strongest defence against 6-axis yaw drift; disabling it
      leaves the bias to accumulate into the heading unchecked.
     */
    void writeRestBiasEstimation(const bool enabled);
    //! @brief Whether rest bias estimation is enabled
    inline bool readRestBiasEstimation() const
    {
        return _vqf->getParams().restBiasEstEnabled;
    }

    /*!
      @brief Enable/disable gyroscope bias estimation while the device is moving
      @param enabled True to estimate (the default)
      @details Complements rest bias estimation for devices that are rarely still -- something
      AhrsController::writeAutoCalibration() cannot provide, since it only adapts during
      stillness.
     */
    void writeMotionBiasEstimation(const bool enabled);
    //! @brief Whether motion bias estimation is enabled
    inline bool readMotionBiasEstimation() const
    {
        return _vqf->getParams().motionBiasEstEnabled;
    }

    /*!
      @brief Enable/disable magnetic disturbance detection and rejection
      @param enabled True to reject disturbances
      @details Compares the measured field's magnitude and dip angle against a learned
      undisturbed reference; while they disagree the magnetometer is ignored for heading and
      the estimate coasts on the gyroscope, so nearby steel or motors cannot yank the yaw
      around. Has no effect on a 6-axis setup (no mag samples to reject).
      @note **Off by default here, unlike upstream.** The protection only exists once a
      reference has been adopted, and upstream only adopts one after the low-passed gyroscope
      has held above 20 deg/s for five continuous seconds (VQFParams::magNewMinGyr /
      magNewFirstTime) with a consistent field -- proof that the field was observed from
      several orientations. A device sitting on a desk never satisfies that, so it stays
      permanently "disturbed": the magnetometer is dropped entirely for the first minute
      (magMaxRejectionTime) and then only applied at half gain (magRejectionFactor), which is
      strictly worse than not rejecting at all -- weaker heading correction and still no
      protection, since there is no reference to compare against. Enable it for devices that
      genuinely move around (wearables, robots), where a reference does get established.
     */
    void writeMagDisturbanceRejection(const bool enabled);
    //! @brief Whether magnetic disturbance rejection is enabled
    inline bool readMagDisturbanceRejection() const
    {
        return _vqf->getParams().magDistRejectionEnabled;
    }

    /*!
      @brief Set the largest gyroscope bias the estimator will accept (VQFParams::biasClip)
      @param bias_clip_dps Maximum expected bias magnitude in dps, must be > 0
      @return False (and keeps the previous value) if bias_clip_dps is not positive
      @warning **This silently gates rest detection, not just the estimate.** Upstream clips the
      bias estimate to this value *and* refuses to call a sample "at rest" when any low-passed
      gyro axis exceeds it (so that a slow but steady rotation is not mistaken for bias). A
      sensor whose real bias is larger than this therefore never triggers rest detection at all,
      and its bias is never estimated -- the drift stays. Upstream's default is 2 dps, which
      several M5 IMUs (the MPU6886's zero-rate output is specified around +-5 dps) can exceed;
      either raise this or remove the bulk of the bias up front with
      AhrsController::calibrateGyro().
      @note Raising it is not free: the larger the window, the more likely a genuine slow
      rotation (a level device turning steadily about yaw, where the accelerometer never moves)
      is absorbed as bias.
      @note VQF only accepts this through its constructor, so the underlying instance is rebuilt
      here; the filter state is carried across, as with a sample-period change.
     */
    bool writeBiasClip(const float bias_clip_dps);
    //! @brief Current maximum expected gyroscope bias (dps)
    inline float readBiasClip() const
    {
        return static_cast<float>(_vqf->getParams().biasClip);
    }

    //! @brief Current estimated gyro bias (dps, body frame); zero unless bias estimation is enabled
    m5::math::Vector3 readGyroBias() const;

    //! @brief Whether the device is currently detected as resting (see writeRestBiasEstimation())
    inline bool restDetected() const
    {
        return _vqf->getRestDetected();
    }

    /*!
      @brief The magnetic field as VQF currently measures it, in earth frame
      @param[out] norm Field magnitude, in whatever unit the magnetometer supplies (uT here)
      @param[out] dip_deg Dip (inclination) angle in degrees: how far the field tilts out of the
      horizontal plane, derived from the field rotated into the earth frame by the current
      accel/gyro attitude
      @details Only updated while writeMagDisturbanceRejection() is enabled, since that is the
      only consumer. Useful as an axis-consistency check: the magnitude is invariant under any
      rotation, but the dip is only stable if the magnetometer's axes agree with the
      accelerometer/gyroscope's. Rotating a device about the vertical must leave the dip
      unchanged -- if it swings instead, the magnetometer needs an axis remap
      (AhrsController::writeMagAxisOrder()), and VQF will never adopt a reference because the
      dip cannot stay inside VQFParams::magDipTh.
     */
    inline void readMagNormDip(float& norm, float& dip_deg) const
    {
        const VQFState& state = _vqf->getState();
        norm                  = static_cast<float>(state.magNormDip[0]);
        dip_deg               = static_cast<float>(state.magNormDip[1]) * (180.0f / 3.14159265358979323846f);
    }

    /*!
      @brief How close the last sample was to being accepted as "at rest"
      @param[out] gyro_relative Gyroscope deviation relative to its threshold
      @param[out] accel_relative Accelerometer deviation relative to its threshold
      @details Both have to stay below 1.0 for restDetected() to arm (and stay armed). Reading
      them tells you *which* sensor is keeping a device that looks stationary from resting --
      the thresholds themselves are VQFParams::restThGyr / restThAcc.
     */
    inline void readRestDeviations(float& gyro_relative, float& accel_relative) const
    {
        vqf_real_t out[2]{};
        _vqf->getRelativeRestDeviations(out);
        gyro_relative  = static_cast<float>(out[0]);
        accel_relative = static_cast<float>(out[1]);
    }

    /*!
      @brief Whether a magnetic disturbance is currently detected
      @details Only meaningful while writeMagDisturbanceRejection() is enabled; with rejection
      off the detector never runs and this keeps the value reset() left it at (true).
      @note This also reads true on a device that never moves: VQF refuses to adopt a magnetic
      reference until the field has been seen from several orientations (its magNewMinGyr
      threshold is 20 deg/s), so a permanently stationary 9-axis setup stays "disturbed" and
      leans on the gyroscope for heading after the initial convergence.
     */
    inline bool magDisturbanceDetected() const
    {
        return _vqf->getMagDistDetected();
    }

private:
    //! @brief (Re)build _vqf for the given dt if none exists yet, or if dt has drifted from
    //! the period it was last built with by more than VQF_RESAMPLE_THRESHOLD_RATIO
    void ensure_sample_period(const float dt);

    std::unique_ptr<::VQF> _vqf{};  // basicvqf's filter instance (see VQFFilter()/ensure_sample_period())
    float _built_dt{};              // Sample period _vqf was (re)built with, in seconds
    bool _mag_seen{};               // Any mag sample consumed? Gates the heading convention fix
};

}  // namespace imu
}  // namespace m5
#endif
