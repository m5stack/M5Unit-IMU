/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file vqf.cpp
  @brief Adapter around the vendored VQF (Versatile Quaternion-based Filter) algorithm
*/
#include "vqf.hpp"

#include <cmath>

namespace m5 {
namespace imu {

namespace {
constexpr float VQF_PI{3.14159265358979323846f};
constexpr float VQF_DEG2RAD{VQF_PI / 180.0f};
constexpr float VQF_RAD2DEG{180.0f / VQF_PI};
//! @brief Standard gravity, for converting accel from G (this repository's unit) to m/s^2 (VQF's unit)
constexpr float VQF_STANDARD_GRAVITY{9.80665f};
//! @brief Nominal sample period VQF is first constructed with, in seconds (100 Hz)
constexpr float VQF_DEFAULT_SAMPLE_PERIOD_S{0.01f};
//! @brief Relative drift from the currently-built sample period that triggers a rebuild (see vqf.hpp)
constexpr float VQF_RESAMPLE_THRESHOLD_RATIO{0.2f};
}  // namespace

VQFFilter::VQFFilter() : _built_dt(VQF_DEFAULT_SAMPLE_PERIOD_S)
{
    VQFParams params;
    // Deliberately off, unlike upstream (see writeMagDisturbanceRejection() for the reasoning):
    // rejection only protects anything once a magnetic reference has been adopted, and adopting one
    // needs several seconds of sustained rotation that a typical desk-bound setup never performs.
    // Until then it merely weakens the heading correction. The rebuild paths below carry the flag
    // over via getParams(), so this stays the default for the lifetime of the filter.
    params.magDistRejectionEnabled = false;
    _vqf.reset(new ::VQF(params, static_cast<vqf_real_t>(_built_dt)));
}

void VQFFilter::reset()
{
    // resetState() clears the attitude/heading state (gyrQuat/accQuat/delta and the acc/mag
    // low-pass histories) but leaves the current tauAcc/tauMag parameters and the sample
    // period _vqf was built with untouched -- matching AhrsFilterBase::reset()'s contract
    // ("reset the internal attitude estimate... to identity") without discarding tuning.
    _vqf->resetState();
    _mag_seen = false;  // delta is 0 again; the heading convention fix must wait for new mag data
}

void VQFFilter::update(const float dt, const float accel[3], const float gyro[3], const float* mag)
{
    if (dt <= 0.0f) {
        return;
    }
    ensure_sample_period(dt);
    _mag_seen = _mag_seen || (mag != nullptr);

    // Unit conversion: this repository's gyro (dps)/accel (G) -> VQF's rad/s and m/s^2
    // (see vqf.hpp for the source citation); mag is passed through unscaled ("arbitrary
    // units" per VQF, and this repository already supplies calibrated uT).
    const vqf_real_t gyr[3] = {
        static_cast<vqf_real_t>(gyro[0] * VQF_DEG2RAD),
        static_cast<vqf_real_t>(gyro[1] * VQF_DEG2RAD),
        static_cast<vqf_real_t>(gyro[2] * VQF_DEG2RAD),
    };
    const vqf_real_t acc[3] = {
        static_cast<vqf_real_t>(accel[0] * VQF_STANDARD_GRAVITY),
        static_cast<vqf_real_t>(accel[1] * VQF_STANDARD_GRAVITY),
        static_cast<vqf_real_t>(accel[2] * VQF_STANDARD_GRAVITY),
    };

    if (mag != nullptr) {
        const vqf_real_t m[3] = {
            static_cast<vqf_real_t>(mag[0]),
            static_cast<vqf_real_t>(mag[1]),
            static_cast<vqf_real_t>(mag[2]),
        };
        _vqf->update(gyr, acc, m);
    } else {
        _vqf->update(gyr, acc);
    }
}

void VQFFilter::quaternion(float& w, float& x, float& y, float& z) const
{
    // getQuat9D() applies the (possibly still-zero, if no mag sample has ever been supplied)
    // heading correction on top of the 6D quaternion; with delta == 0 that correction is the
    // identity rotation, so this is equivalent to getQuat6D() in the mag-less case.
    vqf_real_t q[4]{};
    _vqf->getQuat9D(q);
    const float qw = static_cast<float>(q[0]);
    const float qx = static_cast<float>(q[1]);
    const float qy = static_cast<float>(q[2]);
    const float qz = static_cast<float>(q[3]);

    if (!_mag_seen) {
        w = qw;
        x = qx;
        y = qy;
        z = qz;
        return;
    }
    // Convention fix (see vqf.hpp): VQF's heading correction converges the horizontal mag
    // component onto world +Y, this repository's contract puts it on +X -- a constant +90 degree
    // yaw offset once mag samples flow. Composing a fixed world-Z rotation of -90 degrees
    // (a pure heading rotation: roll/pitch are untouched) restores the shared convention.
    // R_z(-90deg) = (cos(-45deg), 0, 0, sin(-45deg)):
    constexpr float RZ_W{0.70710678f};
    constexpr float RZ_Z{-0.70710678f};
    w = RZ_W * qw - RZ_Z * qz;
    x = RZ_W * qx - RZ_Z * qy;
    y = RZ_W * qy + RZ_Z * qx;
    z = RZ_W * qz + RZ_Z * qw;
}

bool VQFFilter::writeTauAcc(const float tau_acc)
{
    if (!(tau_acc > 0.0f)) {
        return false;
    }
    _vqf->setTauAcc(static_cast<vqf_real_t>(tau_acc));
    return true;
}

bool VQFFilter::writeTauMag(const float tau_mag)
{
    if (!(tau_mag > 0.0f)) {
        return false;
    }
    _vqf->setTauMag(static_cast<vqf_real_t>(tau_mag));
    return true;
}

void VQFFilter::writeRestBiasEstimation(const bool enabled)
{
    _vqf->setRestBiasEstEnabled(enabled);
}

void VQFFilter::writeMotionBiasEstimation(const bool enabled)
{
    _vqf->setMotionBiasEstEnabled(enabled);
}

void VQFFilter::writeMagDisturbanceRejection(const bool enabled)
{
    _vqf->setMagDistRejectionEnabled(enabled);
}

bool VQFFilter::writeBiasClip(const float bias_clip_dps)
{
    if (!(bias_clip_dps > 0.0f)) {
        return false;
    }
    // VQFParams::biasClip is only read when the coefficients are derived, i.e. in VQF's
    // constructors -- there is no setter for it (unlike tauAcc/tauMag). Rebuilding is the only
    // way to honour a new value; see the warning on this function in vqf.hpp.
    VQFParams params      = _vqf->getParams();
    params.biasClip       = static_cast<vqf_real_t>(bias_clip_dps);
    const VQFState state  = _vqf->getState();
    _vqf.reset(new ::VQF(params, static_cast<vqf_real_t>(_built_dt)));
    _vqf->setState(state);  // keep the estimators armed, as ensure_sample_period() does
    return true;
}

m5::math::Vector3 VQFFilter::readGyroBias() const
{
    vqf_real_t bias[3]{};
    _vqf->getBiasEstimate(bias);
    // VQF works in rad/s (see update()); this repository's contract is dps
    return m5::math::Vector3(static_cast<float>(bias[0]) * VQF_RAD2DEG, static_cast<float>(bias[1]) * VQF_RAD2DEG,
                             static_cast<float>(bias[2]) * VQF_RAD2DEG);
}

void VQFFilter::ensure_sample_period(const float dt)
{
    const float diff_ratio = std::fabs(dt - _built_dt) / _built_dt;
    if (diff_ratio <= VQF_RESAMPLE_THRESHOLD_RATIO) {
        return;
    }
    // VQF's per-sample coefficients (VQF::setup()) are only computed from the constructors, with
    // no public API to recompute them for a new gyrTs afterwards -- so a sample period drift
    // beyond the threshold can only be honored by reconstructing the instance (see vqf.hpp).
    //
    // The state is carried across the rebuild. That matters far more than it looks: rest
    // detection only reports "at rest" after restMinT (1.5s) of uninterrupted accumulation, and
    // the gyro bias estimate lives in the same struct. A caller whose loop drains a varying
    // number of samples per update() feeds a dt that jitters by more than the threshold, so
    // rebuilds recur -- and dropping the state each time would keep rest detection from ever
    // arming, silently disabling the bias estimation this filter is chosen for. The low-pass
    // histories inside the state were accumulated with the old coefficients, which upstream
    // offers no way to resample; reusing them is an approximation, but a far smaller error than
    // restarting the estimators.
    const VQFParams params = _vqf->getParams();
    const VQFState state   = _vqf->getState();
    _vqf.reset(new ::VQF(params, static_cast<vqf_real_t>(dt)));
    _vqf->setState(state);
    _built_dt = dt;
}

}  // namespace imu
}  // namespace m5
