/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file vqf.cpp
  @brief Adapter around the vendored basicvqf (Versatile Quaternion-based Filter) algorithm
*/
#include "vqf.hpp"

#include <cmath>

namespace m5 {
namespace imu {

namespace {
constexpr float VQF_PI{3.14159265358979323846f};
constexpr float VQF_DEG2RAD{VQF_PI / 180.0f};
//! @brief Standard gravity, for converting accel from G (this repository's unit) to m/s^2 (basicvqf's unit)
constexpr float VQF_STANDARD_GRAVITY{9.80665f};
//! @brief Nominal sample period BasicVQF is first constructed with, in seconds (100 Hz)
constexpr float VQF_DEFAULT_SAMPLE_PERIOD_S{0.01f};
//! @brief Relative drift from the currently-built sample period that triggers a rebuild (see vqf.hpp)
constexpr float VQF_RESAMPLE_THRESHOLD_RATIO{0.2f};
}  // namespace

VQFFilter::VQFFilter() : _built_dt(VQF_DEFAULT_SAMPLE_PERIOD_S)
{
    _vqf.reset(new ::BasicVQF(static_cast<vqf_real_t>(_built_dt)));
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
    ensureSamplePeriod(dt);
    _mag_seen = _mag_seen || (mag != nullptr);

    // Unit conversion: this repository's gyro (dps)/accel (G) -> basicvqf's rad/s and m/s^2
    // (see vqf.hpp for the source citation); mag is passed through unscaled ("arbitrary
    // units" per basicvqf, and this repository already supplies calibrated uT).
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
    // Convention fix (see vqf.hpp): basicvqf's heading correction converges the horizontal mag
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

void VQFFilter::ensureSamplePeriod(const float dt)
{
    const float diff_ratio = std::fabs(dt - _built_dt) / _built_dt;
    if (diff_ratio <= VQF_RESAMPLE_THRESHOLD_RATIO) {
        return;
    }
    // BasicVQF's per-sample coefficients (BasicVQF::setup()) are only computed from the
    // constructors, with no public API to recompute them for a new gyrTs afterwards -- so a
    // sample period drift beyond the threshold can only be honored by reconstructing the
    // instance (see vqf.hpp). This carries over the current tauAcc/tauMag tuning parameters
    // (user-configured) but loses the accumulated attitude/heading/low-pass state, the same
    // trade-off resetState() makes deliberately, just triggered implicitly here instead.
    const BasicVQFParams params = _vqf->getParams();
    _vqf.reset(new ::BasicVQF(params, static_cast<vqf_real_t>(dt)));
    _built_dt = dt;
    _mag_seen = false;  // the rebuild dropped delta back to 0, same as reset()
}

}  // namespace imu
}  // namespace m5
