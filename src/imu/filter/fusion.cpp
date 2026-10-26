/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file fusion.cpp
  @brief Adapter around the vendored x-io Fusion AHRS algorithm
*/
#include "fusion.hpp"

#include <math/quaternion.hpp>

namespace m5 {
namespace imu {

namespace {
//! @brief Default feedback gain, matching Fusion's own fusionAhrsDefaultSettings.gain
constexpr float FUSION_DEFAULT_GAIN{0.5f};
}  // namespace

FusionFilter::FusionFilter()
{
    _settings            = fusionAhrsDefaultSettings;
    _settings.convention = FusionConventionNwu;  // See fusion.hpp for why Nwu is the match
    _settings.gain       = FUSION_DEFAULT_GAIN;
    FusionAhrsInitialise(&_ahrs);
    FusionAhrsSetSettings(&_ahrs, &_settings);
}

void FusionFilter::reset()
{
    // Restart clears the quaternion/internal feedback state and re-arms the startup
    // gain ramp, but leaves _settings (and the AHRS' copy of them) untouched.
    FusionAhrsRestart(&_ahrs);
}

void FusionFilter::update(const float dt, const float accel[3], const float gyro[3], const float* mag)
{
    if (dt <= 0.0f) {
        return;
    }

    // Fusion is normally driven by a fixed FusionAhrsSettings.sampleRate, but exposes
    // FusionAhrsSetSamplePeriod() precisely so callers can supply the actual per-sample
    // dt instead -- which is what AhrsFilterBase::update() receives.
    FusionAhrsSetSamplePeriod(&_ahrs, dt);

    FusionVector g{};
    g.axis.x = gyro[0];
    g.axis.y = gyro[1];
    g.axis.z = gyro[2];

    FusionVector a{};
    a.axis.x = accel[0];
    a.axis.y = accel[1];
    a.axis.z = accel[2];

    if (mag != nullptr) {
        FusionVector m{};
        m.axis.x = mag[0];
        m.axis.y = mag[1];
        m.axis.z = mag[2];
        FusionAhrsUpdate(&_ahrs, g, a, m);
    } else {
        FusionAhrsUpdateNoMagnetometer(&_ahrs, g, a);
    }
}

void FusionFilter::quaternion(float& w, float& x, float& y, float& z) const
{
    const FusionQuaternion q = FusionAhrsGetQuaternion(&_ahrs);
    w                        = q.element.w;
    x                        = q.element.x;
    y                        = q.element.y;
    z                        = q.element.z;
}

bool FusionFilter::startupInProgress() const
{
    return FusionAhrsGetFlags(&_ahrs).startup;
}

bool FusionFilter::writeGain(const float gain)
{
    if (!(gain >= 0.0f)) {
        return false;
    }
    _settings.gain = gain;
    FusionAhrsSetSettings(&_ahrs, &_settings);
    return true;
}

bool FusionFilter::writeQuaternion(const float w, const float x, const float y, const float z)
{
    m5::math::Quaternion q(w, x, y, z);
    if (!q.normalize()) {
        return false;
    }
    FusionQuaternion fq{};
    fq.element.w = q.w;
    fq.element.x = q.x;
    fq.element.y = q.y;
    fq.element.z = q.z;
    FusionAhrsSetQuaternion(&_ahrs, fq);
    return true;
}

}  // namespace imu
}  // namespace m5
