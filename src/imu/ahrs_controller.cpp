/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file ahrs_controller.cpp
  @brief Layer-2 AHRS controller: merges IMUDataSource capabilities and drives an AhrsFilterBase
*/
#include "ahrs_controller.hpp"

#include <algorithm>
#include <cmath>

#include <M5Utility.hpp>

#include <math/quaternion.hpp>

#include "filter/complementary.hpp"
#include "filter/ekf.hpp"
#include "filter/fusion.hpp"
#include "filter/madgwick.hpp"
#include "filter/mahony.hpp"
#include "filter/vqf.hpp"

namespace m5 {
namespace imu {

namespace {
// Ring-buffer depth this controller can drain from the gyro-owning source within a single
// update() call before extra samples are dropped (with a log warning). Generous relative to
// typical stored_size configs (~20 samples for a 200Hz ODR / 10Hz update() pairing).
constexpr size_t AHRS_MAX_DRAIN_SAMPLES{32};

// Exponential moving average weight applied to update()'s real-clock dt (see
// AhrsController::update()) so that ordinary micros() sample-to-sample jitter does not get
// fed straight into the filters -- VQF in particular rebuilds its internal state whenever a
// new dt differs from its currently built period by more than 20% (see vqf.hpp).
constexpr float AHRS_DT_EMA_ALPHA{0.1f};

//! @brief BMM150 reports -32768 on any saturated axis; the whole sample is unusable then
//! (see Plan 2 handoff notes / filter_base.hpp mag contract).
constexpr int32_t MAG_OVERFLOW_SENTINEL{-32768};

// Auto-calibration parameters (M5Unified IMU_Class::_update_convert_param(), ported from
// raw<<16 fixed point to physical units). radius/tolerance
// are the expected magnitude of (sample - offset) at rest and the tolerance band around it;
// noise is the per-sample movement threshold below which stillness accumulates.
//! @note accel/gyro thresholds below are moved from M5Unified as physical quantity values.
constexpr float ACCEL_CAL_RADIUS_G{1.0f};
constexpr float ACCEL_CAL_TOLERANCE_G{1.0f / 2048.0f};
constexpr float ACCEL_CAL_NOISE_G{0.0625f};
constexpr uint8_t ACCEL_CAL_SHIFT{1};

constexpr float GYRO_CAL_RADIUS_DPS{0.0f};
constexpr float GYRO_CAL_TOLERANCE_DPS{0.0f};
constexpr float GYRO_CAL_NOISE_DPS{2.0f};
constexpr uint8_t GYRO_CAL_SHIFT{6};

//! @note Mag calibration thresholds are retune values: M5Unified's raw-based constants
//! (384/64/96) converted to physical units using this library's mag scale (16 LSB/µT).
//! Conversion: 384×(1/16)=24µT, 64×(1/16)=4µT, 96×(1/16)=6µT.
//! These values reasonably cover Earth's magnetic field range (25–65µT).
//! WARNING: Unlike accel/gyro thresholds, these magnetometer values are NOT numerically
//! verified during auto-calibration convergence testing (only accel/gyro are verified).
constexpr float MAG_CAL_RADIUS_UT{24.0f};
constexpr float MAG_CAL_TOLERANCE_UT{4.0f};
constexpr float MAG_CAL_NOISE_UT{6.0f};
constexpr uint8_t MAG_CAL_SHIFT{1};

// M5Unified triggers calibration() once updateStillness()'s score exceeds this.
constexpr uint8_t CAL_STILLNESS_THRESHOLD{16};

// Guards calibrate()'s division by distance; below this the sample is considered coincident
// with the current offset estimate and no adjustment is meaningful.
constexpr float CAL_MIN_DISTANCE{1.0e-4f};

constexpr uint32_t CALIBRATION_DATA_VERSION{1};
}  // namespace

AhrsController::AhrsController()
{
    _accel_cal.radius          = ACCEL_CAL_RADIUS_G;
    _accel_cal.tolerance       = ACCEL_CAL_TOLERANCE_G;
    _accel_cal.noise           = ACCEL_CAL_NOISE_G;
    _accel_cal.average_shifter = ACCEL_CAL_SHIFT;

    _gyro_cal.radius          = GYRO_CAL_RADIUS_DPS;
    _gyro_cal.tolerance       = GYRO_CAL_TOLERANCE_DPS;
    _gyro_cal.noise           = GYRO_CAL_NOISE_DPS;
    _gyro_cal.average_shifter = GYRO_CAL_SHIFT;

    _mag_cal.radius          = MAG_CAL_RADIUS_UT;
    _mag_cal.tolerance       = MAG_CAL_TOLERANCE_UT;
    _mag_cal.noise           = MAG_CAL_NOISE_UT;
    _mag_cal.average_shifter = MAG_CAL_SHIFT;
}

AhrsController::~AhrsController()
{
    delete _owned_filter;
}

bool AhrsController::add(IMUDataSource& src)
{
    if (_source_count >= MAX_SOURCES) {
        M5_LIB_LOGE("AhrsController::add: already holding the maximum of %zu sources", MAX_SOURCES);
        return false;
    }
    _sources[_source_count++] = &src;

    const Capability cap = src.imuSpec();
    if (any(cap & Capability::Accel)) {
        if (_accel_src) {
            M5_LIB_LOGI("AhrsController::add: Accel capability already claimed, ignoring duplicate");
        } else {
            _accel_src = &src;
        }
    }
    if (any(cap & Capability::Gyro)) {
        if (_gyro_src) {
            M5_LIB_LOGI("AhrsController::add: Gyro capability already claimed, ignoring duplicate");
        } else {
            _gyro_src = &src;
        }
    }
    if (any(cap & Capability::Mag)) {
        if (_mag_src) {
            M5_LIB_LOGI("AhrsController::add: Mag capability already claimed, ignoring duplicate");
        } else {
            _mag_src = &src;
        }
    }
    if (any(cap & Capability::Temp)) {
        if (_temp_src) {
            M5_LIB_LOGI("AhrsController::add: Temp capability already claimed, ignoring duplicate");
        } else {
            _temp_src = &src;
        }
    }
    return true;
}

bool AhrsController::find_source_index(const IMUDataSource& src, size_t& out_index) const
{
    for (size_t i = 0; i < _source_count; ++i) {
        if (_sources[i] == &src) {
            out_index = i;
            return true;
        }
    }
    return false;
}

bool AhrsController::writeAxisOrder(const IMUDataSource& src, const AxisOrder& order)
{
    size_t index{};
    if (!find_source_index(src, index)) {
        M5_LIB_LOGE("AhrsController::writeAxisOrder: source was never add()ed");
        return false;
    }
    _axis_orders[index] = order;
    return true;
}

bool AhrsController::readAxisOrder(const IMUDataSource& src, AxisOrder& out) const
{
    size_t index{};
    if (!find_source_index(src, index)) {
        return false;
    }
    out = _axis_orders[index];
    return true;
}

bool AhrsController::writeMagAxisOrder(const IMUDataSource& src, const AxisOrder& order)
{
    size_t index{};
    if (!find_source_index(src, index)) {
        M5_LIB_LOGE("AhrsController::writeMagAxisOrder: source was never add()ed");
        return false;
    }
    _mag_axis_orders[index] = order;
    return true;
}

bool AhrsController::readMagAxisOrder(const IMUDataSource& src, AxisOrder& out) const
{
    size_t index{};
    if (!find_source_index(src, index)) {
        return false;
    }
    out = _mag_axis_orders[index];
    return true;
}

void AhrsController::writeAutoCalibration(const uint8_t accel, const uint8_t gyro, const uint8_t mag)
{
    _accel_cal.strength = accel;
    _gyro_cal.strength  = gyro;
    _mag_cal.strength   = mag;
}

void AhrsController::readAutoCalibration(uint8_t& accel, uint8_t& gyro, uint8_t& mag) const
{
    accel = _accel_cal.strength;
    gyro  = _gyro_cal.strength;
    mag   = _mag_cal.strength;
}

void AhrsController::begin_gyro_collect()
{
    _gyro_collecting     = true;
    _gyro_collect_sum[0] = 0.0f;
    _gyro_collect_sum[1] = 0.0f;
    _gyro_collect_sum[2] = 0.0f;
    _gyro_collect_count  = 0;
}

bool AhrsController::end_gyro_collect()
{
    _gyro_collecting = false;
    if (_gyro_collect_count == 0) {
        return false;
    }
    const float inv     = 1.0f / static_cast<float>(_gyro_collect_count);
    _gyro_cal.offset[0] = _gyro_collect_sum[0] * inv;
    _gyro_cal.offset[1] = _gyro_collect_sum[1] * inv;
    _gyro_cal.offset[2] = _gyro_collect_sum[2] * inv;
    return true;
}

bool AhrsController::calibrateGyro(const uint32_t duration_ms)
{
    if (!_gyro_src) {
        M5_LIB_LOGE("AhrsController::calibrateGyro: no source has claimed Gyro");
        return false;
    }
    begin_gyro_collect();
    const unsigned long start = m5::utility::millis();
    do {
        update();
    } while (m5::utility::millis() - start < duration_ms);
    return end_gyro_collect();
}

bool AhrsController::calibrateGyro(const uint32_t update_count, const float dt)
{
    if (!_gyro_src) {
        M5_LIB_LOGE("AhrsController::calibrateGyro: no source has claimed Gyro");
        return false;
    }
    begin_gyro_collect();
    for (uint32_t i = 0; i < update_count; ++i) {
        update(dt);
    }
    return end_gyro_collect();
}

bool AhrsController::readCalibration(calibration_data_t& out) const
{
    out.version = CALIBRATION_DATA_VERSION;
    for (int i = 0; i < 3; ++i) {
        out.accel_offset[i] = _accel_cal.offset[i];
        out.gyro_offset[i]  = _gyro_cal.offset[i];
        out.mag_offset[i]   = _mag_cal.offset[i];
    }
    return true;
}

bool AhrsController::writeCalibration(const calibration_data_t& data)
{
    if (data.version != CALIBRATION_DATA_VERSION) {
        M5_LIB_LOGE("AhrsController::writeCalibration: unsupported version %u",
                    static_cast<unsigned int>(data.version));
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        _accel_cal.offset[i] = data.accel_offset[i];
        _gyro_cal.offset[i]  = data.gyro_offset[i];
        _mag_cal.offset[i]   = data.mag_offset[i];
    }
    return true;
}

bool AhrsController::begin()
{
    if (!_accel_src || !_gyro_src) {
        return false;
    }
    if (!_filter) {
        return selectFilter(FilterType::Madgwick);
    }
    return true;
}

bool AhrsController::selectFilter(const FilterType type)
{
    AhrsFilterBase* created{};
    switch (type) {
        case FilterType::Complementary:
            created = new ComplementaryFilter();
            break;
        case FilterType::Madgwick:
            created = new MadgwickFilter();
            break;
        case FilterType::Mahony:
            created = new MahonyFilter();
            break;
        case FilterType::EKF:
            created = new EKFFilter();
            break;
        case FilterType::Fusion:
            created = new FusionFilter();
            break;
        case FilterType::VQF:
            created = new VQFFilter();
            break;
    }
    if (!created) {
        return false;
    }
    delete _owned_filter;
    _owned_filter = created;
    _filter       = _owned_filter;
    _filter->reset();
    return true;
}

void AhrsController::setFilter(AhrsFilterBase& filter)
{
    delete _owned_filter;
    _owned_filter = nullptr;
    _filter       = &filter;
}

// Composes _yaw_tare onto the filter's raw quaternion -- the single place that applies the tare
// offset, so quaternion() and euler() (which is derived from this) can never disagree about it.
void AhrsController::quaternion(float& w, float& x, float& y, float& z) const
{
    if (_filter) {
        _filter->quaternion(w, x, y, z);
        const m5::math::Quaternion composed = _yaw_tare * m5::math::Quaternion(w, x, y, z);
        w                                   = composed.w;
        x                                   = composed.x;
        y                                   = composed.y;
        z                                   = composed.z;
        return;
    }
    w = 1.0f;
    x = y = z = 0.0f;
}

void AhrsController::euler(float& roll_deg, float& pitch_deg, float& yaw_deg) const
{
    float w{1.0f}, x{}, y{}, z{};
    quaternion(w, x, y, z);
    m5::math::Quaternion(w, x, y, z).toEuler(roll_deg, pitch_deg, yaw_deg);
}

void AhrsController::tareYaw()
{
    if (!_filter) {
        return;
    }
    // Sample the filter's raw (untared) quaternion -- not quaternion()'s already-composed
    // output -- so a repeated tareYaw() call re-zeros to the filter's current heading instead of
    // drifting back toward it (composing the correction onto an already-corrected sample would
    // partially cancel a previous tareYaw()).
    float w{1.0f}, x{}, y{}, z{};
    _filter->quaternion(w, x, y, z);
    float roll{}, pitch{}, yaw{};
    m5::math::Quaternion(w, x, y, z).toEuler(roll, pitch, yaw);
    _yaw_tare = m5::math::Quaternion::fromEuler(0.0f, 0.0f, -yaw);
}

void AhrsController::clearYawTare()
{
    _yaw_tare = m5::math::Quaternion::identity();
}

bool AhrsController::is_mag_overflow(const int32_t mag[3])
{
    return mag[0] == MAG_OVERFLOW_SENTINEL || mag[1] == MAG_OVERFLOW_SENTINEL || mag[2] == MAG_OVERFLOW_SENTINEL;
}

// M5Unified IMU_Class::offset_point_t::updateStillness(), ported from raw<<16 fixed point to
// physical-unit floats. Tracks how little `sample` has moved relative to both the previous
// sample and a slow moving average; the returned score saturates toward 255 while genuinely
// stationary and collapses back toward 0 the moment either diff exceeds the noise threshold.
uint8_t AhrsController::axis_offset_state_t::update_stillness(const float sample[3])
{
    float maxdiff{};
    for (int i = 0; i < 3; ++i) {
        const float diff_prev = sample[i] - prev[i];
        prev[i]               = sample[i];
        maxdiff               = std::max(maxdiff, std::fabs(diff_prev));

        const float diff_avg = sample[i] - avg[i];
        maxdiff              = std::max(maxdiff, std::fabs(diff_avg));
        avg[i] += diff_avg / static_cast<float>(1 << average_shifter);
    }

    uint16_t score{};
    if (noise > 0.0f && maxdiff < noise) {
        score = static_cast<uint16_t>(255.0f * (noise - maxdiff) / noise);
    }
    const uint16_t candidate = std::min<uint16_t>(static_cast<uint16_t>(stillness) + 1, score);
    stillness                = static_cast<uint8_t>(candidate);
    return stillness;
}

// M5Unified IMU_Class::offset_point_t::calibration(), ported the same way. Nudges `offset`
// so that the distance between the last sample (`prev`) and `offset` approaches `radius`
// (0 for gyro: pull the offset toward the raw sample itself; non-zero for accel/mag: fit a
// sphere of that radius around the offset), scaled by how confident this update is
// (stillness * strength) relative to their maximum (255 * 255).
void AhrsController::axis_offset_state_t::calibrate()
{
    if (stillness == 0 || strength == 0) {
        return;
    }

    float diffs[3];
    float distance_sq{};
    for (int i = 0; i < 3; ++i) {
        diffs[i] = prev[i] - offset[i];
        distance_sq += diffs[i] * diffs[i];
    }
    const float distance = std::sqrt(distance_sq);
    if (distance < CAL_MIN_DISTANCE) {
        return;
    }

    const float measure_error = distance - radius;
    float force               = std::fabs(measure_error);
    if (force <= tolerance) {
        return;
    }
    if (measure_error < 0.0f) {
        force = -force;
    }
    const float fk =
        force * (static_cast<float>(stillness) * static_cast<float>(strength)) / (distance * 255.0f * 255.0f);
    for (int i = 0; i < 3; ++i) {
        offset[i] += diffs[i] * fk;
    }
}

// Applies `cal`'s stillness/calibration bookkeeping (only while strength > 0) to a
// post-axis-remap physical sample, then subtracts the (possibly just-updated) offset in
// place -- this is the "subtract offset, feed filter/latest" step described in
// writeAutoCalibration().
void AhrsController::apply_offset_and_calibrate(axis_offset_state_t& cal, float v[3])
{
    if (cal.strength > 0) {
        const uint8_t stillness = cal.update_stillness(v);
        if (stillness > CAL_STILLNESS_THRESHOLD) {
            cal.calibrate();
        }
    }
    v[0] -= cal.offset[0];
    v[1] -= cal.offset[1];
    v[2] -= cal.offset[2];
}

Capability AhrsController::drain_and_fuse(const float dt)
{
    Capability updated = Capability::None;

    imu_convert_param_t params[MAX_SOURCES]{};
    for (size_t i = 0; i < _source_count; ++i) {
        _sources[i]->imuConvertParam(params[i]);
    }

    // Every source other than the gyro clock: drain fully now, "latest sample wins" for
    // whichever capability it owns.
    for (size_t i = 0; i < _source_count; ++i) {
        IMUDataSource* src = _sources[i];
        if (src == _gyro_src) {
            continue;
        }
        const imu_convert_param_t& param = params[i];
        const AxisOrder& order           = _axis_orders[i];
        const AxisOrder& mag_order       = _mag_axis_orders[i];
        imu_raw_data_t raw{};
        while (any(src->readImuRaw(raw))) {
            if (src == _accel_src) {
                float v[3] = {
                    raw.accel[0] * param.accel_res,
                    raw.accel[1] * param.accel_res,
                    raw.accel[2] * param.accel_res,
                };
                order.apply(v);
                apply_offset_and_calibrate(_accel_cal, v);
                _latest.accel[0] = v[0];
                _latest.accel[1] = v[1];
                _latest.accel[2] = v[2];
                updated |= Capability::Accel;
            }
            if (src == _mag_src) {
                if (!is_mag_overflow(raw.mag)) {
                    float v[3] = {
                        raw.mag[0] * param.mag_res,
                        raw.mag[1] * param.mag_res,
                        raw.mag[2] * param.mag_res,
                    };
                    mag_order.apply(v);
                    apply_offset_and_calibrate(_mag_cal, v);
                    _latest.mag[0] = v[0];
                    _latest.mag[1] = v[1];
                    _latest.mag[2] = v[2];
                    _has_valid_mag = true;
                    updated |= Capability::Mag;
                }
            }
            if (src == _temp_src) {
                _latest.temp = raw.temp * param.temp_res + param.temp_offset;
                updated |= Capability::Temp;
            }
        }
    }

    // The gyro-owning source is the primary clock: every sample it hands out drives one
    // filter->update() call, with dt split evenly across the batch drained this round.
    if (_gyro_src) {
        size_t gyro_index{};
        for (size_t i = 0; i < _source_count; ++i) {
            if (_sources[i] == _gyro_src) {
                gyro_index = i;
                break;
            }
        }
        const imu_convert_param_t& gyro_param = params[gyro_index];
        const AxisOrder& gyro_order           = _axis_orders[gyro_index];
        const AxisOrder& gyro_mag_order       = _mag_axis_orders[gyro_index];

        imu_raw_data_t gyro_buf[AHRS_MAX_DRAIN_SAMPLES]{};
        size_t gyro_count{};
        size_t dropped_count{};
        imu_raw_data_t raw{};
        while (any(_gyro_src->readImuRaw(raw))) {
            if (gyro_count < AHRS_MAX_DRAIN_SAMPLES) {
                gyro_buf[gyro_count++] = raw;
            } else {
                ++dropped_count;
            }
        }
        if (dropped_count > 0) {
            M5_LIB_LOGW("AhrsController::update: gyro drain overflow, dropped %zu sample(s)", dropped_count);
        }

        if (gyro_count > 0) {
            updated |= Capability::Gyro;
            const float dt_per_sample = dt / static_cast<float>(gyro_count);

            for (size_t k = 0; k < gyro_count; ++k) {
                const imu_raw_data_t& sample = gyro_buf[k];

                float gyro_v[3] = {
                    sample.gyro[0] * gyro_param.gyro_res,
                    sample.gyro[1] * gyro_param.gyro_res,
                    sample.gyro[2] * gyro_param.gyro_res,
                };
                gyro_order.apply(gyro_v);
                if (_gyro_collecting) {
                    _gyro_collect_sum[0] += gyro_v[0];
                    _gyro_collect_sum[1] += gyro_v[1];
                    _gyro_collect_sum[2] += gyro_v[2];
                    ++_gyro_collect_count;
                }
                apply_offset_and_calibrate(_gyro_cal, gyro_v);
                _latest.gyro[0] = gyro_v[0];
                _latest.gyro[1] = gyro_v[1];
                _latest.gyro[2] = gyro_v[2];

                if (_gyro_src == _accel_src) {
                    float accel_v[3] = {
                        sample.accel[0] * gyro_param.accel_res,
                        sample.accel[1] * gyro_param.accel_res,
                        sample.accel[2] * gyro_param.accel_res,
                    };
                    gyro_order.apply(accel_v);
                    apply_offset_and_calibrate(_accel_cal, accel_v);
                    _latest.accel[0] = accel_v[0];
                    _latest.accel[1] = accel_v[1];
                    _latest.accel[2] = accel_v[2];
                    updated |= Capability::Accel;
                }
                if (_gyro_src == _mag_src) {
                    if (!is_mag_overflow(sample.mag)) {
                        float mag_v[3] = {
                            sample.mag[0] * gyro_param.mag_res,
                            sample.mag[1] * gyro_param.mag_res,
                            sample.mag[2] * gyro_param.mag_res,
                        };
                        gyro_mag_order.apply(mag_v);
                        apply_offset_and_calibrate(_mag_cal, mag_v);
                        _latest.mag[0] = mag_v[0];
                        _latest.mag[1] = mag_v[1];
                        _latest.mag[2] = mag_v[2];
                        _has_valid_mag = true;
                        updated |= Capability::Mag;
                    }
                }
                if (_gyro_src == _temp_src) {
                    _latest.temp = sample.temp * gyro_param.temp_res + gyro_param.temp_offset;
                    updated |= Capability::Temp;
                }

                if (_filter) {
                    const float* mag_ptr = _has_valid_mag ? _latest.mag : nullptr;
                    _filter->update(dt_per_sample, _latest.accel, gyro_v, mag_ptr);
                }
            }
        }
    }

    return updated;
}

Capability AhrsController::update()
{
    const unsigned long now = m5::utility::micros();

    if (!_has_last_micros) {
        _last_micros     = now;
        _has_last_micros = true;
        return Capability::None;
    }

    const unsigned long elapsed_us = now - _last_micros;  // unsigned subtraction: wraparound-safe
    _last_micros                   = now;
    const float dt                 = static_cast<float>(elapsed_us) * 1.0e-6f;

    if (!_has_dt_ema) {
        _dt_ema     = dt;
        _has_dt_ema = true;
    } else {
        _dt_ema = AHRS_DT_EMA_ALPHA * dt + (1.0f - AHRS_DT_EMA_ALPHA) * _dt_ema;
    }

    return drain_and_fuse(_dt_ema);
}

Capability AhrsController::update(const float dt)
{
    return drain_and_fuse(dt);
}

}  // namespace imu
}  // namespace m5
