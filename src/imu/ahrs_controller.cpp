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
}  // namespace

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

void AhrsController::quaternion(float& w, float& x, float& y, float& z) const
{
    if (_filter) {
        _filter->quaternion(w, x, y, z);
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

bool AhrsController::is_mag_overflow(const int32_t mag[3])
{
    return mag[0] == MAG_OVERFLOW_SENTINEL || mag[1] == MAG_OVERFLOW_SENTINEL || mag[2] == MAG_OVERFLOW_SENTINEL;
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
        imu_raw_data_t raw{};
        while (any(src->readImuRaw(raw))) {
            if (src == _accel_src) {
                _latest.accel[0] = raw.accel[0] * param.accel_res;
                _latest.accel[1] = raw.accel[1] * param.accel_res;
                _latest.accel[2] = raw.accel[2] * param.accel_res;
                updated |= Capability::Accel;
            }
            if (src == _mag_src) {
                if (!is_mag_overflow(raw.mag)) {
                    _latest.mag[0] = raw.mag[0] * param.mag_res;
                    _latest.mag[1] = raw.mag[1] * param.mag_res;
                    _latest.mag[2] = raw.mag[2] * param.mag_res;
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

        imu_raw_data_t gyro_buf[AHRS_MAX_DRAIN_SAMPLES]{};
        size_t gyro_count{};
        imu_raw_data_t raw{};
        while (any(_gyro_src->readImuRaw(raw))) {
            if (gyro_count < AHRS_MAX_DRAIN_SAMPLES) {
                gyro_buf[gyro_count++] = raw;
            } else {
                M5_LIB_LOGW("AhrsController::update: gyro drain overflow, dropping a sample");
            }
        }

        if (gyro_count > 0) {
            updated |= Capability::Gyro;
            const float dt_per_sample = dt / static_cast<float>(gyro_count);

            for (size_t k = 0; k < gyro_count; ++k) {
                const imu_raw_data_t& sample = gyro_buf[k];

                const float gyro_phys[3] = {
                    sample.gyro[0] * gyro_param.gyro_res,
                    sample.gyro[1] * gyro_param.gyro_res,
                    sample.gyro[2] * gyro_param.gyro_res,
                };
                _latest.gyro[0] = gyro_phys[0];
                _latest.gyro[1] = gyro_phys[1];
                _latest.gyro[2] = gyro_phys[2];

                if (_gyro_src == _accel_src) {
                    _latest.accel[0] = sample.accel[0] * gyro_param.accel_res;
                    _latest.accel[1] = sample.accel[1] * gyro_param.accel_res;
                    _latest.accel[2] = sample.accel[2] * gyro_param.accel_res;
                    updated |= Capability::Accel;
                }
                if (_gyro_src == _mag_src) {
                    if (!is_mag_overflow(sample.mag)) {
                        _latest.mag[0] = sample.mag[0] * gyro_param.mag_res;
                        _latest.mag[1] = sample.mag[1] * gyro_param.mag_res;
                        _latest.mag[2] = sample.mag[2] * gyro_param.mag_res;
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
                    _filter->update(dt_per_sample, _latest.accel, gyro_phys, mag_ptr);
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
