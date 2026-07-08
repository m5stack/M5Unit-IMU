/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6886.cpp
  @brief UnitMPU6886 (6-axis IMU) unit component
*/
#include "unit_mpu6886.hpp"
#include <M5Utility.hpp>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::mpu6886;
using namespace m5::unit::mpu6886::command;
using m5::imu::Capability;

namespace {
constexpr uint32_t WRITE_VERIFY_RETRY{16};
}  // namespace

namespace m5 {
namespace unit {

// class UnitMPU6886
const char UnitMPU6886::name[] = "UnitMPU6886";
const types::uid_t UnitMPU6886::uid{"UnitMPU6886"_mmh3};
const types::attr_t UnitMPU6886::attr{attribute::AccessI2C};

UnitMPU6886::UnitMPU6886(const uint8_t addr)
    : Component(addr), _data{new m5::container::CircularBuffer<mpu6886::Data>(_cfg.stored_size)}
{
    auto ccfg        = component_config();
    ccfg.clock       = 400 * 1000U;
    ccfg.stored_size = _cfg.stored_size;
    component_config(ccfg);
}

bool UnitMPU6886::begin()
{
    if (_cfg.stored_size != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<mpu6886::Data>(_cfg.stored_size));
    }
    if (!_data) {
        M5_LIB_LOGE("Failed to allocate");
        return false;
    }
    _data->clear();
    _pushed = _consumed = 0;

    uint8_t id{};
    if (!readRegister8(WHO_AM_I, id, 0) || id != WHOAMI_VALUE) {
        M5_LIB_LOGE("Not MPU6886 %02X", id);
        return false;
    }
    if (!writeRegister8(PWR_MGMT_1, DEVICE_RESET)) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }
    m5::utility::delay(10);

    if (!write_register8_verified(PWR_MGMT_1, CLKSEL_AUTO_PLL) || !writeAccelRange(_cfg.accel_range) ||
        !writeGyroRange(_cfg.gyro_range) || !write_register8_verified(CONFIG, static_cast<uint8_t>(_cfg.dlpf)) ||
        !write_register8_verified(SMPLRT_DIV, _cfg.sample_rate_div) || !write_register8_verified(INT_PIN_CFG, 0xC0) ||
        !write_register8_verified(INT_ENABLE, 0x00) || !write_register8_verified(ACCEL_CONFIG2, 0x00) ||
        !write_register8_verified(USER_CTRL, 0x00) || !write_register8_verified(FIFO_EN, 0x00)) {
        M5_LIB_LOGE("Failed to initialize");
        return false;
    }
    m5::utility::delay(50);  // Wait for gyro (typ 35ms) / accel (typ 10ms) startup

    return _cfg.start_periodic ? startPeriodicMeasurement() : true;
}

void UnitMPU6886::update(const bool force)
{
    _updated = false;
    if (inPeriodic()) {
        const auto at = m5::utility::millis();
        if (force || !_latest || at >= _latest + _interval) {
            mpu6886::Data d{};
            _updated = read_measurement(d);
            if (_updated) {
                _data->push_back(d);
                ++_pushed;
                _latest = at;
            }
        }
    }
}

bool UnitMPU6886::start_periodic_measurement()
{
    if (inPeriodic()) {
        return false;
    }
    _periodic = true;
    _latest   = 0;
    _interval = 1 + _cfg.sample_rate_div;
    return true;
}

bool UnitMPU6886::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

bool UnitMPU6886::startPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitMPU6886, mpu6886::Data>::startPeriodicMeasurement();
}

bool UnitMPU6886::stopPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitMPU6886, mpu6886::Data>::stopPeriodicMeasurement();
}

bool UnitMPU6886::writeAccelRange(const mpu6886::AccelRange range)
{
    if (write_register8_verified(ACCEL_CONFIG, static_cast<uint8_t>(range) << 3)) {
        _cfg.accel_range = range;
        return true;
    }
    return false;
}

bool UnitMPU6886::writeGyroRange(const mpu6886::GyroRange range)
{
    if (write_register8_verified(GYRO_CONFIG, static_cast<uint8_t>(range) << 3)) {
        _cfg.gyro_range = range;
        return true;
    }
    return false;
}

bool UnitMPU6886::read_measurement(mpu6886::Data& d)
{
    uint8_t status{};
    if (!readRegister8(INT_STATUS, status, 0) || !(status & 0x01)) {
        return false;
    }
    return readRegister(ACCEL_XOUT_H, d.raw.data(), d.raw.size(), 0);
}

bool UnitMPU6886::write_register8_verified(const uint8_t reg, const uint8_t val)
{
    for (uint32_t i = 0; i < WRITE_VERIFY_RETRY; ++i) {
        uint8_t readback{};
        if (writeRegister8(reg, val) && readRegister8(reg, readback, 0) && readback == val) {
            return true;
        }
        m5::utility::delay(1);
    }
    M5_LIB_LOGE("Failed to write and verify register:%02X val:%02X", reg, val);
    return false;
}

Capability UnitMPU6886::readImuRaw(m5::imu::imu_raw_data_t& out)
{
    if (!_data || _data->empty() || _consumed >= _pushed) {
        return Capability::None;
    }

    uint32_t lag = _pushed - _consumed;
    if (lag > _data->size()) {
        lag       = _data->size();
        _consumed = _pushed - lag;
    }

    const auto d = _data->at(_data->size() - lag);
    if (!d) {
        return Capability::None;
    }
    ++_consumed;

    out.accel[0] = d->accel_x();
    out.accel[1] = d->accel_y();
    out.accel[2] = d->accel_z();
    out.gyro[0]  = d->gyro_x();
    out.gyro[1]  = d->gyro_y();
    out.gyro[2]  = d->gyro_z();
    out.temp     = d->temp_adc();
    return imuSpec();
}

void UnitMPU6886::imuConvertParam(m5::imu::imu_convert_param_t& out) const
{
    out.accel_res   = accel_res(_cfg.accel_range);
    out.gyro_res    = gyro_res(_cfg.gyro_range);
    out.temp_res    = 1.0f / 326.8f;
    out.temp_offset = 25.0f;
}

}  // namespace unit
}  // namespace m5
