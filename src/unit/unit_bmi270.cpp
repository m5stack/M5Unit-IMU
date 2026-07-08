/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_bmi270.cpp
  @brief UnitBMI270 (6-axis IMU) unit component
*/
#include "unit_bmi270.hpp"
#include <M5Utility.hpp>
#include <algorithm>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::bmi270;
using namespace m5::unit::bmi270::command;
using m5::imu::Capability;

namespace {
constexpr uint32_t WRITE_VERIFY_RETRY{16};
// PWR_CONF must read 0 within 16ms of soft reset (datasheet-cited retry count, 1ms steps)
constexpr uint32_t PWR_CONF_POLL_RETRY{16};
// INTERNAL_STATUS.message must reach init_ok (0x01) within ~20ms of INIT_CTRL=0x01 (1ms steps)
constexpr uint32_t INTERNAL_STATUS_POLL_RETRY{20};

// Chunk size (bytes) for the config blob burst write to INIT_DATA (0x5E).
// Rationale: AdapterI2C::WireImpl::writeWithTransaction() (m5_unit_component/adapter_i2c.cpp)
// buffers the register address byte plus the payload in a single Wire transaction before
// endTransmission() actually clocks it out, and arduino-esp32's TwoWire defaults
// I2C_BUFFER_LENGTH to 128 bytes (Wire.h). 64 bytes of payload (+1 register address byte =
// 65 bytes) stays well under that default regardless of which bus backend is active
// (SoftwareI2C bit-bangs a byte at a time and has no such limit), and divides the 8192 byte
// blob evenly (128 chunks) while staying word (2 byte) aligned for the INIT_ADDR_0/1 addressing
// scheme below.
constexpr size_t CONFIG_CHUNK_SIZE{64};

#include "unit_bmi270_config.inl"

}  // namespace

namespace m5 {
namespace unit {

// class UnitBMI270
const char UnitBMI270::name[] = "UnitBMI270";
const types::uid_t UnitBMI270::uid{"UnitBMI270"_mmh3};
const types::attr_t UnitBMI270::attr{attribute::AccessI2C};

UnitBMI270::UnitBMI270(const uint8_t addr)
    : Component(addr), _data{new m5::container::CircularBuffer<bmi270::Data>(_cfg.stored_size)}
{
    auto ccfg        = component_config();
    ccfg.clock       = 400 * 1000U;
    ccfg.stored_size = _cfg.stored_size;
    component_config(ccfg);
}

bool UnitBMI270::begin()
{
    if (_cfg.stored_size != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<bmi270::Data>(_cfg.stored_size));
    }
    if (!_data) {
        M5_LIB_LOGE("Failed to allocate");
        return false;
    }
    _data->clear();
    _pushed = _consumed = 0;
    _last_temp          = 0;

    // config_t.stored_size is the sole control surface for the ring buffer depth;
    // keep component_config().stored_size in sync with it here.
    auto ccfg        = component_config();
    ccfg.stored_size = _cfg.stored_size;
    component_config(ccfg);

    uint8_t id{};
    if (!readRegister8(CHIP_ID, id, 0) || id != CHIP_ID_VALUE) {
        M5_LIB_LOGE("Not BMI270 %02X", id);
        return false;
    }

    if (!writeRegister8(CMD, CMD_SOFTRESET)) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }
    {
        uint8_t pwr_conf{1};
        uint32_t retry = PWR_CONF_POLL_RETRY;
        for (;;) {
            m5::utility::delay(1);
            if (!readRegister8(PWR_CONF, pwr_conf, 0)) {
                M5_LIB_LOGE("Failed to read PWR_CONF");
                return false;
            }
            if (pwr_conf == 0 || --retry == 0) {
                break;
            }
        }
        if (pwr_conf != 0) {
            M5_LIB_LOGE("PWR_CONF did not clear after reset");
            return false;
        }
    }

    if (!writeRegister8(PWR_CONF, 0x00)) {
        M5_LIB_LOGE("Failed to disable advanced power save");
        return false;
    }
    m5::utility::delay(1);  // >= 450us required before further register access

    if (!writeRegister8(INIT_CTRL, 0x00)) {
        M5_LIB_LOGE("Failed to begin config load");
        return false;
    }
    if (!upload_config()) {
        M5_LIB_LOGE("Failed to upload config");
        return false;
    }
    if (!writeRegister8(INIT_CTRL, 0x01)) {
        M5_LIB_LOGE("Failed to finalize config load");
        return false;
    }

    {
        uint8_t status{};
        uint32_t retry = INTERNAL_STATUS_POLL_RETRY;
        for (;;) {
            m5::utility::delay(1);
            if (!readRegister8(INTERNAL_STATUS, status, 0)) {
                M5_LIB_LOGE("Failed to read INTERNAL_STATUS");
                return false;
            }
            const uint8_t msg = status & INTERNAL_STATUS_MESSAGE_MASK;
            if (msg == INTERNAL_STATUS_INIT_OK) {
                break;
            }
            if (msg != 0x00) {
                M5_LIB_LOGE("BMI270 init error (INTERNAL_STATUS:%02X)", status);
                return false;
            }
            if (--retry == 0) {
                M5_LIB_LOGE("BMI270 init timeout (INTERNAL_STATUS:%02X)", status);
                return false;
            }
        }
    }

    if (!write_register8_verified(PWR_CTRL, PWR_CTRL_ACC_GYR_TEMP_ON) ||
        !write_register8_verified(ACC_CONF,
                                  CONF_FILTER_PERF | CONF_BWP_NORMAL | static_cast<uint8_t>(_cfg.accel_odr)) ||
        !write_register8_verified(ACC_RANGE, static_cast<uint8_t>(_cfg.accel_range)) ||
        !write_register8_verified(GYR_CONF, CONF_FILTER_PERF | CONF_BWP_NORMAL | static_cast<uint8_t>(_cfg.gyro_odr)) ||
        !write_register8_verified(GYR_RANGE, static_cast<uint8_t>(_cfg.gyro_range))) {
        M5_LIB_LOGE("Failed to configure");
        return false;
    }

    return _cfg.start_periodic ? startPeriodicMeasurement() : true;
}

void UnitBMI270::update(const bool force)
{
    _updated = false;
    if (inPeriodic()) {
        const auto at = m5::utility::millis();
        if (force || !_latest || at >= _latest + _interval) {
            bmi270::Data d{};
            _updated = read_measurement(d);
            if (_updated) {
                _data->push_back(d);
                ++_pushed;
                _latest = at;
            }
        }
    }
}

bool UnitBMI270::start_periodic_measurement()
{
    if (inPeriodic()) {
        return false;
    }
    _periodic = true;
    _latest   = 0;
    _interval = odr_interval_ms(_cfg.gyro_odr);
    return true;
}

bool UnitBMI270::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

bool UnitBMI270::startPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitBMI270, bmi270::Data>::startPeriodicMeasurement();
}

bool UnitBMI270::stopPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitBMI270, bmi270::Data>::stopPeriodicMeasurement();
}

bool UnitBMI270::writeAccelRange(const bmi270::AccelRange range)
{
    if (write_register8_verified(ACC_RANGE, static_cast<uint8_t>(range))) {
        _cfg.accel_range = range;
        return true;
    }
    return false;
}

bool UnitBMI270::writeGyroRange(const bmi270::GyroRange range)
{
    if (write_register8_verified(GYR_RANGE, static_cast<uint8_t>(range))) {
        _cfg.gyro_range = range;
        return true;
    }
    return false;
}

bool UnitBMI270::upload_config()
{
    const size_t total = sizeof(bmi270_config_file);
    for (size_t index = 0; index < total; index += CONFIG_CHUNK_SIZE) {
        const size_t len = std::min(CONFIG_CHUNK_SIZE, total - index);
        const uint8_t addr[2]{
            static_cast<uint8_t>((index >> 1) & 0x0F),
            static_cast<uint8_t>(index >> 5),
        };
        if (!writeRegister(INIT_ADDR_0, addr, 2) || !writeRegister(INIT_DATA, bmi270_config_file + index, len)) {
            M5_LIB_LOGE("Failed to upload config at offset %u", static_cast<unsigned int>(index));
            return false;
        }
    }
    return true;
}

bool UnitBMI270::read_measurement(bmi270::Data& d)
{
    uint8_t status{};
    if (!readRegister8(INT_STATUS_1, status, 0)) {
        M5_LIB_LOGE("Failed to read INT_STATUS_1");
        return false;
    }
    constexpr uint8_t both_drdy = INT_STATUS_1_ACC_DRDY | INT_STATUS_1_GYR_DRDY;
    if ((status & both_drdy) != both_drdy) {
        return false;  // Not ready yet (normal case, no log)
    }
    if (!readRegister(DATA_8, d.raw.data(), d.raw.size(), 0)) {
        M5_LIB_LOGE("Failed to read data");
        return false;
    }

    // Temperature is a separate, lower rate register (updates every 10ms in gyro normal
    // mode); a failed or invalid read simply keeps the previous _last_temp, it does not
    // fail the whole measurement.
    uint8_t traw[2]{};
    if (readRegister(TEMPERATURE_0, traw, sizeof(traw), 0)) {
        const int16_t t = static_cast<int16_t>((traw[1] << 8) | traw[0]);
        if (t != static_cast<int16_t>(0x8000)) {
            _last_temp = t;
        }
    }
    return true;
}

bool UnitBMI270::write_register8_verified(const uint8_t reg, const uint8_t val)
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

Capability UnitBMI270::readImuRaw(m5::imu::imu_raw_data_t& out)
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
    out.temp     = _last_temp;
    return imuSpec();
}

void UnitBMI270::imuConvertParam(m5::imu::imu_convert_param_t& out) const
{
    out.accel_res   = accel_res(_cfg.accel_range);
    out.gyro_res    = gyro_res(_cfg.gyro_range);
    out.temp_res    = 1.0f / 512.0f;
    out.temp_offset = 23.0f;
}

}  // namespace unit
}  // namespace m5
