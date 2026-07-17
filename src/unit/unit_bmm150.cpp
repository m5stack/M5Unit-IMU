/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_bmm150.cpp
  @brief UnitBMM150 (3-axis magnetometer) unit component
*/
#include "unit_bmm150.hpp"
#include <M5Utility.hpp>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::bmm150;
using namespace m5::unit::bmm150::command;
using m5::imu::Capability;

namespace {
constexpr uint32_t WRITE_VERIFY_RETRY{16};

// REG_CHIP_ID expected value / REG_POWER_CONTROL, REG_OP_MODE command values
constexpr uint8_t WHOAMI_VALUE{0x32};
constexpr uint8_t POWER_ON{0x01};
// bit7 SoftReset + bit1 SoftReset + bit0 PowerControl combined.
// Not modeled as an enum because it aliases distinct bit fields (m5-cpp-rules)
constexpr uint8_t SOFT_RESET_AND_POWER_ON{0x83};
constexpr uint8_t OPMODE_NORMAL{0x00};

// Output data rate -> sampling interval (ms), indexed by the DataRate raw value.
// DataRate is bit-value based (non-monotonic in Hz); see unit_bmm150_data.hpp
constexpr uint32_t RATE_INTERVAL_MS[8] = {
    100,  // Rate10Hz
    500,  // Rate2Hz
    167,  // Rate6Hz
    125,  // Rate8Hz
    67,   // Rate15Hz
    50,   // Rate20Hz
    40,   // Rate25Hz
    33,   // Rate30Hz
};

constexpr uint32_t interval_ms(const DataRate rate)
{
    return RATE_INTERVAL_MS[static_cast<uint8_t>(rate)];
}
}  // namespace

namespace m5 {
namespace unit {

// class UnitBMM150
const char UnitBMM150::name[] = "UnitBMM150";
const types::uid_t UnitBMM150::uid{"UnitBMM150"_mmh3};
const types::attr_t UnitBMM150::attr{attribute::AccessI2C};

UnitBMM150::UnitBMM150(const uint8_t addr)
    : Component(addr), _data{new m5::container::CircularBuffer<bmm150::Data>(bmm150::DEFAULT_STORED_SIZE)}
{
    auto ccfg        = component_config();
    ccfg.clock       = 400 * 1000U;
    ccfg.stored_size = bmm150::DEFAULT_STORED_SIZE;
    component_config(ccfg);
}

bool UnitBMM150::begin()
{
    // Re-entrant begin(): make sure _periodic and the hardware never get out of sync
    stop_periodic_measurement();

    auto ssize = stored_size();
    if (ssize != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<bmm150::Data>(ssize));
        if (!_data) {
            M5_LIB_LOGE("Failed to allocate");
            return false;
        }
    }
    _data->clear();
    _pushed = _consumed = 0;

    // Suspend->Sleep transition: power on then wait for the start-up time before
    // REG_CHIP_ID becomes readable (datasheet 3.0ms; missing in M5Unified's implementation)
    if (!writeRegister8(REG_POWER_CONTROL, POWER_ON)) {
        M5_LIB_LOGE("Failed to power on");
        return false;
    }
    m5::utility::delay(3);

    // REG_CHIP_ID reads 0x00 while suspended, so this check must come after power-on
    uint8_t id{};
    if (!readRegister8(REG_CHIP_ID, id, 0) || id != WHOAMI_VALUE) {
        M5_LIB_LOGE("Not BMM150 %02X", id);
        return false;
    }

    if (!softReset()) {
        return false;
    }

    if (!read_trim(_trim)) {
        return false;
    }

    return _cfg.start_periodic ? startPeriodicMeasurement(_cfg.rate, _cfg.preset) : true;
}

bool UnitBMM150::softReset()
{
    if (inPeriodic()) {
        M5_LIB_LOGE("Periodic measurements are running");
        return false;
    }
    if (!writeRegister8(REG_POWER_CONTROL, SOFT_RESET_AND_POWER_ON)) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }
    m5::utility::delay(3);
    return true;
}

void UnitBMM150::update(const bool force)
{
    _updated = false;
    if (inPeriodic()) {
        const auto at = m5::utility::millis();
        if (force || !_latest || at >= _latest + _interval) {
            bmm150::Data d{};
            _updated = read_measurement(d);
            if (_updated) {
                _data->push_back(d);
                ++_pushed;
                _latest = at;
            }
        }
    }
}

bool UnitBMM150::start_periodic_measurement(const bmm150::DataRate rate, const bmm150::Preset preset)
{
    if (inPeriodic()) {
        return false;
    }

    // writePreset/writeDataRate also sync _cfg (and _interval) on success
    if (!writePreset(preset) || !writeDataRate(rate)) {
        M5_LIB_LOGE("Failed to initialize");
        return false;
    }

    _periodic = true;
    _latest   = 0;
    _interval = interval_ms(rate);
    return true;
}

bool UnitBMM150::start_periodic_measurement()
{
    return start_periodic_measurement(_cfg.rate, _cfg.preset);
}

bool UnitBMM150::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

bool UnitBMM150::startPeriodicMeasurement(const bmm150::DataRate rate, const bmm150::Preset preset)
{
    return PeriodicMeasurementAdapter<UnitBMM150, bmm150::Data>::startPeriodicMeasurement(rate, preset);
}

bool UnitBMM150::startPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitBMM150, bmm150::Data>::startPeriodicMeasurement();
}

bool UnitBMM150::stopPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitBMM150, bmm150::Data>::stopPeriodicMeasurement();
}

bool UnitBMM150::writePreset(const bmm150::Preset preset)
{
    if (write_register8_verified(REG_REP_XY, preset_rep_xy(preset)) &&
        write_register8_verified(REG_REP_Z, preset_rep_z(preset))) {
        _cfg.preset = preset;
        return true;
    }
    return false;
}

bool UnitBMM150::writeDataRate(const bmm150::DataRate rate)
{
    if (write_register8_verified(REG_OP_MODE,
                                 static_cast<uint8_t>((static_cast<uint8_t>(rate) << 3) | OPMODE_NORMAL))) {
        _cfg.rate = rate;
        _interval = interval_ms(rate);
        return true;
    }
    return false;
}

bool UnitBMM150::readDataRate(bmm150::DataRate& rate)
{
    uint8_t reg{};
    if (!readRegister8(REG_OP_MODE, reg, 0)) {
        return false;
    }
    rate = static_cast<bmm150::DataRate>((reg >> 3) & 0x07);
    return true;
}

bool UnitBMM150::readPreset(bmm150::Preset& preset)
{
    uint8_t rep_xy{}, rep_z{};
    if (!readRegister8(REG_REP_XY, rep_xy, 0) || !readRegister8(REG_REP_Z, rep_z, 0)) {
        return false;
    }
    constexpr bmm150::Preset PRESETS[]{bmm150::Preset::LowPower, bmm150::Preset::Regular,
                                       bmm150::Preset::EnhancedRegular, bmm150::Preset::HighAccuracy};
    for (const auto p : PRESETS) {
        if (rep_xy == preset_rep_xy(p) && rep_z == preset_rep_z(p)) {
            preset = p;
            return true;
        }
    }
    return false;  // Current registers do not match any known preset
}

bool UnitBMM150::read_trim(bmm150::trim_t& trim)
{
    std::array<uint8_t, 2> buf0{};   // 0x5D..0x5E: dig_x1, dig_y1
    std::array<uint8_t, 4> buf1{};   // 0x62..0x65: dig_z4(2), dig_x2, dig_y2
    std::array<uint8_t, 10> buf2{};  // 0x68..0x71: dig_z2(2), dig_z1(2), dig_xyz1(2), dig_z3(2), dig_xy2, dig_xy1

    if (!readRegister(REG_TRIM_START, buf0.data(), buf0.size(), 0) ||
        !readRegister(static_cast<uint8_t>(REG_TRIM_START + 5), buf1.data(), buf1.size(), 0) ||
        !readRegister(static_cast<uint8_t>(REG_TRIM_START + 11), buf2.data(), buf2.size(), 0)) {
        M5_LIB_LOGE("Failed to read trim registers");
        return false;
    }

    trim.dig_x1 = static_cast<int8_t>(buf0[0]);
    trim.dig_y1 = static_cast<int8_t>(buf0[1]);

    trim.dig_z4 = static_cast<int16_t>((buf1[1] << 8) | buf1[0]);
    trim.dig_x2 = static_cast<int8_t>(buf1[2]);
    trim.dig_y2 = static_cast<int8_t>(buf1[3]);

    trim.dig_z2   = static_cast<int16_t>((buf2[1] << 8) | buf2[0]);
    trim.dig_z1   = static_cast<uint16_t>((buf2[3] << 8) | buf2[2]);
    trim.dig_xyz1 = static_cast<uint16_t>(((buf2[5] << 8) | buf2[4]) & 0x3FFF);
    trim.dig_z3   = static_cast<int16_t>((buf2[7] << 8) | buf2[6]);
    trim.dig_xy2  = static_cast<int8_t>(buf2[8]);
    trim.dig_xy1  = buf2[9];

    return true;
}

bool UnitBMM150::read_measurement(bmm150::Data& d)
{
    if (!readRegister(REG_DATA_X_LSB, d.raw.data(), d.raw.size(), 0)) {
        M5_LIB_LOGE("Failed to read data");
        return false;
    }
    if (!d.drdy()) {
        return false;  // Not ready yet (normal case, no log)
    }
    d.trim = _trim;
    return true;
}

bool UnitBMM150::write_register8_verified(const uint8_t reg, const uint8_t val)
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

Capability UnitBMM150::readImuRaw(m5::imu::imu_raw_data_t& out)
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

    out.mag[0] = d->comp_x();
    out.mag[1] = d->comp_y();
    out.mag[2] = d->comp_z();
    return imuSpec();
}

void UnitBMM150::imuConvertParam(m5::imu::imu_convert_param_t& out) const
{
    out.mag_res = 1.0f / 16;
}

}  // namespace unit
}  // namespace m5
