/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_ak8963.cpp
  @brief UnitAK8963 (3-axis magnetometer) unit component
*/
#include "unit_ak8963.hpp"
#include <M5Utility.hpp>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::ak8963;
using namespace m5::unit::ak8963::command;
using m5::imu::Capability;

namespace {
constexpr uint8_t WHOAMI_VALUE{0x48};
constexpr uint8_t BIT_16BIT{0x10};  // CNTL1 bit4: 16-bit output
constexpr uint8_t SOFT_RESET{0x01};
}  // namespace

namespace m5 {
namespace unit {

const char UnitAK8963::name[] = "UnitAK8963";
const types::uid_t UnitAK8963::uid{"UnitAK8963"_mmh3};
const types::attr_t UnitAK8963::attr{attribute::AccessI2C};

UnitAK8963::UnitAK8963(const uint8_t addr)
    : Component(addr), _data{new m5::container::CircularBuffer<ak8963::Data>(ak8963::DEFAULT_STORED_SIZE)}
{
    auto ccfg        = component_config();
    ccfg.clock       = 400 * 1000U;
    ccfg.stored_size = ak8963::DEFAULT_STORED_SIZE;
    component_config(ccfg);
}

bool UnitAK8963::begin()
{
    // Re-entrant begin(): make sure _periodic and the hardware never get out of sync
    stop_periodic_measurement();

    auto ssize = stored_size();
    if (ssize != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<ak8963::Data>(ssize));
        if (!_data) {
            M5_LIB_LOGE("Failed to allocate");
            return false;
        }
    }
    _data->clear();
    _pushed = _consumed = 0;

    uint8_t id{};
    if (!readRegister8(REG_WIA, id, 0) || id != WHOAMI_VALUE) {
        M5_LIB_LOGE("Not AK8963 %02X", id);
        return false;
    }

    // Soft reset, then read the factory sensitivity adjustment (ASA) from FUSE-ROM.
    if (!softReset()) {
        return false;
    }
    if (!read_sensitivity(_asa)) {
        return false;
    }

    return _cfg.start_periodic ? startPeriodicMeasurement() : true;
}

bool UnitAK8963::softReset()
{
    if (inPeriodic()) {
        M5_LIB_LOGE("Periodic measurements are running");
        return false;
    }
    if (!writeRegister8(REG_CNTL2, SOFT_RESET)) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }
    m5::utility::delay(10);
    return true;
}

bool UnitAK8963::read_sensitivity(std::array<uint8_t, 3>& asa)
{
    if (!writeRegister8(REG_CNTL1, static_cast<uint8_t>(Mode::PowerDown))) {
        return false;
    }
    m5::utility::delay(10);
    if (!writeRegister8(REG_CNTL1, static_cast<uint8_t>(Mode::FuseRom))) {
        return false;
    }
    m5::utility::delay(10);
    if (!readRegister(REG_ASAX, asa.data(), asa.size(), 0)) {
        M5_LIB_LOGE("Failed to read ASA");
        return false;
    }
    // Leave Fuse ROM access mode. Power-down is the state every mode change on this chip has to
    // pass through, and it is where begin() has to leave the device when it is not going on to
    // start measuring -- Fuse ROM access is neither a measurement nor an idle state.
    if (!writeRegister8(REG_CNTL1, static_cast<uint8_t>(Mode::PowerDown))) {
        return false;
    }
    m5::utility::delay(10);
    return true;
}

void UnitAK8963::update(const bool force)
{
    _updated = false;
    if (inPeriodic()) {
        const auto at = m5::utility::millis();
        if (force || !_latest || at >= _latest + _interval) {
            ak8963::Data d{};
            _updated = read_measurement(d);
            if (_updated) {
                _data->push_back(d);
                ++_pushed;
                _latest = at;
            }
        }
    }
}

bool UnitAK8963::start_periodic_measurement()
{
    if (inPeriodic()) {
        return false;
    }

    // Enter 16-bit continuous 100Hz. AK8963 requires a power-down between mode changes.
    if (!writeRegister8(REG_CNTL1, static_cast<uint8_t>(Mode::PowerDown))) {
        M5_LIB_LOGE("Failed to power down");
        return false;
    }
    m5::utility::delay(10);
    if (!writeRegister8(REG_CNTL1, static_cast<uint8_t>(BIT_16BIT | static_cast<uint8_t>(Mode::Cont100Hz)))) {
        M5_LIB_LOGE("Failed to set mode");
        return false;
    }

    _periodic = true;
    _latest   = 0;
    _interval = ak8963::CONT100HZ_INTERVAL_MS;
    return true;
}

bool UnitAK8963::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

bool UnitAK8963::startPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitAK8963, ak8963::Data>::startPeriodicMeasurement();
}

bool UnitAK8963::stopPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitAK8963, ak8963::Data>::stopPeriodicMeasurement();
}

bool UnitAK8963::sleep()
{
    // A powered-down chip must not be polled, so stop measuring first
    if (inPeriodic() && !stopPeriodicMeasurement()) {
        M5_LIB_LOGE("Failed to stop before sleep");
        return false;
    }
    if (!writeRegister8(REG_CNTL1, static_cast<uint8_t>(Mode::PowerDown))) {
        M5_LIB_LOGE("Failed to power down");
        return false;
    }
    m5::utility::delay(10);
    return true;
}

bool UnitAK8963::wakeup()
{
    // Power-down is also the state every mode change has to pass through on this chip, so the
    // device is already in a configurable state; startPeriodicMeasurement() sets the mode.
    return true;
}

bool UnitAK8963::read_measurement(ak8963::Data& d)
{
    // Burst from ST1: ST1 + X/Y/Z(LE) + ST2. Reading ST2 unlatches the sample.
    if (!readRegister(REG_ST1, d.raw.data(), d.raw.size(), 0)) {
        M5_LIB_LOGE("Failed to read data");
        return false;
    }
    if (!d.drdy() || d.overflow()) {
        return false;  // Not ready or magnetic overflow (drop, no log)
    }
    d.asa = _asa;
    return true;
}

Capability UnitAK8963::readImuRaw(m5::imu::imu_raw_data_t& out)
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

    out.mag[0] = d->adj_x();
    out.mag[1] = d->adj_y();
    out.mag[2] = d->adj_z();
    return imuSpec();
}

void UnitAK8963::imuConvertParam(m5::imu::imu_convert_param_t& out) const
{
    out.mag_res = 1.0f / 16;
}

}  // namespace unit
}  // namespace m5
