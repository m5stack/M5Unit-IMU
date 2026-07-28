/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mag3110.cpp
  @brief UnitMAG3110 (3-axis magnetometer) unit component
*/
#include "unit_mag3110.hpp"
#include <M5Utility.hpp>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::mag3110;
using namespace m5::unit::mag3110::command;
using m5::imu::Capability;

namespace {
constexpr uint8_t WHOAMI_VALUE{0xC4};
constexpr uint8_t CTRL_REG1_STANDBY{0x00};    // AC=0
constexpr uint8_t CTRL_REG1_AC{0x01};         // bit0: 0=STANDBY 1=ACTIVE
constexpr uint8_t CTRL_REG1_DR_MASK{0xE0};    // bits[7:5]
constexpr uint8_t CTRL_REG1_OS_MASK{0x18};    // bits[4:3]
constexpr uint8_t CTRL_REG2_AUTO_MRST{0x80};  // AUTO_MRST_EN=1, RAW=0 (user-offset corrected)
}  // namespace

namespace m5 {
namespace unit {

const char UnitMAG3110::name[] = "UnitMAG3110";
const types::uid_t UnitMAG3110::uid{"UnitMAG3110"_mmh3};
const types::attr_t UnitMAG3110::attr{attribute::AccessI2C};

UnitMAG3110::UnitMAG3110(const uint8_t addr)
    : Component(addr), _data{new m5::container::CircularBuffer<mag3110::Data>(mag3110::DEFAULT_STORED_SIZE)}
{
    auto ccfg        = component_config();
    ccfg.clock       = 400 * 1000U;
    ccfg.stored_size = mag3110::DEFAULT_STORED_SIZE;
    component_config(ccfg);
}

bool UnitMAG3110::begin()
{
    // Re-entrant begin(): make sure _periodic and the hardware never get out of sync
    stop_periodic_measurement();

    auto ssize = stored_size();
    if (ssize != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<mag3110::Data>(ssize));
        if (!_data) {
            M5_LIB_LOGE("Failed to allocate");
            return false;
        }
    }
    _data->clear();
    _pushed = _consumed = 0;

    uint8_t id{};
    if (!readRegister8(REG_WHO_AM_I, id, 0) || id != WHOAMI_VALUE) {
        M5_LIB_LOGE("Not MAG3110 %02X", id);
        return false;
    }

    if (!softReset()) {
        return false;
    }

    // Seed the live state from the begin settings; writeDataRate() owns it from here on
    _rate = _cfg.rate;
    _os   = _cfg.over_sampling;

    return _cfg.start_periodic ? startPeriodicMeasurement(_cfg.rate, _cfg.over_sampling) : true;
}

bool UnitMAG3110::writeDataRate(const mag3110::DataRate rate, const mag3110::OverSampling os)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    // Every REG_CTRL_REG1 field except AC can only be changed in STANDBY, and a write made in
    // ACTIVE is silently ignored instead of reported. inPeriodic() being false does not mean AC is
    // clear -- stopPeriodicMeasurement() deliberately leaves the chip running -- so clear it here,
    // in its own write, before the fields are touched.
    if (!writeRegister8(REG_CTRL_REG1, CTRL_REG1_STANDBY)) {
        M5_LIB_LOGE("Failed to standby");
        return false;
    }
    // Park the new rate in the register with AC still clear, so readDataRate() reflects it even
    // before measurement starts
    if (!writeRegister8(REG_CTRL_REG1, static_cast<uint8_t>(mag3110::ctrl_reg1_active(rate, os) & ~CTRL_REG1_AC))) {
        M5_LIB_LOGE("Failed to set the data rate");
        return false;
    }
    _rate     = rate;
    _os       = os;
    _interval = mag3110::interval_ms(rate, os);
    return true;
}

bool UnitMAG3110::readDataRate(mag3110::DataRate& rate, mag3110::OverSampling& os)
{
    uint8_t v{};
    if (!readRegister8(REG_CTRL_REG1, v, 0)) {
        M5_LIB_LOGE("Failed to read");
        return false;
    }
    rate = static_cast<mag3110::DataRate>((v & CTRL_REG1_DR_MASK) >> 5);
    os   = static_cast<mag3110::OverSampling>((v & CTRL_REG1_OS_MASK) >> 3);
    return true;
}

bool UnitMAG3110::softReset()
{
    if (inPeriodic()) {
        M5_LIB_LOGE("Periodic measurements are running");
        return false;
    }
    // The MAG3110 has no soft reset command; returning to STANDBY (CTRL_REG1[AC]=0) is the
    // closest equivalent and is also the state begin() configures from
    if (!writeRegister8(REG_CTRL_REG1, CTRL_REG1_STANDBY)) {
        M5_LIB_LOGE("Failed to standby");
        return false;
    }
    m5::utility::delay(10);
    return true;
}

void UnitMAG3110::update(const bool force)
{
    _updated = false;
    if (inPeriodic()) {
        const auto at = m5::utility::millis();
        if (force || !_latest || at >= _latest + _interval) {
            mag3110::Data d{};
            _updated = read_measurement(d);
            if (_updated) {
                _data->push_back(d);
                ++_pushed;
                _latest = at;
            }
        }
    }
}

bool UnitMAG3110::start_periodic_measurement(const mag3110::DataRate rate, const mag3110::OverSampling os)
{
    if (inPeriodic()) {
        return false;
    }

    // Configure while in STANDBY (CTRL_REG1[AC]=0): enable automatic magnetic reset, then enter
    // continuous ACTIVE mode at the requested rate. CTRL_REG2 is written whole rather than
    // read-modify-written: AUTO_MRST_EN is write-only and always reads back as 0, so merging would
    // silently clear it.
    if (!writeRegister8(REG_CTRL_REG2, CTRL_REG2_AUTO_MRST) ||
        !writeRegister8(REG_CTRL_REG1, mag3110::ctrl_reg1_active(rate, os))) {
        M5_LIB_LOGE("Failed to set mode");
        return false;
    }

    _rate     = rate;
    _os       = os;
    _periodic = true;
    _latest   = 0;
    _interval = mag3110::interval_ms(rate, os);
    return true;
}

bool UnitMAG3110::start_periodic_measurement()
{
    return start_periodic_measurement(_rate, _os);
}

bool UnitMAG3110::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

bool UnitMAG3110::startPeriodicMeasurement(const mag3110::DataRate rate, const mag3110::OverSampling os)
{
    return PeriodicMeasurementAdapter<UnitMAG3110, mag3110::Data>::startPeriodicMeasurement(rate, os);
}

bool UnitMAG3110::startPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitMAG3110, mag3110::Data>::startPeriodicMeasurement();
}

bool UnitMAG3110::stopPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitMAG3110, mag3110::Data>::stopPeriodicMeasurement();
}

bool UnitMAG3110::sleep()
{
    // A standby chip produces no data, so stop measuring first
    if (inPeriodic() && !stopPeriodicMeasurement()) {
        M5_LIB_LOGE("Failed to stop before sleep");
        return false;
    }
    if (!writeRegister8(REG_CTRL_REG1, CTRL_REG1_STANDBY)) {
        M5_LIB_LOGE("Failed to enter standby");
        return false;
    }
    return true;
}

bool UnitMAG3110::wakeup()
{
    // STANDBY is also the state the chip has to be in to be configured, so it is already
    // ready; startPeriodicMeasurement() enters ACTIVE.
    return true;
}

bool UnitMAG3110::read_measurement(mag3110::Data& d)
{
    // Burst from DR_STATUS: DR_STATUS + X/Y/Z (big-endian). Reading the data MSBs clears ZYXDR.
    if (!readRegister(REG_DR_STATUS, d.raw.data(), d.raw.size(), 0)) {
        M5_LIB_LOGE("Failed to read data");
        return false;
    }
    if (!d.drdy()) {
        return false;  // Not ready yet (normal case, no log)
    }
    return true;
}

Capability UnitMAG3110::readImuRaw(m5::imu::imu_raw_data_t& out)
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

void UnitMAG3110::imuConvertParam(m5::imu::imu_convert_param_t& out) const
{
    out.mag_res = 1.0f / 16;
}

}  // namespace unit
}  // namespace m5
