/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_spa06.cpp
  @brief UnitSPA06 (SPA06-003) barometric pressure + temperature
*/
#include "unit_spa06.hpp"
#include <M5Utility.hpp>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::spa06;
using namespace m5::unit::spa06::command;

namespace {
// ID (0x0D) reset value is 0x11h (datasheet Table 7 / Sec 7.10): REV_ID[7:4]=1, PROD_ID[3:0]=1.
constexpr uint8_t PROD_ID_VALUE{0x01};
constexpr uint8_t SOFT_RESET{0x09};         // RESET(0x0C) SOFT_RST[3:0] = 1001b (datasheet Sec 7.9)
constexpr uint8_t MEAS_CTRL_CONT_PT{0x07};  // MEAS_CFG(0x08) MEAS_CTRL[2:0] = continuous P+T
constexpr uint8_t MEAS_CTRL_STANDBY{0x00};  // MEAS_CFG(0x08) MEAS_CTRL[2:0] = idle/stop
constexpr uint8_t P_SHIFT_BIT{0x04};        // CFG_REG(0x09) bit2 PRS_SHIFT_EN
constexpr uint8_t T_SHIFT_BIT{0x08};        // CFG_REG(0x09) bit3 TMP_SHIFT_EN
constexpr uint8_t COEF_SENSOR_RDY_MASK{0xC0};
constexpr uint32_t POLL_INTERVAL_MS{20};  // ~50Hz poll of MEAS_CFG (measurement itself runs at the
                                           // configured PM_RATE/TMP_RATE)

// Map an oversampling value (1..128) to its PM_PRC/TM_PRC nibble (0..7, datasheet Table 4)
uint8_t prc_bits(const uint8_t oversampling)
{
    switch (oversampling) {
        case 1:
            return 0;
        case 2:
            return 1;
        case 4:
            return 2;
        case 8:
            return 3;
        case 16:
            return 4;
        case 32:
            return 5;
        case 64:
            return 6;
        case 128:
            return 7;
        default:
            return 4;
    }
}
}  // namespace

namespace m5 {
namespace unit {

const char UnitSPA06::name[] = "UnitSPA06";
const types::uid_t UnitSPA06::uid{"UnitSPA06"_mmh3};
const types::attr_t UnitSPA06::attr{attribute::AccessI2C};

UnitSPA06::UnitSPA06(const uint8_t addr)
    : Component(addr), _data{new m5::container::CircularBuffer<spa06::Data>(spa06::DEFAULT_STORED_SIZE)}
{
    auto ccfg        = component_config();
    ccfg.clock       = 400 * 1000U;
    ccfg.stored_size = spa06::DEFAULT_STORED_SIZE;
    component_config(ccfg);
}

bool UnitSPA06::begin()
{
    auto ssize = stored_size();
    if (ssize != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<spa06::Data>(ssize));
        if (!_data) {
            M5_LIB_LOGE("Failed to allocate");
            return false;
        }
    }
    _data->clear();
    _pushed = _consumed = 0;

    uint8_t id{};
    if (!readRegister8(REG_ID, id, 0) || (id & 0x0F) != PROD_ID_VALUE) {
        M5_LIB_LOGE("Not SPA06 %02X", id);
        return false;
    }
    if (!writeRegister8(REG_RESET, SOFT_RESET)) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }
    m5::utility::delay(50);

    // Wait for sensor + coefficients ready (MEAS_CFG bit7 COEF_RDY, bit6 SENSOR_RDY).
    bool ready{};
    for (int i = 0; i < 100; ++i) {
        uint8_t st{};
        if (readRegister8(REG_MEAS_CFG, st, 0) && (st & COEF_SENSOR_RDY_MASK) == COEF_SENSOR_RDY_MASK) {
            ready = true;
            break;
        }
        m5::utility::delay(10);
    }
    if (!ready) {
        M5_LIB_LOGE("Sensor not ready");
        return false;
    }
    if (!read_coefficients()) {
        return false;
    }

    _kp = spa06::scale_factor(_cfg.pressure_oversampling);
    _kt = spa06::scale_factor(_cfg.temperature_oversampling);

    const uint8_t prs_cfg = static_cast<uint8_t>((_cfg.rate << 4) | prc_bits(_cfg.pressure_oversampling));
    const uint8_t tmp_cfg = static_cast<uint8_t>((_cfg.rate << 4) | prc_bits(_cfg.temperature_oversampling));
    uint8_t cfg_reg{};
    // The datasheet requires the SHIFT bit to be enabled whenever oversampling exceeds 8x.
    if (_cfg.pressure_oversampling > 8) {
        cfg_reg |= P_SHIFT_BIT;
    }
    if (_cfg.temperature_oversampling > 8) {
        cfg_reg |= T_SHIFT_BIT;
    }
    if (!writeRegister8(REG_PRS_CFG, prs_cfg) || !writeRegister8(REG_TMP_CFG, tmp_cfg) ||
        !writeRegister8(REG_CFG_REG, cfg_reg)) {
        M5_LIB_LOGE("Failed to configure");
        return false;
    }

    return _cfg.start_periodic ? startPeriodicMeasurement() : true;
}

bool UnitSPA06::read_coefficients()
{
    std::array<uint8_t, 21> r{};
    if (!readRegister(REG_COEF, r.data(), r.size(), 0)) {
        M5_LIB_LOGE("Failed to read coefficients");
        return false;
    }
    _coeffs = spa06::parse_coeffs(r);
    return true;
}

void UnitSPA06::update(const bool force)
{
    _updated = false;
    if (inPeriodic()) {
        const auto at = m5::utility::millis();
        if (force || !_latest || at >= _latest + _interval) {
            spa06::Data d{};
            _updated = read_measurement(d);
            if (_updated) {
                _data->push_back(d);
                ++_pushed;
                _latest = at;
            }
        }
    }
}

bool UnitSPA06::read_measurement(spa06::Data& d)
{
    uint8_t st{};
    if (!readRegister8(REG_MEAS_CFG, st, 0)) {
        M5_LIB_LOGE("Failed to read status");
        return false;
    }
    if ((st & 0x30) == 0) {
        return false;  // Neither PRS_RDY nor TMP_RDY yet (normal, no log)
    }
    if (!readRegister(REG_PSR_B2, d.raw.data(), d.raw.size(), 0)) {
        M5_LIB_LOGE("Failed to read data");
        return false;
    }
    return true;
}

bool UnitSPA06::start_periodic_measurement()
{
    if (inPeriodic()) {
        return false;
    }
    // Start continuous pressure + temperature background measurement.
    if (!writeRegister8(REG_MEAS_CFG, MEAS_CTRL_CONT_PT)) {
        return false;
    }
    _periodic = true;
    _latest   = 0;
    _interval = POLL_INTERVAL_MS;
    return true;
}

bool UnitSPA06::stop_periodic_measurement()
{
    writeRegister8(REG_MEAS_CFG, MEAS_CTRL_STANDBY);
    _periodic = false;
    return true;
}

bool UnitSPA06::startPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitSPA06, spa06::Data>::startPeriodicMeasurement();
}

bool UnitSPA06::stopPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitSPA06, spa06::Data>::stopPeriodicMeasurement();
}

float UnitSPA06::pressure() const
{
    if (!_data || _data->empty()) {
        return 0.0f;
    }
    const auto d = _data->back();
    if (!d) {
        return 0.0f;
    }
    return spa06::compensate_pressure(d->psr_raw(), d->tmp_raw(), _coeffs, _kp, _kt) / 100.0f;  // Pa -> hPa
}

float UnitSPA06::temperature() const
{
    if (!_data || _data->empty()) {
        return 0.0f;
    }
    const auto d = _data->back();
    if (!d) {
        return 0.0f;
    }
    return spa06::compensate_temperature(d->tmp_raw(), _coeffs, _kt);
}

float UnitSPA06::altitude(const float sea_level_hpa) const
{
    return spa06::altitude_m(pressure(), sea_level_hpa);
}

float UnitSPA06::relativeAltitude() const
{
    return spa06::altitude_m(pressure(), _reference_hpa);
}

void UnitSPA06::setReference()
{
    _reference_hpa = pressure();
}

}  // namespace unit
}  // namespace m5
