/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_sh200q.cpp
  @brief UnitSH200Q (6-axis accel + gyro) unit component
*/
#include "unit_sh200q.hpp"
#include <M5Utility.hpp>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::sh200q;
using namespace m5::unit::sh200q::command;
using m5::imu::Capability;

namespace {
constexpr uint32_t WRITE_VERIFY_RETRY{16};
constexpr uint8_t CHIP_ID_VALUE{0x18};
// REG_ACC_CONFIG: bit7 HPF bypass + bit0 filter enable; ODR merges into bits[4:3]
constexpr uint8_t ACC_CONFIG_BASE{0x81};
// REG_GYRO_CONFIG: bit4 HPF bypass + bit0 filter enable; ODR merges into bits[3:1]
constexpr uint8_t GYRO_CONFIG_BASE{0x11};
constexpr uint8_t FIFO_BYPASS{0x00};
constexpr uint8_t SET1_VALUE{0xC0};
}  // namespace

namespace m5 {
namespace unit {

const char UnitSH200Q::name[] = "UnitSH200Q";
const types::uid_t UnitSH200Q::uid{"UnitSH200Q"_mmh3};
const types::attr_t UnitSH200Q::attr{attribute::AccessI2C};

UnitSH200Q::UnitSH200Q(const uint8_t addr)
    : Component(addr), _data{new m5::container::CircularBuffer<sh200q::Data>(sh200q::DEFAULT_STORED_SIZE)}
{
    auto ccfg        = component_config();
    ccfg.clock       = 400 * 1000U;
    ccfg.stored_size = sh200q::DEFAULT_STORED_SIZE;
    component_config(ccfg);
}

bool UnitSH200Q::begin()
{
    // Re-entrant begin(): make sure _periodic and the hardware never get out of sync
    stop_periodic_measurement();

    auto ssize = stored_size();
    if (ssize != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<sh200q::Data>(ssize));
        if (!_data) {
            M5_LIB_LOGE("Failed to allocate");
            return false;
        }
    }
    _data->clear();
    _pushed = _consumed = 0;

    uint8_t id{};
    if (!readRegister8(REG_CHIP_ID, id, 0) || id != CHIP_ID_VALUE) {
        M5_LIB_LOGE("Not SH200Q %02X", id);
        return false;
    }

    if (!softReset()) {
        return false;
    }

    // Seed the live state from the begin settings; start_periodic_measurement() keeps it in step
    _accel_range = _cfg.accel_range;
    _gyro_range  = _cfg.gyro_range;
    _accel_odr   = _cfg.accel_odr;
    _gyro_odr    = _cfg.gyro_odr;
    _gyro_dlpf   = _cfg.gyro_dlpf;

    return _cfg.start_periodic ? startPeriodicMeasurement(_cfg.accel_range, _cfg.gyro_range, _cfg.accel_odr,
                                                          _cfg.gyro_odr, _cfg.gyro_dlpf)
                               : true;
}

bool UnitSH200Q::softReset()
{
    if (inPeriodic()) {
        M5_LIB_LOGE("Periodic measurements are running");
        return false;
    }
    // Undocumented init dance, mirrored verbatim from the original M5StickC library (see
    // unit_sh200q.hpp command namespace): ADC reset, then two more toggle/write rituals.
    if (!toggle_register_bits(REG_ADC_RESET, 0x04) || !toggle_register_bits(REG_INIT_D8, 0x80) ||
        !writeRegister8(REG_INIT_78, 0x61)) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }
    m5::utility::delay(1);
    if (!writeRegister8(REG_INIT_78, 0x00)) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }
    return true;
}

void UnitSH200Q::update(const bool force)
{
    _updated = false;
    if (inPeriodic()) {
        const auto at = m5::utility::millis();
        if (force || !_latest || at >= _latest + _interval) {
            sh200q::Data d{};
            _updated = read_measurement(d);
            if (_updated) {
                _data->push_back(d);
                ++_pushed;
                _latest = at;
            }
        }
    }
}

bool UnitSH200Q::start_periodic_measurement(const sh200q::AccelRange accel_range, const sh200q::GyroRange gyro_range,
                                            const sh200q::AccelOdr accel_odr, const sh200q::GyroOdr gyro_odr,
                                            const sh200q::GyroDlpf gyro_dlpf)
{
    if (inPeriodic()) {
        return false;
    }

    const uint8_t acc_cfg  = ACC_CONFIG_BASE | (static_cast<uint8_t>(accel_odr) << 3);
    const uint8_t gyro_cfg = GYRO_CONFIG_BASE | (static_cast<uint8_t>(gyro_odr) << 1);
    if (!write_register8_verified(REG_ACC_CONFIG, acc_cfg) || !write_register8_verified(REG_GYRO_CONFIG, gyro_cfg) ||
        !write_register8_verified(REG_GYRO_DLPF, static_cast<uint8_t>(gyro_dlpf)) ||
        !write_register8_verified(REG_FIFO_CONFIG, FIFO_BYPASS) || !writeAccelRange(accel_range) ||
        !writeGyroRange(gyro_range)) {
        M5_LIB_LOGE("Failed to initialize");
        return false;
    }

    // Final undocumented step (the original library's second "ADC reset"), then settle.
    if (!writeRegister8(REG_SET1, SET1_VALUE) || !toggle_register_bits(REG_SET2, 0x10)) {
        M5_LIB_LOGE("Failed to initialize");
        return false;
    }
    m5::utility::delay(10);

    // accel_range/gyro_range are tracked by writeAccelRange/writeGyroRange above
    _accel_odr = accel_odr;
    _gyro_odr  = gyro_odr;
    _gyro_dlpf = gyro_dlpf;

    _periodic = true;
    _latest   = 0;
    _interval = 1000U / sh200q::gyro_odr_hz(gyro_odr);
    return true;
}

bool UnitSH200Q::start_periodic_measurement()
{
    return start_periodic_measurement(_accel_range, _gyro_range, _accel_odr, _gyro_odr, _gyro_dlpf);
}

bool UnitSH200Q::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

bool UnitSH200Q::startPeriodicMeasurement(const sh200q::AccelRange accel_range, const sh200q::GyroRange gyro_range,
                                          const sh200q::AccelOdr accel_odr, const sh200q::GyroOdr gyro_odr,
                                          const sh200q::GyroDlpf gyro_dlpf)
{
    return PeriodicMeasurementAdapter<UnitSH200Q, sh200q::Data>::startPeriodicMeasurement(
        accel_range, gyro_range, accel_odr, gyro_odr, gyro_dlpf);
}

bool UnitSH200Q::startPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitSH200Q, sh200q::Data>::startPeriodicMeasurement();
}

bool UnitSH200Q::stopPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitSH200Q, sh200q::Data>::stopPeriodicMeasurement();
}

bool UnitSH200Q::writeAccelRange(const sh200q::AccelRange range)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    if (write_register8_verified(REG_ACC_RANGE, static_cast<uint8_t>(range))) {
        _accel_range = range;
        return true;
    }
    return false;
}

bool UnitSH200Q::writeGyroRange(const sh200q::GyroRange range)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    if (write_register8_verified(REG_GYRO_RANGE, static_cast<uint8_t>(range))) {
        _gyro_range = range;
        return true;
    }
    return false;
}

bool UnitSH200Q::readAccelRange(sh200q::AccelRange& range)
{
    uint8_t reg{};
    if (!readRegister8(REG_ACC_RANGE, reg, 0)) {
        return false;
    }
    range = static_cast<sh200q::AccelRange>(reg & 0x03);
    return true;
}

bool UnitSH200Q::readGyroRange(sh200q::GyroRange& range)
{
    uint8_t reg{};
    if (!readRegister8(REG_GYRO_RANGE, reg, 0)) {
        return false;
    }
    range = static_cast<sh200q::GyroRange>(reg & 0x07);
    return true;
}

bool UnitSH200Q::writeAccelOdr(const sh200q::AccelOdr odr)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    if (!write_register8_verified(REG_ACC_CONFIG, ACC_CONFIG_BASE | (static_cast<uint8_t>(odr) << 3))) {
        return false;
    }
    _accel_odr = odr;
    return true;
}

bool UnitSH200Q::writeGyroOdr(const sh200q::GyroOdr odr)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    if (!write_register8_verified(REG_GYRO_CONFIG, GYRO_CONFIG_BASE | (static_cast<uint8_t>(odr) << 1))) {
        return false;
    }
    _gyro_odr = odr;
    _interval     = 1000U / sh200q::gyro_odr_hz(odr);  // update() rate-limits on this
    return true;
}

bool UnitSH200Q::readAccelOdr(sh200q::AccelOdr& odr)
{
    uint8_t reg{};
    if (!readRegister8(REG_ACC_CONFIG, reg, 0)) {
        return false;
    }
    // bits[4:3], every one of the 4 values is a documented rate
    odr = static_cast<sh200q::AccelOdr>((reg >> 3) & 0x03);
    return true;
}

bool UnitSH200Q::readGyroOdr(sh200q::GyroOdr& odr)
{
    uint8_t reg{};
    if (!readRegister8(REG_GYRO_CONFIG, reg, 0)) {
        return false;
    }
    const uint8_t v = (reg >> 1) & 0x07;
    if (v > static_cast<uint8_t>(sh200q::GyroOdr::HZ250)) {
        return false;  // Reserved or a rate this driver does not model
    }
    odr = static_cast<sh200q::GyroOdr>(v);
    return true;
}

bool UnitSH200Q::read_measurement(sh200q::Data& d)
{
    // No data-ready flag is used: INT_STATUS behavior with interrupts disabled is undocumented,
    // and the original library reads unconditionally. The periodic interval paces the reads.
    return readRegister(REG_OUTPUT, d.raw.data(), d.raw.size(), 0);
}

bool UnitSH200Q::write_register8_verified(const uint8_t reg, const uint8_t val)
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

bool UnitSH200Q::toggle_register_bits(const uint8_t reg, const uint8_t bits)
{
    uint8_t val{};
    if (!readRegister8(reg, val, 0) || !writeRegister8(reg, val | bits)) {
        return false;
    }
    m5::utility::delay(1);
    return writeRegister8(reg, val & ~bits);
}

Capability UnitSH200Q::readImuRaw(m5::imu::imu_raw_data_t& out)
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

void UnitSH200Q::imuConvertParam(m5::imu::imu_convert_param_t& out) const
{
    out.accel_res   = accel_res(_accel_range);
    out.gyro_res    = gyro_res(_gyro_range);
    out.temp_res    = 1.0f / 333.87f;
    out.temp_offset = 21.0f;
}

}  // namespace unit
}  // namespace m5
