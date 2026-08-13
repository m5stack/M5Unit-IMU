/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6xxx.cpp
  @brief UnitMPU6xxx: shared base for the InvenSense MPU 6-axis family
*/
#include "unit_mpu6xxx.hpp"
#include <M5Utility.hpp>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::mpu6886;
using namespace m5::unit::mpu6886::command;
using m5::imu::Capability;

namespace {
constexpr uint32_t WRITE_VERIFY_RETRY{16};
// REG_PWR_MGMT_1 command values
// CLKSEL: 0,6 = internal 20MHz (alias) / 1..5 = auto PLL (alias) / 7 = stop
// Not modeled as an enum because of duplicate values (m5-cpp-rules). Use 1 (auto PLL) for full performance
constexpr uint8_t CLKSEL_AUTO_PLL{0x01};
constexpr uint8_t DEVICE_RESET{0x80};
constexpr uint8_t SLEEP_ENABLE{0x40};  // bit6 SLEEP, CLKSEL left at the internal oscillator
}  // namespace

namespace m5 {
namespace unit {

// class UnitMPU6xxx (abstract base; identity is shadowed by each concrete derived's own static members)
const char UnitMPU6xxx::name[] = "UnitMPU6xxx";
const types::uid_t UnitMPU6xxx::uid{"UnitMPU6xxx"_mmh3};
const types::attr_t UnitMPU6xxx::attr{attribute::AccessI2C};

UnitMPU6xxx::UnitMPU6xxx(const uint8_t addr)
    : Component(addr), _data{new m5::container::CircularBuffer<mpu6886::Data>(mpu6886::DEFAULT_STORED_SIZE)}
{
    auto ccfg        = component_config();
    ccfg.clock       = 400 * 1000U;
    ccfg.stored_size = mpu6886::DEFAULT_STORED_SIZE;
    component_config(ccfg);
}

bool UnitMPU6xxx::begin()
{
    // Re-entrant begin(): make sure _periodic and the hardware never get out of sync
    stop_periodic_measurement();

    auto ssize = stored_size();
    if (ssize != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<mpu6886::Data>(ssize));
        if (!_data) {
            M5_LIB_LOGE("Failed to allocate");
            return false;
        }
    }
    _data->clear();
    _pushed = _consumed = 0;

    uint8_t id{};
    // First-transaction NO_ACK insurance for SoftwareI2C hosts (NessoN1 SoftwareI2C /
    // NanoC6 & NanoH2 Ex_I2C, especially H2): retry the WHO_AM_I read a few times before
    // failing. Cannot reorder to write-first here because address 0x68 is shared by many
    // IMUs (ICM-20xxx etc.); a bare soft-reset write could corrupt whichever chip answers.
    bool who_ok{false};
    for (int i = 0; i < 3; ++i) {
        if (readRegister8(REG_WHO_AM_I, id, 0) && accepts_whoami(id)) {
            who_ok = true;
            break;
        }
        m5::utility::delay(1);
    }
    if (!who_ok) {
        M5_LIB_LOGE("WHO_AM_I mismatch %02X", id);
        return false;
    }
    if (!softReset()) {
        return false;
    }

    // Seed the live state from the begin settings; start_periodic_measurement() keeps it in step
    _accel_range     = _cfg.accel_range;
    _gyro_range      = _cfg.gyro_range;
    _dlpf            = _cfg.dlpf;
    _sample_rate_div = _cfg.sample_rate_div;

    return _cfg.start_periodic
               ? startPeriodicMeasurement(_cfg.accel_range, _cfg.gyro_range, _cfg.dlpf, _cfg.sample_rate_div)
               : true;
}

bool UnitMPU6xxx::softReset()
{
    if (inPeriodic()) {
        M5_LIB_LOGE("Periodic measurements are running");
        return false;
    }
    if (!writeRegister8(REG_PWR_MGMT_1, DEVICE_RESET)) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }
    m5::utility::delay(10);
    return true;
}

void UnitMPU6xxx::update(const bool force)
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

bool UnitMPU6xxx::start_periodic_measurement(const mpu6886::AccelRange accel_range, const mpu6886::GyroRange gyro_range,
                                             const mpu6886::Dlpf dlpf, const uint8_t sample_rate_div)
{
    if (inPeriodic()) {
        return false;
    }

    // Wake from the post-reset sleep (CLKSEL auto PLL), then apply the measurement settings
    if (!write_register8_verified(REG_PWR_MGMT_1, CLKSEL_AUTO_PLL) || !writeAccelRange(accel_range) ||
        !writeGyroRange(gyro_range) || !write_register8_verified(REG_CONFIG, static_cast<uint8_t>(dlpf)) ||
        !write_register8_verified(REG_SMPLRT_DIV, sample_rate_div) ||
        !write_register8_verified(REG_INT_PIN_CFG, 0xC0) || !write_register8_verified(REG_INT_ENABLE, 0x00) ||
        !write_register8_verified(REG_ACCEL_CONFIG2, 0x00) || !write_register8_verified(REG_USER_CTRL, 0x00) ||
        !write_register8_verified(REG_FIFO_EN, 0x00)) {
        M5_LIB_LOGE("Failed to initialize");
        return false;
    }
    m5::utility::delay(50);  // Wait for gyro (typ 35ms) / accel (typ 10ms) startup

    // accel_range/gyro_range are tracked by writeAccelRange/writeGyroRange above
    _dlpf            = dlpf;
    _sample_rate_div = sample_rate_div;

    _periodic = true;
    _latest   = 0;
    _interval = 1 + sample_rate_div;
    return true;
}

bool UnitMPU6xxx::start_periodic_measurement()
{
    return start_periodic_measurement(_accel_range, _gyro_range, _dlpf, _sample_rate_div);
}

bool UnitMPU6xxx::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

bool UnitMPU6xxx::startPeriodicMeasurement(const mpu6886::AccelRange accel_range, const mpu6886::GyroRange gyro_range,
                                           const mpu6886::Dlpf dlpf, const uint8_t sample_rate_div)
{
    return PeriodicMeasurementAdapter<UnitMPU6xxx, mpu6886::Data>::startPeriodicMeasurement(accel_range, gyro_range,
                                                                                            dlpf, sample_rate_div);
}

bool UnitMPU6xxx::startPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitMPU6xxx, mpu6886::Data>::startPeriodicMeasurement();
}

bool UnitMPU6xxx::stopPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitMPU6xxx, mpu6886::Data>::stopPeriodicMeasurement();
}

bool UnitMPU6xxx::sleep()
{
    // Explicitly disable motion events; the user must re-enable them after wakeup().
    disableAllEvents();

    // A sleeping chip must not be polled, so stop measuring first (unlike softReset(), sleeping
    // during periodic measurement is allowed)
    if (inPeriodic() && !stopPeriodicMeasurement()) {
        M5_LIB_LOGE("Failed to stop before sleep");
        return false;
    }
    if (!writeRegister8(REG_PWR_MGMT_1, SLEEP_ENABLE)) {
        M5_LIB_LOGE("Failed to sleep");
        return false;
    }
    return true;
}

bool UnitMPU6xxx::wakeup()
{
    if (!write_register8_verified(REG_PWR_MGMT_1, CLKSEL_AUTO_PLL)) {
        M5_LIB_LOGE("Failed to wake");
        return false;
    }
    m5::utility::delay(50);  // Wait for gyro (typ 35ms) / accel (typ 10ms) startup
    return true;
}

bool UnitMPU6xxx::writeAccelRange(const mpu6886::AccelRange range)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    if (write_register8_verified(REG_ACCEL_CONFIG, static_cast<uint8_t>(range) << 3)) {
        _accel_range = range;
        return true;
    }
    return false;
}

bool UnitMPU6xxx::writeGyroRange(const mpu6886::GyroRange range)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    if (write_register8_verified(REG_GYRO_CONFIG, static_cast<uint8_t>(range) << 3)) {
        _gyro_range = range;
        return true;
    }
    return false;
}

bool UnitMPU6xxx::readAccelRange(mpu6886::AccelRange& range)
{
    uint8_t reg{};
    if (!readRegister8(REG_ACCEL_CONFIG, reg, 0)) {
        return false;
    }
    range = static_cast<mpu6886::AccelRange>((reg >> 3) & 0x03);
    return true;
}

bool UnitMPU6xxx::readGyroRange(mpu6886::GyroRange& range)
{
    uint8_t reg{};
    if (!readRegister8(REG_GYRO_CONFIG, reg, 0)) {
        return false;
    }
    range = static_cast<mpu6886::GyroRange>((reg >> 3) & 0x03);
    return true;
}

bool UnitMPU6xxx::writeSampleRateDiv(const uint8_t div)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    if (!write_register8_verified(REG_SMPLRT_DIV, div)) {
        return false;
    }
    _sample_rate_div = div;
    _interval        = 1 + div;  // update() rate-limits on this
    return true;
}

bool UnitMPU6xxx::readSampleRateDiv(uint8_t& div)
{
    return readRegister8(REG_SMPLRT_DIV, div, 0);
}

bool UnitMPU6xxx::writeDlpf(const mpu6886::Dlpf dlpf)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    if (!write_register8_verified(REG_CONFIG, static_cast<uint8_t>(dlpf))) {
        return false;
    }
    _dlpf = dlpf;
    return true;
}

bool UnitMPU6xxx::readDlpf(mpu6886::Dlpf& dlpf)
{
    uint8_t reg{};
    if (!readRegister8(REG_CONFIG, reg, 0)) {
        return false;
    }
    dlpf = static_cast<mpu6886::Dlpf>(reg & 0x07);  // bits[2:0], all 8 values are documented
    return true;
}

bool UnitMPU6xxx::read_measurement(mpu6886::Data& d)
{
    uint8_t status{};
    if (!readRegister8(REG_INT_STATUS, status, 0)) {
        M5_LIB_LOGE("Failed to read REG_INT_STATUS");
        return false;
    }
    if (!(status & 0x01)) {
        return false;  // Not ready yet (normal case, no log)
    }
    return readRegister(REG_ACCEL_XOUT_H, d.raw.data(), d.raw.size(), 0);
}

bool UnitMPU6xxx::write_register8_verified(const uint8_t reg, const uint8_t val)
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

Capability UnitMPU6xxx::readImuRaw(m5::imu::imu_raw_data_t& out)
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

void UnitMPU6xxx::imuConvertParam(m5::imu::imu_convert_param_t& out) const
{
    out.accel_res   = accel_res(_accel_range);
    out.gyro_res    = gyro_res(_gyro_range);
    out.temp_res    = temperature_res();
    out.temp_offset = temperature_offset();
}

}  // namespace unit
}  // namespace m5
