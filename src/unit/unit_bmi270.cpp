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
// REG_PWR_CONF must read 0 within 16ms of soft reset (datasheet-cited retry count, 1ms steps)
constexpr uint32_t PWR_CONF_POLL_RETRY{16};
// REG_INTERNAL_STATUS.message must reach init_ok (0x01) within ~20ms of REG_INIT_CTRL=0x01 (1ms steps)
constexpr uint32_t INTERNAL_STATUS_POLL_RETRY{20};

// Chunk size (bytes) for the config blob burst write to REG_INIT_DATA (0x5E).
// Rationale: AdapterI2C::WireImpl::writeWithTransaction() (m5_unit_component/adapter_i2c.cpp)
// buffers the register address byte plus the payload in a single Wire transaction before
// endTransmission() actually clocks it out, and arduino-esp32's TwoWire defaults
// I2C_BUFFER_LENGTH to 128 bytes (Wire.h). 64 bytes of payload (+1 register address byte =
// 65 bytes) stays well under that default regardless of which bus backend is active
// (SoftwareI2C bit-bangs a byte at a time and has no such limit), and divides the 8192 byte
// blob evenly (128 chunks) while staying word (2 byte) aligned for the REG_INIT_ADDR_0/1 addressing
// scheme below.
constexpr size_t CONFIG_CHUNK_SIZE{64};


// REG_CMD command value (REG_CHIP_ID expected value is bmi270::CHIP_ID, a namespace-scope constant)
constexpr uint8_t CMD_SOFTRESET{0xB6};


// REG_INT_STATUS_1 (0x1D) bits
constexpr uint8_t INT_STATUS_1_GYR_DRDY{0x40};  // bit6
constexpr uint8_t INT_STATUS_1_ACC_DRDY{0x80};  // bit7
constexpr uint8_t INT_STATUS_1_AUX_DRDY{0x20};  // bit5 (not used to gate reads; BMM150's own 30Hz update
                                                // rate means a stale AUX sample is simply re-read as-is)

// REG_INTERNAL_STATUS (0x21) message field, bits[3:0]. Not enum'd: 0x00 (not_init) is a
// transient polling state, not a selectable setting (m5-cpp-rules)
constexpr uint8_t INTERNAL_STATUS_MESSAGE_MASK{0x0F};
constexpr uint8_t INTERNAL_STATUS_INIT_OK{0x01};

// REG_PWR_CTRL (0x7D): temp_en | acc_en | gyr_en (aux_en added separately once AUX probing succeeds)
constexpr uint8_t PWR_CTRL_ACC_GYR_TEMP_ON{0x0E};
constexpr uint8_t PWR_CTRL_ALL_OFF{0x00};  // Suspend: every sensor domain disabled

// REG_PWR_CONF (0x7C)
constexpr uint8_t PWR_CONF_ADV_POWER_SAVE{0x01};  // bit0; must be cleared before configuration writes
constexpr uint8_t PWR_CONF_ACTIVE{0x00};



#include "unit_bmi270_config.inl"

}  // namespace

namespace m5 {
namespace unit {

// class UnitBMI270
const char UnitBMI270::name[] = "UnitBMI270";
const types::uid_t UnitBMI270::uid{"UnitBMI270"_mmh3};
const types::attr_t UnitBMI270::attr{attribute::AccessI2C};

UnitBMI270::UnitBMI270(const uint8_t addr)
    : Component(addr), _data{new m5::container::CircularBuffer<bmi270::Data>(bmi270::DEFAULT_STORED_SIZE)}
{
    auto ccfg        = component_config();
    ccfg.clock       = 400 * 1000U;
    ccfg.stored_size = bmi270::DEFAULT_STORED_SIZE;
    component_config(ccfg);
}

bool UnitBMI270::begin()
{
    // Re-entrant begin(): make sure _periodic and the hardware never get out of sync
    stop_periodic_measurement();

    auto ssize = stored_size();
    if (ssize != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<bmi270::Data>(ssize));
        if (!_data) {
            M5_LIB_LOGE("Failed to allocate");
            return false;
        }
    }
    _data->clear();
    _pushed = _consumed = 0;
    _last_temp          = 0;

    uint8_t id{};
    // After power-up the BMI270 needs one dummy transaction to establish its I2C interface; the
    // first register read may return stale data. Discard one read before validating CHIP_ID
    // (mirrors M5Unified's BMI270_Class, which reads WhoAmI twice).
    readRegister8(REG_CHIP_ID, id, 0);
    if (!readRegister8(REG_CHIP_ID, id, 0) || id != CHIP_ID) {
        M5_LIB_LOGE("Not BMI270 %02X", id);
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

    return _cfg.start_periodic ? startPeriodicMeasurement(_cfg.accel_range, _cfg.gyro_range, _cfg.accel_odr,
                                                          _cfg.gyro_odr, _cfg.accel_filter_perf, _cfg.accel_bwp,
                                                          _cfg.gyro_filter_perf, _cfg.gyro_bwp)
                              : true;
}

bool UnitBMI270::softReset()
{
    if (inPeriodic()) {
        M5_LIB_LOGE("Periodic measurements are running");
        return false;
    }
    if (!writeRegister8(REG_CMD, CMD_SOFTRESET)) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }

    // The soft reset reboots the device (~2 ms) during which register reads NAK or return 0.
    // Poll PWR_CONF, tolerating those transient failures, until it reads its non-zero reset
    // default -- i.e. the interface is back (mirrors M5Unified's BMI270_Class::begin()).
    uint8_t pwr_conf{};
    uint32_t retry = PWR_CONF_POLL_RETRY;
    do {
        m5::utility::delay(1);
        if (!readRegister8(REG_PWR_CONF, pwr_conf, 0)) {
            pwr_conf = 0;
        }
    } while (pwr_conf == 0 && --retry);
    if (pwr_conf == 0) {
        M5_LIB_LOGE("BMI270 did not respond after soft reset");
        return false;
    }

    // Mandatory post-reset initialization: the reset erases the config blob, so re-upload it
    // and wait for the internal engine to report init_ok.
    if (!writeRegister8(REG_PWR_CONF, 0x00)) {
        M5_LIB_LOGE("Failed to disable advanced power save");
        return false;
    }
    m5::utility::delay(1);  // >= 450us required before further register access

    if (!writeRegister8(REG_INIT_CTRL, 0x00)) {
        M5_LIB_LOGE("Failed to begin config load");
        return false;
    }
    if (!upload_config()) {
        M5_LIB_LOGE("Failed to upload config");
        return false;
    }
    if (!writeRegister8(REG_INIT_CTRL, 0x01)) {
        M5_LIB_LOGE("Failed to finalize config load");
        return false;
    }

    uint8_t status{};
    retry = INTERNAL_STATUS_POLL_RETRY;
    for (;;) {
        m5::utility::delay(1);
        if (!readRegister8(REG_INTERNAL_STATUS, status, 0)) {
            status = 0;  // transient NAK while the device settles; keep polling
        }
        const uint8_t msg = status & INTERNAL_STATUS_MESSAGE_MASK;
        if (msg == INTERNAL_STATUS_INIT_OK) {
            break;
        }
        if (msg != 0x00) {
            M5_LIB_LOGE("BMI270 init error (REG_INTERNAL_STATUS:%02X)", status);
            return false;
        }
        if (--retry == 0) {
            M5_LIB_LOGE("BMI270 init timeout (REG_INTERNAL_STATUS:%02X)", status);
            return false;
        }
    }
    return true;
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

bool UnitBMI270::start_periodic_measurement(const bmi270::AccelRange accel_range, const bmi270::GyroRange gyro_range,
                                            const bmi270::AccelODR accel_odr, const bmi270::GyroODR gyro_odr,
                                            const bmi270::FilterPerf accel_filter_perf,
                                            const bmi270::AccelBwp accel_bwp,
                                            const bmi270::FilterPerf gyro_filter_perf, const bmi270::GyroBwp gyro_bwp)
{
    if (inPeriodic()) {
        return false;
    }

    // The accelerometer's averaging-only bandwidth values are reserved while the filter runs in
    // Performance mode, so refuse that pair instead of writing a reserved code to the register
    if (!bwp_fits(accel_filter_perf, accel_bwp)) {
        M5_LIB_LOGE("accel_bwp 0x%02X is reserved while accel_filter_perf is Performance",
                    static_cast<uint8_t>(accel_bwp));
        return false;
    }

    if (!write_register8_verified(REG_PWR_CTRL, PWR_CTRL_ACC_GYR_TEMP_ON) ||
        !write_register8_verified(REG_ACC_CONF, acc_conf_value(accel_filter_perf, accel_bwp, accel_odr)) ||
        !writeAccelRange(accel_range) ||
        !write_register8_verified(REG_GYR_CONF, gyr_conf_value(gyro_filter_perf, gyro_bwp, gyro_odr)) ||
        !writeGyroRange(gyro_range)) {
        M5_LIB_LOGE("Failed to configure");
        return false;
    }

    // accel_range/gyro_range are tracked by writeAccelRange/writeGyroRange above
    _accel_odr         = accel_odr;
    _gyro_odr          = gyro_odr;
    _accel_filter_perf = accel_filter_perf;
    _accel_bwp         = accel_bwp;
    _gyro_filter_perf  = gyro_filter_perf;
    _gyro_bwp          = gyro_bwp;

    _periodic = true;
    _latest   = 0;
    _interval = odr_interval_ms(gyro_odr);
    return true;
}

bool UnitBMI270::start_periodic_measurement()
{
    return start_periodic_measurement(_accel_range, _gyro_range, _accel_odr, _gyro_odr, _accel_filter_perf,
                                      _accel_bwp, _gyro_filter_perf, _gyro_bwp);
}

bool UnitBMI270::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

bool UnitBMI270::startPeriodicMeasurement(const bmi270::AccelRange accel_range, const bmi270::GyroRange gyro_range,
                                          const bmi270::AccelODR accel_odr, const bmi270::GyroODR gyro_odr,
                                          const bmi270::FilterPerf accel_filter_perf, const bmi270::AccelBwp accel_bwp,
                                          const bmi270::FilterPerf gyro_filter_perf, const bmi270::GyroBwp gyro_bwp)
{
    return PeriodicMeasurementAdapter<UnitBMI270, bmi270::Data>::startPeriodicMeasurement(
        accel_range, gyro_range, accel_odr, gyro_odr, accel_filter_perf, accel_bwp, gyro_filter_perf, gyro_bwp);
}

bool UnitBMI270::startPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitBMI270, bmi270::Data>::startPeriodicMeasurement();
}

bool UnitBMI270::stopPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitBMI270, bmi270::Data>::stopPeriodicMeasurement();
}

bool UnitBMI270::sleep()
{
    // A suspended chip must not be polled, so stop measuring first (unlike softReset(),
    // suspending during periodic measurement is allowed)
    if (inPeriodic() && !stopPeriodicMeasurement()) {
        M5_LIB_LOGE("Failed to stop before sleep");
        return false;
    }
    if (!writeRegister8(REG_PWR_CTRL, PWR_CTRL_ALL_OFF) ||
        !writeRegister8(REG_PWR_CONF, PWR_CONF_ADV_POWER_SAVE)) {
        M5_LIB_LOGE("Failed to suspend");
        return false;
    }
    return true;
}

bool UnitBMI270::wakeup()
{
    // Configuration writes are ignored while advanced power save is active, so clear it first
    if (!writeRegister8(REG_PWR_CONF, PWR_CONF_ACTIVE)) {
        M5_LIB_LOGE("Failed to leave power save");
        return false;
    }
    m5::utility::delay(5);
    if (!write_register8_verified(REG_PWR_CTRL, PWR_CTRL_ACC_GYR_TEMP_ON)) {
        M5_LIB_LOGE("Failed to wake");
        return false;
    }
    return true;
}

bool UnitBMI270::writeAccelRange(const bmi270::AccelRange range)
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

bool UnitBMI270::writeGyroRange(const bmi270::GyroRange range)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    if (write_register8_verified(REG_GYR_RANGE, static_cast<uint8_t>(range))) {
        _gyro_range = range;
        return true;
    }
    return false;
}

bool UnitBMI270::readAccelRange(bmi270::AccelRange& range)
{
    uint8_t reg{};
    if (!readRegister8(REG_ACC_RANGE, reg, 0)) {
        return false;
    }
    range = static_cast<bmi270::AccelRange>(reg & 0x03);
    return true;
}

bool UnitBMI270::readGyroRange(bmi270::GyroRange& range)
{
    uint8_t reg{};
    if (!readRegister8(REG_GYR_RANGE, reg, 0)) {
        return false;
    }
    const uint8_t val = reg & 0x07;
    if (val > static_cast<uint8_t>(bmi270::GyroRange::DPS125)) {
        return false;  // 5..7 are reserved (unmapped) values
    }
    range = static_cast<bmi270::GyroRange>(val);
    return true;
}

bool UnitBMI270::writeAccelODR(const bmi270::AccelODR odr)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    // The rate shares REG_ACC_CONF with the bandwidth and filter performance, so the whole register
    // is recomposed from the configuration rather than read-modify-written
    if (!write_register8_verified(REG_ACC_CONF, acc_conf_value(_accel_filter_perf, _accel_bwp, odr))) {
        return false;
    }
    _accel_odr = odr;
    return true;
}

bool UnitBMI270::writeGyroODR(const bmi270::GyroODR odr)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    if (!write_register8_verified(REG_GYR_CONF, gyr_conf_value(_gyro_filter_perf, _gyro_bwp, odr))) {
        return false;
    }
    _gyro_odr = odr;
    _interval     = odr_interval_ms(odr);  // update() rate-limits on this
    return true;
}

bool UnitBMI270::readAccelODR(bmi270::AccelODR& odr)
{
    uint8_t reg{};
    if (!readRegister8(REG_ACC_CONF, reg, 0)) {
        return false;
    }
    const uint8_t v = reg & 0x0F;
    if (v < static_cast<uint8_t>(bmi270::AccelODR::Rate12_5Hz) ||
        v > static_cast<uint8_t>(bmi270::AccelODR::Rate1600Hz)) {
        return false;  // Reserved or a rate this driver does not model
    }
    odr = static_cast<bmi270::AccelODR>(v);
    return true;
}

bool UnitBMI270::readGyroODR(bmi270::GyroODR& odr)
{
    uint8_t reg{};
    if (!readRegister8(REG_GYR_CONF, reg, 0)) {
        return false;
    }
    const uint8_t v = reg & 0x0F;
    if (v < static_cast<uint8_t>(bmi270::GyroODR::Rate25Hz) || v > static_cast<uint8_t>(bmi270::GyroODR::Rate3200Hz)) {
        return false;  // Reserved or a rate this driver does not model
    }
    odr = static_cast<bmi270::GyroODR>(v);
    return true;
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
        if (!writeRegister(REG_INIT_ADDR_0, addr, 2) ||
            !writeRegister(REG_INIT_DATA, bmi270_config_file + index, len)) {
            M5_LIB_LOGE("Failed to upload config at offset %u", static_cast<unsigned int>(index));
            return false;
        }
    }
    return true;
}

bool UnitBMI270::read_sensor_burst(bmi270::Data& d)
{
    if (!readRegister(REG_DATA_8, d.raw.data(), d.raw.size(), 0)) {
        M5_LIB_LOGE("Failed to read data");
        return false;
    }
    d.aux_mag = false;
    return true;
}

bool UnitBMI270::read_measurement(bmi270::Data& d)
{
    // The BMI270 data registers always hold the most recent sample, so read them directly at the
    // caller's cadence -- update() already rate-limits to the configured ODR. This mirrors
    // M5Unified's BMI270_Class::getAccelAdc(), which likewise reads without a data-ready gate
    // (INT_STATUS_1's drdy bits would require INT_MAP_DATA, and STATUS polling is unnecessary here).
    if (!read_sensor_burst(d)) {
        return false;
    }

    // Temperature is a separate, lower rate register (updates every 10ms in gyro normal
    // mode); a failed or invalid read simply keeps the previous _last_temp, it does not
    // fail the whole measurement.
    uint8_t traw[2]{};
    if (readRegister(REG_TEMPERATURE_0, traw, sizeof(traw), 0)) {
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
    if (d->aux_mag) {
        out.mag[0] = d->mag.adj_x();
        out.mag[1] = d->mag.adj_y();
        out.mag[2] = d->mag.adj_z();
    }
    return imuSpec();
}

void UnitBMI270::imuConvertParam(m5::imu::imu_convert_param_t& out) const
{
    out.accel_res   = accel_res(_accel_range);
    out.gyro_res    = gyro_res(_gyro_range);
    out.temp_res    = 1.0f / 512.0f;
    out.temp_offset = 23.0f;
}

}  // namespace unit
}  // namespace m5
