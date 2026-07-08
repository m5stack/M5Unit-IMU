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

// STATUS.aux_busy must clear within this many 1ms polls (datasheet-cited AUX transaction timing)
constexpr uint32_t AUX_BUSY_POLL_RETRY{3};
// BMM150 soft-reset settling time before it responds to AUX reads (datasheet 3.0ms)
constexpr uint32_t AUX_RESET_DELAY_MS{3};

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

    _aux_mag = _cfg.enable_aux_bmm150 && probe_aux_bmm150();
    if (_cfg.enable_aux_bmm150 && !_aux_mag) {
        // No BMM150 responded on the AUX interface (normal on boards without one):
        // disable the AUX interface and continue with 6-axis (accel/gyro/temp) only.
        writeRegister8(IF_CONF, 0x00);
        M5_LIB_LOGI("No AUX BMM150 detected; continuing with 6-axis (accel/gyro/temp) only");
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

bool UnitBMI270::probe_aux_bmm150()
{
    // Setup mode: enable the AUX interface and switch it to manual (single-register) access
    // so we can talk to the BMM150 directly (mirrors M5Unified's BMI270_Class::auxSetupMode()).
    if (!writeRegister8(IF_CONF, IF_CONF_AUX_EN) || !writeRegister8(PWR_CONF, 0x00) ||
        !writeRegister8(PWR_CTRL, PWR_CTRL_ACC_GYR_TEMP_ON) || !writeRegister8(AUX_IF_CONF, AUX_IF_CONF_MANUAL_BL1) ||
        !writeRegister8(AUX_DEV_ID, AUX_DEV_ID_BMM150)) {
        M5_LIB_LOGE("Failed to configure the AUX interface");
        return false;
    }

    if (!aux_write_register8(BMM150_POWER_CONTROL, BMM150_SOFT_RESET_AND_POWER_ON)) {
        M5_LIB_LOGE("Failed to reset the AUX BMM150");
        return false;
    }
    m5::utility::delay(AUX_RESET_DELAY_MS);

    // The first read right after a fresh AUX target reset returns a stale value
    // (M5Unified BMI270_Class.cpp:76-77); discard it before checking the chip ID.
    uint8_t who{};
    if (!aux_read_register8(BMM150_CHIP_ID, who) || !aux_read_register8(BMM150_CHIP_ID, who) ||
        who != BMM150_CHIP_ID_VALUE) {
        return false;  // No AUX BMM150 wired on this board (normal case, no log)
    }

    if (!aux_read_trim(_mag_trim)) {
        M5_LIB_LOGE("Failed to read the AUX BMM150 trim registers");
        return false;
    }

    if (!aux_write_register8(BMM150_OP_MODE, BMM150_OP_MODE_NORMAL_ODR30HZ) ||
        !write_register8_verified(AUX_IF_CONF, AUX_IF_CONF_DATA_MODE) ||
        !write_register8_verified(AUX_RD_ADDR, BMM150_DATA_X_LSB) ||
        !write_register8_verified(PWR_CTRL, PWR_CTRL_ACC_GYR_TEMP_AUX_ON)) {
        M5_LIB_LOGE("Failed to switch the AUX BMM150 to data mode");
        return false;
    }
    return true;
}

bool UnitBMI270::aux_wait_ready()
{
    for (uint32_t retry = 0; retry < AUX_BUSY_POLL_RETRY; ++retry) {
        uint8_t status{};
        if (!readRegister8(STATUS_REG, status, 0)) {
            return false;
        }
        if ((status & STATUS_AUX_BUSY) == 0) {
            return true;
        }
        m5::utility::delay(1);
    }
    return false;
}

bool UnitBMI270::aux_write_register8(const uint8_t reg, const uint8_t val)
{
    return writeRegister8(AUX_WR_DATA, val) && writeRegister8(AUX_WR_ADDR, reg) && aux_wait_ready();
}

bool UnitBMI270::aux_read_register8(const uint8_t reg, uint8_t& out)
{
    if (!writeRegister8(AUX_IF_CONF, AUX_IF_CONF_MANUAL_BL1) || !writeRegister8(AUX_RD_ADDR, reg) ||
        !aux_wait_ready()) {
        return false;
    }
    return readRegister8(DATA_0, out, 0);
}

bool UnitBMI270::aux_read_burst8(const uint8_t bmm150_reg, std::array<uint8_t, 8>& out)
{
    if (!writeRegister8(AUX_RD_ADDR, bmm150_reg) || !aux_wait_ready()) {
        return false;
    }
    return readRegister(DATA_0, out.data(), out.size(), 0);
}

bool UnitBMI270::aux_read_trim(bmm150::trim_t& trim)
{
    // BMM150 trim occupies registers 0x5D-0x71 (21 bytes), but AUX manual burst reads are
    // limited to 1/2/6/8 bytes. Three overlapping 8 byte bursts starting at 0x5D/0x65/0x6A
    // cover the whole range; overlapping bytes are simply overwritten with identical values.
    constexpr uint8_t TRIM_FIRST  = 0x5D;
    constexpr uint8_t TRIM_SECOND = 0x65;
    constexpr uint8_t TRIM_THIRD  = 0x6A;
    constexpr size_t TRIM_BYTES   = 0x71 - TRIM_FIRST + 1;  // 21

    if (!writeRegister8(AUX_IF_CONF, AUX_IF_CONF_MANUAL_BL8)) {
        return false;
    }

    std::array<uint8_t, 8> w0{}, w1{}, w2{};
    if (!aux_read_burst8(TRIM_FIRST, w0) || !aux_read_burst8(TRIM_SECOND, w1) || !aux_read_burst8(TRIM_THIRD, w2)) {
        return false;
    }

    // Reassemble into the 0x5D..0x71 layout, indexed by (register - TRIM_FIRST), the same
    // way UnitBMM150::read_trim() consumes its own (non-AUX) burst reads.
    std::array<uint8_t, TRIM_BYTES> t{};
    std::copy(w0.begin(), w0.end(), t.begin());
    std::copy(w1.begin(), w1.end(), t.begin() + (TRIM_SECOND - TRIM_FIRST));
    std::copy(w2.begin(), w2.end(), t.begin() + (TRIM_THIRD - TRIM_FIRST));

    trim.dig_x1   = static_cast<int8_t>(t[0x5D - TRIM_FIRST]);
    trim.dig_y1   = static_cast<int8_t>(t[0x5E - TRIM_FIRST]);
    trim.dig_z4   = static_cast<int16_t>((t[0x63 - TRIM_FIRST] << 8) | t[0x62 - TRIM_FIRST]);
    trim.dig_x2   = static_cast<int8_t>(t[0x64 - TRIM_FIRST]);
    trim.dig_y2   = static_cast<int8_t>(t[0x65 - TRIM_FIRST]);
    trim.dig_z2   = static_cast<int16_t>((t[0x69 - TRIM_FIRST] << 8) | t[0x68 - TRIM_FIRST]);
    trim.dig_z1   = static_cast<uint16_t>((t[0x6B - TRIM_FIRST] << 8) | t[0x6A - TRIM_FIRST]);
    trim.dig_xyz1 = static_cast<uint16_t>(((t[0x6D - TRIM_FIRST] << 8) | t[0x6C - TRIM_FIRST]) & 0x3FFF);
    trim.dig_z3   = static_cast<int16_t>((t[0x6F - TRIM_FIRST] << 8) | t[0x6E - TRIM_FIRST]);
    trim.dig_xy2  = static_cast<int8_t>(t[0x70 - TRIM_FIRST]);
    trim.dig_xy1  = t[0x71 - TRIM_FIRST];
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

    if (_aux_mag) {
        // AUX data mode auto-polls the BMM150 into DATA_0..DATA_7 (mag+rhall) followed by
        // the usual accel/gyro at DATA_8..DATA_19; one 20 byte burst covers all three.
        std::array<uint8_t, 20> buf{};
        if (!readRegister(DATA_0, buf.data(), buf.size(), 0)) {
            M5_LIB_LOGE("Failed to read data");
            return false;
        }
        std::copy(buf.begin(), buf.begin() + d.mag.raw.size(), d.mag.raw.begin());
        std::copy(buf.begin() + d.mag.raw.size(), buf.end(), d.raw.begin());
        d.mag.trim = _mag_trim;
        d.aux_mag  = true;
    } else {
        if (!readRegister(DATA_8, d.raw.data(), d.raw.size(), 0)) {
            M5_LIB_LOGE("Failed to read data");
            return false;
        }
        d.aux_mag = false;
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
    if (d->aux_mag) {
        out.mag[0] = d->mag.comp_x();
        out.mag[1] = d->mag.comp_y();
        out.mag[2] = d->mag.comp_z();
    }
    return imuSpec();
}

void UnitBMI270::imuConvertParam(m5::imu::imu_convert_param_t& out) const
{
    out.accel_res   = accel_res(_cfg.accel_range);
    out.gyro_res    = gyro_res(_cfg.gyro_range);
    out.temp_res    = 1.0f / 512.0f;
    out.temp_offset = 23.0f;
    if (_aux_mag) {
        out.mag_res = 1.0f / 16.0f;  // Compensated BMM150 output, matches UnitBMM150's scale
    }
}

}  // namespace unit
}  // namespace m5
