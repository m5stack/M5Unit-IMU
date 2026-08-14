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

// ANYMO_1 (word0, REG_FEATURES_ANY_MOTION): duration[12:0], select_x[13], select_y[14], select_z[15]
// (BMI270 datasheet section 4.8.2 / register table, address 0x3C, page 1)
constexpr uint16_t ANYMO1_DURATION_MASK{0x1FFF};
constexpr uint16_t ANYMO1_SELECT_X_BIT{1u << 13};
constexpr uint16_t ANYMO1_SELECT_Y_BIT{1u << 14};
constexpr uint16_t ANYMO1_SELECT_Z_BIT{1u << 15};

// ANYMO_2 (word1, REG_FEATURES_ANY_MOTION_2): threshold[10:0], out_conf[14:11], enable[15]
// (same datasheet reference, address 0x3E)
constexpr uint16_t ANYMO2_THRESHOLD_MASK{0x07FF};
// out_conf value 0x7 routes any_motion_out onto INT_STATUS_0 bit 6 (its documented power-on
// default); forced explicitly here so it does not depend on what the config blob left it at
constexpr uint16_t ANYMO2_OUT_CONF_BIT6{0x07u << 11};
constexpr uint16_t ANYMO2_ENABLE_BIT{1u << 15};

// NOMO_1 (word0, BMI270_FEATURE_NO_MOTION_OFS_WORDS): duration[12:0], select_x[13], select_y[14],
// select_z[15] (BMI270 datasheet section 4.8.3 / register table, address 0x30, page 2). Same bit
// layout as ANYMO_1 above.
constexpr uint16_t NOMO1_DURATION_MASK{0x1FFF};
constexpr uint16_t NOMO1_SELECT_X_BIT{1u << 13};
constexpr uint16_t NOMO1_SELECT_Y_BIT{1u << 14};
constexpr uint16_t NOMO1_SELECT_Z_BIT{1u << 15};

// NOMO_2 (word1, BMI270_FEATURE_NO_MOTION_2_OFS_WORDS): threshold[10:0], out_conf[14:11],
// enable[15] (same datasheet reference, address 0x32). Unlike ANYMO_2 above, out_conf is preserved
// rather than forced -- see write_no_motion_features().
constexpr uint16_t NOMO2_THRESHOLD_MASK{0x07FF};
constexpr uint16_t NOMO2_OUT_CONF_MASK{0x0Fu << 11};
constexpr uint16_t NOMO2_ENABLE_BIT{1u << 15};

// SIGMO_1 (word0, BMI270_FEATURE_SIG_MOTION_OFS_WORDS): block_size occupies the full 16 bits --
// unlike ANYMO_1/NOMO_1 above, there is no per-axis select field here (BMI270 datasheet section
// 4.8.4 / register table, address 0x34, page 2)
constexpr uint16_t SIGMO1_BLOCK_SIZE_MASK{0xFFFF};

// SIGMO_2 (word, BMI270_FEATURE_SIG_MOTION_2_OFS_WORDS): enable[0], out_conf[4:1] -- a different
// bit layout from ANYMO_2/NOMO_2 above, whose enable bit is bit 15 (same datasheet reference,
// address 0x3E)
constexpr uint16_t SIGMO2_ENABLE_BIT{1u << 0};
constexpr uint16_t SIGMO2_OUT_CONF_MASK{0x0Fu << 1};

// SC_26 (word, BMI270_FEATURE_STEP_COUNT_SC26_OFS, page FEAT_PAGE_STEP_COUNTER): watermark_level
// [9:0], reset_counter[10], en_detector[11], en_counter[12], en_activity[13] (BMI270 datasheet
// section 4.8.8, "FEATURES Pages" register table page 6, address 0x32). en_activity is preserved
// wherever this driver touches the word -- activity detection is out of scope here.
constexpr uint16_t SC26_WATERMARK_MASK{0x03FF};
constexpr uint16_t SC26_RESET_COUNTER_BIT{1u << 10};
constexpr uint16_t SC26_EN_DETECTOR_BIT{1u << 11};
constexpr uint16_t SC26_EN_COUNTER_BIT{1u << 12};
constexpr uint16_t SC26_EN_ACTIVITY_BIT{1u << 13};
// watermark_level's LSB is worth this many actual steps -- "Holds implicitly a 20x factor, so the
// range is 0 to 20460, with resolution of 20 steps. If 0, the output is disabled" (same datasheet
// reference). Bosch's reference driver (BMI270_SensorAPI bmi270.c set_step_config()) writes the
// watermark_level field through unscaled, confirming the 20x factor is not applied by hardware nor
// firmware -- callers of the register must do it themselves.
constexpr uint32_t SC26_WATERMARK_STEP_UNIT{20};

// Feature engine internal sample rate is fixed 50Hz (1 sample = 20ms) regardless of accel ODR.
// Convert a user-facing duration_ms to a sample count. Rejects durations shorter than one sample
// and counts that do not fit max_samples (13 bit field for no_motion, 16 bit for
// significant_motion). Returns false without touching out_samples on rejection.
bool duration_ms_to_samples_50hz(uint32_t duration_ms, uint16_t max_samples, uint16_t& out_samples)
{
    if (duration_ms < 20u) {
        return false;
    }
    const uint32_t samples = (duration_ms + 10u) / 20u;  // round to nearest
    if (samples == 0u || samples > max_samples) {
        return false;
    }
    out_samples = static_cast<uint16_t>(samples);
    return true;
}

inline uint32_t samples_50hz_to_duration_ms(uint16_t samples)
{
    return static_cast<uint32_t>(samples) * 20u;
}

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
    // first register read may return stale data or not be answered at all. Discard one read before
    // validating CHIP_ID (mirrors M5Unified's BMI270_Class, which reads WhoAmI twice).
    read_register8_quiet(REG_CHIP_ID, id);
    // A wedged BMI270 still answers, it just answers wrong, so the recovery below is only worth
    // trying once something acknowledged this address. Nothing answering means nothing is here --
    // which is the normal outcome of a probe -- and it must not be written to on the way out
    const bool answered = readRegister8(REG_CHIP_ID, id, 0);
    if (!answered || (id != CHIP_ID && !recover_wedged_interface(id))) {
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

    return _cfg.start_periodic
               ? startPeriodicMeasurement(_cfg.accel_range, _cfg.gyro_range, _cfg.accel_odr, _cfg.gyro_odr,
                                          _cfg.accel_filter_perf, _cfg.accel_bwp, _cfg.gyro_filter_perf, _cfg.gyro_bwp)
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
        if (!read_register8_quiet(REG_PWR_CONF, pwr_conf)) {
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
        if (!read_register8_quiet(REG_INTERNAL_STATUS, status)) {
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
                                            const bmi270::AccelBwp accel_bwp, const bmi270::FilterPerf gyro_filter_perf,
                                            const bmi270::GyroBwp gyro_bwp)
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
    return start_periodic_measurement(_accel_range, _gyro_range, _accel_odr, _gyro_odr, _accel_filter_perf, _accel_bwp,
                                      _gyro_filter_perf, _gyro_bwp);
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
    // Explicitly disable motion events; the user must re-enable them after wakeup().
    disableAllEvents();

    // A suspended chip must not be polled, so stop measuring first (unlike softReset(),
    // suspending during periodic measurement is allowed)
    if (inPeriodic() && !stopPeriodicMeasurement()) {
        M5_LIB_LOGE("Failed to stop before sleep");
        return false;
    }
    if (!writeRegister8(REG_PWR_CTRL, PWR_CTRL_ALL_OFF) || !writeRegister8(REG_PWR_CONF, PWR_CONF_ADV_POWER_SAVE)) {
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
    _interval = odr_interval_ms(odr);  // update() rate-limits on this
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
        if (writeRegister8(reg, val) && read_register8_quiet(reg, readback) && readback == val) {
            return true;
        }
        m5::utility::delay(1);
    }
    M5_LIB_LOGE("Failed to write and verify register:%02X val:%02X", reg, val);
    return false;
}

bool UnitBMI270::read_register8_quiet(const uint8_t reg, uint8_t& out)
{
    // The same I2C sequence readRegister8() performs -- a zero-length register write followed by a
    // one byte read -- without the error it logs when the register address is not acknowledged.
    // Every caller here is either retrying or deliberately discarding the result, so a NAK is an
    // expected answer rather than a fault, and logging one line per attempt buries the real errors.
    //
    // Note this goes through writeRegister(reg, ...), the same overload readRegister8() uses, not
    // the raw writeWithTransaction(nullptr, 0) that SoftwareI2C is known to crash on.
    if (!writeRegister(reg, nullptr, 0U)) {
        return false;
    }
    return readWithTransaction(&out, 1) == m5::hal::error::error_t::OK;
}

bool UnitBMI270::recover_wedged_interface(const uint8_t id)
{
    // Only a value whose set bits are a subset of the real CHIP_ID is worth this. I2C is open drain,
    // so anything else sharing the bus can pull a bit from 1 to 0 but never the other way: a BMI270
    // whose reply collided with another driver reads back as 0x20 or 0x00, never as 0x25. Any other
    // value is a different chip answering at this address, and writing a reset command to that would
    // be poking registers of a device we know nothing about.
    if ((id & static_cast<uint8_t>(~CHIP_ID)) != 0) {
        return false;
    }
    // This only reaches the mild form, where the chip still answers and just answers short: the
    // reset command below has to get through for any of it to work. It is deliberately not a cure
    // for the severe form -- a BMI270 whose auxiliary interface was switched on while its pins sat
    // on the host bus holds SDA low and takes no writes at all, measured on a DoF10, and there the
    // unit has to be unplugged (see UnitDoF9/UnitDoF10,
    // and the BMI270 datasheet, section 4.10.1: "No other I2C master or slave devices must be
    // attached to the auxiliary sensor interface").
    //
    // For the form it does reach, the soft reset is the only thing short of a power cycle that
    // clears the auxiliary configuration (BMI270 datasheet, section 4.17: "largely equivalent to a
    // power cycle"),
    // and it sits below the CHIP_ID check in begin() -- so without this one attempt that check would
    // keep rejecting the very chip that is waiting to be reset.
    M5_LIB_LOGW("CHIP_ID read as %02X; resetting in case the interface was left wedged", id);
    if (!writeRegister8(REG_CMD, CMD_SOFTRESET)) {
        return false;
    }

    // Same tolerant poll softReset() uses, for the same ~2ms reboot; a NAK here is expected, and the
    // reset default of REG_PWR_CONF being readable again is what says the interface is back
    uint8_t pwr_conf{};
    uint32_t retry = PWR_CONF_POLL_RETRY;
    do {
        m5::utility::delay(1);
        if (!read_register8_quiet(REG_PWR_CONF, pwr_conf)) {
            pwr_conf = 0;
        }
    } while (pwr_conf == 0 && --retry);

    // Quiet reads throughout: this whole path is a rescue attempt, and failing it just means the
    // caller reports the chip ID it originally saw. The discarded first read is the same dummy
    // transaction begin() performs, since a reset leaves the interface needing one again
    uint8_t recovered{};
    read_register8_quiet(REG_CHIP_ID, recovered);
    if (!read_register8_quiet(REG_CHIP_ID, recovered) || recovered != CHIP_ID) {
        return false;
    }
    M5_LIB_LOGI("Recovered a wedged BMI270 interface");
    return true;
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

bool UnitBMI270::enableEvent(const m5::imu::MotionEventKind kind, const void* cfg)
{
    if (cfg == nullptr) return false;
    if (kind == m5::imu::MotionEventKind::AnyMotion) {
        const auto& c = *static_cast<const m5::imu::AnyMotionConfig*>(cfg);
        if (!write_any_motion_features(c)) {
            return false;
        }
        _any_motion_cfg = c;
        _enabled_mask |= m5::imu::MotionEventKind::AnyMotion;
        _any_motion_pending = false;
        return true;
    } else if (kind == m5::imu::MotionEventKind::NoMotion) {
        const auto& c = *static_cast<const m5::imu::NoMotionConfig*>(cfg);
        if (!write_no_motion_features(c)) {
            return false;
        }
        _no_motion_cfg = c;
        _enabled_mask |= m5::imu::MotionEventKind::NoMotion;
        _no_motion_pending = false;
        return true;
    } else if (kind == m5::imu::MotionEventKind::SignificantMotion) {
        const auto& c = *static_cast<const m5::imu::SignificantMotionConfig*>(cfg);
        if (!write_sig_motion_features(c)) {
            return false;
        }
        _sig_motion_cfg = c;
        _enabled_mask |= m5::imu::MotionEventKind::SignificantMotion;
        _sig_motion_pending = false;
        return true;
    } else if (kind == m5::imu::MotionEventKind::StepCounter) {
        const auto& c = *static_cast<const m5::imu::StepCounterConfig*>(cfg);
        if (!write_step_counter_features(c)) {
            return false;
        }
        _step_counter_cfg = c;
        _enabled_mask |= m5::imu::MotionEventKind::StepCounter;
        _step_counter_pending = false;
        return true;
    }
    return false;
}

bool UnitBMI270::disableEvent(const m5::imu::MotionEventKind kind)
{
    if (kind == m5::imu::MotionEventKind::AnyMotion) {
        if (!clear_any_motion_features()) {
            return false;
        }
        _enabled_mask &= ~m5::imu::MotionEventKind::AnyMotion;
        _any_motion_pending = false;
        return true;
    } else if (kind == m5::imu::MotionEventKind::NoMotion) {
        if (!clear_no_motion_features()) {
            return false;
        }
        _enabled_mask &= ~m5::imu::MotionEventKind::NoMotion;
        _no_motion_pending = false;
        return true;
    } else if (kind == m5::imu::MotionEventKind::SignificantMotion) {
        if (!clear_sig_motion_features()) {
            return false;
        }
        _enabled_mask &= ~m5::imu::MotionEventKind::SignificantMotion;
        _sig_motion_pending = false;
        return true;
    } else if (kind == m5::imu::MotionEventKind::StepCounter) {
        if (!clear_step_counter_features()) {
            return false;
        }
        _enabled_mask &= ~m5::imu::MotionEventKind::StepCounter;
        _step_counter_pending = false;
        return true;
    }
    return false;
}

bool UnitBMI270::readEventStatus(m5::imu::MotionEventStatus& out)
{
    const m5::imu::MotionEventKind feature_mask =
        m5::imu::MotionEventKind::AnyMotion | m5::imu::MotionEventKind::NoMotion |
        m5::imu::MotionEventKind::SignificantMotion | m5::imu::MotionEventKind::StepCounter;
    if (!m5::imu::any(_enabled_mask & feature_mask)) return false;

    // INT_STATUS_0 is read-to-clear on the chip and reports every feature engine output in a
    // single byte, so drain it once per call into the per-event pending latches below rather than
    // re-reading it per event (a second read would come back empty and silently drop whichever
    // event was not checked first).
    uint8_t status{};
    if (readRegister8(REG_INT_STATUS_0, status, 0)) {
        if (status & INT_STATUS_0_ANY_MOTION_OUT_BIT) {
            _any_motion_pending = true;
        }
        if (status & INT_STATUS_0_NO_MOTION_OUT_BIT) {
            _no_motion_pending = true;
        }
        if (status & INT_STATUS_0_SIG_MOTION_OUT_BIT) {
            _sig_motion_pending = true;
        }
        if (status & INT_STATUS_0_STEP_COUNTER_OUT_BIT) {
            _step_counter_pending = true;
        }
    }

    bool fired = false;
    if (m5::imu::any(_enabled_mask & m5::imu::MotionEventKind::AnyMotion) && _any_motion_pending) {
        out.fired |= m5::imu::MotionEventKind::AnyMotion;
        _any_motion_pending = false;
        fired = true;
    }
    if (m5::imu::any(_enabled_mask & m5::imu::MotionEventKind::NoMotion) && _no_motion_pending) {
        out.fired |= m5::imu::MotionEventKind::NoMotion;
        _no_motion_pending = false;
        fired = true;
    }
    if (m5::imu::any(_enabled_mask & m5::imu::MotionEventKind::SignificantMotion) && _sig_motion_pending) {
        out.fired |= m5::imu::MotionEventKind::SignificantMotion;
        _sig_motion_pending = false;
        fired = true;
    }
    if (m5::imu::any(_enabled_mask & m5::imu::MotionEventKind::StepCounter) && _step_counter_pending) {
        // Unlike the other events, StepCounter's payload (the cumulative step count) is not part
        // of INT_STATUS_0 -- refresh the cache from REG_STEP_COUNT_0 so out.step_count carries the
        // latest value at fire time.
        read_step_count(_step_count_latest);
        out.fired |= m5::imu::MotionEventKind::StepCounter;
        out.step_count = _step_count_latest;
        _step_counter_pending = false;
        fired = true;
    }
    return fired;
}

bool UnitBMI270::write_any_motion_features(const m5::imu::AnyMotionConfig& c)
{
    if (!writeRegister8(REG_FEAT_PAGE, FEAT_PAGE_ANY_MOTION)) {
        return false;
    }

    // ANYMO_1 (duration + per-axis select) and ANYMO_2 (threshold + out_conf + enable) are each
    // fully specified by this driver -- neither word has reserved bits left to preserve -- so both
    // are built from scratch rather than read-modified.
    const uint32_t dur_samples = (static_cast<uint32_t>(c.duration_ms) * accelOdrHz() + 500) / 1000;
    const uint16_t word0 =
        static_cast<uint16_t>((dur_samples & ANYMO1_DURATION_MASK) | (c.axis_x ? ANYMO1_SELECT_X_BIT : 0) |
                              (c.axis_y ? ANYMO1_SELECT_Y_BIT : 0) | (c.axis_z ? ANYMO1_SELECT_Z_BIT : 0));
    // 0.48 mg/LSB (BMI270 datasheet 4.8.2); rounds to nearest LSB
    const uint32_t thr_lsb = (static_cast<uint32_t>(c.threshold_mg) * 1000U + 240U) / 480U;
    const uint16_t word1 =
        static_cast<uint16_t>((thr_lsb & ANYMO2_THRESHOLD_MASK) | ANYMO2_OUT_CONF_BIT6 | ANYMO2_ENABLE_BIT);

    const uint8_t buf[4]{
        static_cast<uint8_t>(word0 & 0xFF),
        static_cast<uint8_t>(word0 >> 8),
        static_cast<uint8_t>(word1 & 0xFF),
        static_cast<uint8_t>(word1 >> 8),
    };
    if (!write_feature_bytes(REG_FEATURES_ANY_MOTION, buf, sizeof(buf))) {
        return false;
    }

    // Map any_motion_out onto INT1 as well. No electrical INT pin is used by this poll-only
    // driver, but INT_STATUS_0 is documented to report a feature's output only once it is mapped
    // (BMI270 datasheet section 4.8.1: "INT1_MAP_FEAT and INT2_MAP_FEAT configure these features;
    // INT_STATUS_0 reports the interrupt source").
    uint8_t map{};
    if (!readRegister8(REG_INT_MAP_FEAT, map, 0)) {
        return false;
    }
    map |= INT_MAP_FEAT_ANY_MOTION_OUT_BIT;
    return writeRegister8(REG_INT_MAP_FEAT, map);
}

bool UnitBMI270::clear_any_motion_features()
{
    if (!writeRegister8(REG_FEAT_PAGE, FEAT_PAGE_ANY_MOTION)) {
        return false;
    }

    // Only ANYMO_2.enable needs clearing to stop the feature; duration/threshold/axis selection in
    // ANYMO_1 and the rest of ANYMO_2 are left as they were so a later enableEvent() call is not
    // required to fully reprogram them (write_any_motion_features() does that anyway, but this
    // keeps disableEvent() a minimal, targeted change).
    uint8_t buf[2]{};
    if (!read_feature_bytes(REG_FEATURES_ANY_MOTION_2, buf, sizeof(buf))) {
        return false;
    }
    uint16_t word1 = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    word1          = static_cast<uint16_t>(word1 & ~ANYMO2_ENABLE_BIT);
    buf[0]         = static_cast<uint8_t>(word1 & 0xFF);
    buf[1]         = static_cast<uint8_t>(word1 >> 8);
    if (!write_feature_bytes(REG_FEATURES_ANY_MOTION_2, buf, sizeof(buf))) {
        return false;
    }

    uint8_t map{};
    if (!readRegister8(REG_INT_MAP_FEAT, map, 0)) {
        return false;
    }
    map &= static_cast<uint8_t>(~INT_MAP_FEAT_ANY_MOTION_OUT_BIT);
    return writeRegister8(REG_INT_MAP_FEAT, map);
}

bool UnitBMI270::write_no_motion_features(const m5::imu::NoMotionConfig& cfg)
{
    if (!writeRegister8(REG_FEAT_PAGE, FEAT_PAGE_NO_MOTION)) {
        return false;
    }

    uint16_t dur_samples{};
    if (!duration_ms_to_samples_50hz(cfg.duration_ms, NOMO1_DURATION_MASK, dur_samples)) {
        return false;
    }
    const uint16_t word0 =
        static_cast<uint16_t>((dur_samples & NOMO1_DURATION_MASK) | (cfg.axis_x ? NOMO1_SELECT_X_BIT : 0) |
                              (cfg.axis_y ? NOMO1_SELECT_Y_BIT : 0) | (cfg.axis_z ? NOMO1_SELECT_Z_BIT : 0));

    // NOMO_2.out_conf is preserved instead of being forced to a fixed value (unlike ANYMO_2 in
    // write_any_motion_features()): its reset default already routes no_motion_out onto
    // INT_STATUS_0 bit 5, and nothing else in this driver ever rewrites it, so read-modify-write
    // is enough here and avoids clobbering a routing the application may have set some other way.
    uint8_t existing[4]{};
    if (!read_feature_bytes(BMI270_FEATURE_NO_MOTION_OFS_WORDS, existing, sizeof(existing))) {
        return false;
    }
    const uint16_t existing_word1 = static_cast<uint16_t>(existing[2]) | (static_cast<uint16_t>(existing[3]) << 8);
    // 0.48 mg/LSB (BMI270 datasheet 4.8.3, same scale as ANYMO_2.threshold); rounds to nearest LSB
    const uint32_t thr_lsb = (static_cast<uint32_t>(cfg.threshold_mg) * 1000U + 240U) / 480U;
    const uint16_t word1   = static_cast<uint16_t>((thr_lsb & NOMO2_THRESHOLD_MASK) |
                                                  (existing_word1 & NOMO2_OUT_CONF_MASK) | NOMO2_ENABLE_BIT);

    const uint8_t buf[4]{
        static_cast<uint8_t>(word0 & 0xFF),
        static_cast<uint8_t>(word0 >> 8),
        static_cast<uint8_t>(word1 & 0xFF),
        static_cast<uint8_t>(word1 >> 8),
    };
    if (!write_feature_bytes(BMI270_FEATURE_NO_MOTION_OFS_WORDS, buf, sizeof(buf))) {
        return false;
    }

    // Map no_motion_out onto INT1 as well, same reasoning as write_any_motion_features(): INT_STATUS_0
    // reports a feature's output only once it is mapped (BMI270 datasheet section 4.8.1).
    uint8_t map{};
    if (!readRegister8(REG_INT_MAP_FEAT, map, 0)) {
        return false;
    }
    map |= INT_MAP_FEAT_NO_MOTION_OUT_BIT;
    return writeRegister8(REG_INT_MAP_FEAT, map);
}

bool UnitBMI270::clear_no_motion_features()
{
    if (!writeRegister8(REG_FEAT_PAGE, FEAT_PAGE_NO_MOTION)) {
        return false;
    }

    // Only NOMO_2.enable needs clearing to stop the feature; duration/threshold/axis selection in
    // NOMO_1 and the rest of NOMO_2 (including out_conf) are left as they were so a later
    // enableEvent() call is not required to fully reprogram them.
    uint8_t buf[2]{};
    if (!read_feature_bytes(BMI270_FEATURE_NO_MOTION_2_OFS_WORDS, buf, sizeof(buf))) {
        return false;
    }
    uint16_t word1 = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    word1          = static_cast<uint16_t>(word1 & ~NOMO2_ENABLE_BIT);
    buf[0]         = static_cast<uint8_t>(word1 & 0xFF);
    buf[1]         = static_cast<uint8_t>(word1 >> 8);
    if (!write_feature_bytes(BMI270_FEATURE_NO_MOTION_2_OFS_WORDS, buf, sizeof(buf))) {
        return false;
    }

    uint8_t map{};
    if (!readRegister8(REG_INT_MAP_FEAT, map, 0)) {
        return false;
    }
    map &= static_cast<uint8_t>(~INT_MAP_FEAT_NO_MOTION_OUT_BIT);
    return writeRegister8(REG_INT_MAP_FEAT, map);
}

bool UnitBMI270::write_sig_motion_features(const m5::imu::SignificantMotionConfig& cfg)
{
    if (!writeRegister8(REG_FEAT_PAGE, FEAT_PAGE_NO_MOTION)) {
        return false;
    }

    // SIGMO_1 is fully specified by this driver -- the whole word is block_size, no reserved bits
    // left to preserve -- so it is built from scratch rather than read-modified.
    uint16_t block_size{};
    if (!duration_ms_to_samples_50hz(cfg.duration_ms, SIGMO1_BLOCK_SIZE_MASK, block_size)) {
        return false;
    }
    const uint8_t word0[2]{
        static_cast<uint8_t>(block_size & 0xFF),
        static_cast<uint8_t>(block_size >> 8),
    };
    if (!write_feature_bytes(BMI270_FEATURE_SIG_MOTION_OFS_WORDS, word0, sizeof(word0))) {
        return false;
    }

    // SIGMO_2.out_conf is preserved instead of being forced to a fixed value, same reasoning as
    // NOMO_2.out_conf in write_no_motion_features(): its reset default already routes
    // sig_motion_out onto INT_STATUS_0 bit 0.
    uint8_t existing[2]{};
    if (!read_feature_bytes(BMI270_FEATURE_SIG_MOTION_2_OFS_WORDS, existing, sizeof(existing))) {
        return false;
    }
    uint16_t word1 = static_cast<uint16_t>(existing[0]) | (static_cast<uint16_t>(existing[1]) << 8);
    word1          = static_cast<uint16_t>((word1 & SIGMO2_OUT_CONF_MASK) | SIGMO2_ENABLE_BIT);
    const uint8_t word1_buf[2]{
        static_cast<uint8_t>(word1 & 0xFF),
        static_cast<uint8_t>(word1 >> 8),
    };
    if (!write_feature_bytes(BMI270_FEATURE_SIG_MOTION_2_OFS_WORDS, word1_buf, sizeof(word1_buf))) {
        return false;
    }

    // Map sig_motion_out onto INT1 as well, same reasoning as write_any_motion_features():
    // INT_STATUS_0 reports a feature's output only once it is mapped (BMI270 datasheet section
    // 4.8.1).
    uint8_t map{};
    if (!readRegister8(REG_INT_MAP_FEAT, map, 0)) {
        return false;
    }
    map |= INT_MAP_FEAT_SIG_MOTION_OUT_BIT;
    return writeRegister8(REG_INT_MAP_FEAT, map);
}

bool UnitBMI270::clear_sig_motion_features()
{
    if (!writeRegister8(REG_FEAT_PAGE, FEAT_PAGE_NO_MOTION)) {
        return false;
    }

    // Only SIGMO_2.enable needs clearing to stop the feature; block_size in SIGMO_1 and out_conf in
    // SIGMO_2 are left as they were so a later enableEvent() call is not required to fully
    // reprogram them.
    uint8_t buf[2]{};
    if (!read_feature_bytes(BMI270_FEATURE_SIG_MOTION_2_OFS_WORDS, buf, sizeof(buf))) {
        return false;
    }
    uint16_t word1 = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    word1          = static_cast<uint16_t>(word1 & ~SIGMO2_ENABLE_BIT);
    buf[0]         = static_cast<uint8_t>(word1 & 0xFF);
    buf[1]         = static_cast<uint8_t>(word1 >> 8);
    if (!write_feature_bytes(BMI270_FEATURE_SIG_MOTION_2_OFS_WORDS, buf, sizeof(buf))) {
        return false;
    }

    uint8_t map{};
    if (!readRegister8(REG_INT_MAP_FEAT, map, 0)) {
        return false;
    }
    map &= static_cast<uint8_t>(~INT_MAP_FEAT_SIG_MOTION_OUT_BIT);
    return writeRegister8(REG_INT_MAP_FEAT, map);
}

bool UnitBMI270::write_step_counter_features(const m5::imu::StepCounterConfig& cfg)
{
    if (!writeRegister8(REG_FEAT_PAGE, FEAT_PAGE_STEP_COUNTER)) {
        return false;
    }

    uint8_t buf[2]{};
    if (!read_feature_bytes(BMI270_FEATURE_STEP_COUNT_SC26_OFS, buf, sizeof(buf))) {
        return false;
    }
    uint16_t word = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);

    // en_counter/en_detector (mutually exclusive per cfg.mode) and watermark_level are rewritten
    // below; en_activity and reset_counter are preserved -- activity detection is out of this
    // driver's scope, and a reset is only ever pulsed by resetStepCount(), never bundled in here.
    word = static_cast<uint16_t>(word & ~(SC26_EN_COUNTER_BIT | SC26_EN_DETECTOR_BIT | SC26_WATERMARK_MASK));

    if (cfg.mode == m5::imu::StepCounterConfig::Mode::Counter) {
        // Round cfg.watermark_steps to the nearest watermark_level field value (LSB = 20 actual
        // steps), but never round a nonzero request down to a field of 0: the datasheet documents
        // 0 as "output disabled", which would silently turn Counter mode into a no-op for any
        // watermark under 10 steps (StepCounterConfig's own default is 1).
        uint32_t field = (static_cast<uint32_t>(cfg.watermark_steps) + SC26_WATERMARK_STEP_UNIT / 2) /
                         SC26_WATERMARK_STEP_UNIT;
        if (cfg.watermark_steps > 0 && field == 0) {
            field = 1;
        }
        if (field > SC26_WATERMARK_MASK) {
            field = SC26_WATERMARK_MASK;
        }
        // Warn when the requested watermark cannot be represented exactly at the chip's 20-step
        // resolution (SC_26.watermark_level LSB = SC26_WATERMARK_STEP_UNIT actual steps): either
        // the rounding or the clamp above can make the value that actually fires differ from what
        // the caller asked for.
        const uint32_t actual_fire_steps = field * SC26_WATERMARK_STEP_UNIT;
        if (cfg.watermark_steps > 0 && actual_fire_steps != cfg.watermark_steps) {
            M5_LIB_LOGW("BMI270 step watermark: requested %u steps, actual %u steps (HW resolution %u)",
                        cfg.watermark_steps, actual_fire_steps, SC26_WATERMARK_STEP_UNIT);
        }
        word |= SC26_EN_COUNTER_BIT;
        word |= static_cast<uint16_t>(field);
    } else {
        word |= SC26_EN_DETECTOR_BIT;
    }

    buf[0] = static_cast<uint8_t>(word & 0xFF);
    buf[1] = static_cast<uint8_t>(word >> 8);
    if (!write_feature_bytes(BMI270_FEATURE_STEP_COUNT_SC26_OFS, buf, sizeof(buf))) {
        return false;
    }

    // Map step_counter_out onto INT1 as well, same reasoning as write_any_motion_features():
    // INT_STATUS_0 reports a feature's output only once it is mapped (BMI270 datasheet section
    // 4.8.1). SC_27.out_conf_step_detector is left untouched: its reset default (0x2, "output
    // assigned to bit-1") already routes step_counter_out onto INT_STATUS_0 bit 1.
    uint8_t map{};
    if (!readRegister8(REG_INT_MAP_FEAT, map, 0)) {
        return false;
    }
    map |= INT_MAP_FEAT_STEP_COUNTER_OUT_BIT;
    return writeRegister8(REG_INT_MAP_FEAT, map);
}

bool UnitBMI270::clear_step_counter_features()
{
    if (!writeRegister8(REG_FEAT_PAGE, FEAT_PAGE_STEP_COUNTER)) {
        return false;
    }

    // Only en_counter/en_detector need clearing to stop the feature; watermark_level, en_activity,
    // and reset_counter are left as they were so a later enableEvent() call is not required to
    // fully reprogram them (write_step_counter_features() does that anyway for watermark_level).
    uint8_t buf[2]{};
    if (!read_feature_bytes(BMI270_FEATURE_STEP_COUNT_SC26_OFS, buf, sizeof(buf))) {
        return false;
    }
    uint16_t word = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    word          = static_cast<uint16_t>(word & ~(SC26_EN_COUNTER_BIT | SC26_EN_DETECTOR_BIT));
    buf[0]        = static_cast<uint8_t>(word & 0xFF);
    buf[1]        = static_cast<uint8_t>(word >> 8);
    if (!write_feature_bytes(BMI270_FEATURE_STEP_COUNT_SC26_OFS, buf, sizeof(buf))) {
        return false;
    }

    uint8_t map{};
    if (!readRegister8(REG_INT_MAP_FEAT, map, 0)) {
        return false;
    }
    map &= static_cast<uint8_t>(~INT_MAP_FEAT_STEP_COUNTER_OUT_BIT);
    return writeRegister8(REG_INT_MAP_FEAT, map);
}

bool UnitBMI270::read_step_count(uint32_t& out)
{
    uint8_t buf[2]{};
    if (!readRegister(REG_STEP_COUNT_0, buf, sizeof(buf), 0)) {
        return false;
    }
    out = static_cast<uint32_t>(buf[0]) | (static_cast<uint32_t>(buf[1]) << 8);  // 16-bit little-endian
    return true;
}

bool UnitBMI270::resetStepCount()
{
    if (!writeRegister8(REG_FEAT_PAGE, FEAT_PAGE_STEP_COUNTER)) {
        return false;
    }

    uint8_t buf[2]{};
    if (!read_feature_bytes(BMI270_FEATURE_STEP_COUNT_SC26_OFS, buf, sizeof(buf))) {
        return false;
    }
    const uint16_t word = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);

    // Pulse reset_counter (set, then clear): the datasheet does not document it as self-clearing,
    // and Bosch's reference driver (BMI270_SensorAPI bmi270.c set_step_config()) just writes
    // through whatever value the caller supplies with no auto-clear of its own, so this driver
    // clears the bit back itself to leave the feature re-armed for a future reset. Per datasheet
    // section 4.8.8, the reset only takes effect while at least one of en_counter/en_detector/
    // en_activity is enabled.
    const uint16_t set_word = static_cast<uint16_t>(word | SC26_RESET_COUNTER_BIT);
    buf[0]                  = static_cast<uint8_t>(set_word & 0xFF);
    buf[1]                  = static_cast<uint8_t>(set_word >> 8);
    if (!write_feature_bytes(BMI270_FEATURE_STEP_COUNT_SC26_OFS, buf, sizeof(buf))) {
        return false;
    }
    m5::utility::delay(2);

    const uint16_t clear_word = static_cast<uint16_t>(word & ~SC26_RESET_COUNTER_BIT);
    buf[0]                    = static_cast<uint8_t>(clear_word & 0xFF);
    buf[1]                    = static_cast<uint8_t>(clear_word >> 8);
    if (!write_feature_bytes(BMI270_FEATURE_STEP_COUNT_SC26_OFS, buf, sizeof(buf))) {
        return false;
    }

    _step_count_latest = 0;
    return true;
}

bool UnitBMI270::read_feature_bytes(const uint16_t ofs_bytes, uint8_t* buf, const size_t len)
{
    return readRegister(static_cast<uint8_t>(ofs_bytes), buf, len, 0);
}

bool UnitBMI270::write_feature_bytes(const uint16_t ofs_bytes, const uint8_t* buf, const size_t len)
{
    return writeRegister(static_cast<uint8_t>(ofs_bytes), buf, len);
}

uint16_t UnitBMI270::accelOdrHz() const
{
    switch (_accel_odr) {
        case bmi270::AccelODR::Rate12_5Hz:
            return 12;
        case bmi270::AccelODR::Rate25Hz:
            return 25;
        case bmi270::AccelODR::Rate50Hz:
            return 50;
        case bmi270::AccelODR::Rate100Hz:
            return 100;
        case bmi270::AccelODR::Rate200Hz:
            return 200;
        case bmi270::AccelODR::Rate400Hz:
            return 400;
        case bmi270::AccelODR::Rate800Hz:
            return 800;
        case bmi270::AccelODR::Rate1600Hz:
            return 1600;
    }
    return 200;  // Unreachable for a valid AccelODR; matches config_t's power-on-default rate
}

}  // namespace unit
}  // namespace m5
