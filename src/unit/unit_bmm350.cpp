/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_bmm350.cpp
  @brief UnitBMM350 (3-axis magnetometer, U231) unit component
*/
#include "unit_bmm350.hpp"
#include <M5Utility.hpp>
#include <array>
#include <cmath>
#include <cstring>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::bmm350;
using namespace m5::unit::bmm350::command;
using m5::imu::Capability;

namespace {
constexpr uint32_t WRITE_VERIFY_RETRY{16};
// Bounded poll for OTP_STATUS_REG.otp_cmd_done (datasheet/SensorAPI poll every 300us; 1ms steps here)
constexpr uint32_t OTP_POLL_RETRY{50};
// Max single-shot read length via read_registers() (2 dummy bytes are added on top of this)
constexpr uint32_t READ_MAX_LEN{15};

// Output data rate -> sampling interval (ms), indexed by the DataRate raw value (0x02..0x0A)
constexpr uint32_t RATE_INTERVAL_MS[11] = {
    0,   0,  3,  5,   10,   // 0x00,0x01(unused),0x02=400Hz,0x03=200Hz,0x04=100Hz
    20,  40, 80, 160, 320,  // 0x05=50Hz,0x06=25Hz,0x07=12.5Hz,0x08=6.25Hz,0x09=3.125Hz
    640,                    // 0x0A=1.5625Hz
};

constexpr uint32_t interval_ms(const DataRate rate)
{
    return RATE_INTERVAL_MS[static_cast<uint8_t>(rate)];
}

// Sign-extends an unsigned value of the given bit width (Bosch bmm350.c fix_sign(), reimplemented
// from its documented behaviour: value >= 2^(bits-1) is treated as negative)
constexpr int32_t fix_sign_bits(const uint32_t val, const uint32_t bits)
{
    return (val >= (1u << (bits - 1))) ? static_cast<int32_t>(val - (1u << bits)) : static_cast<int32_t>(val);
}

// Decodes the 32 OTP words (read via OTP_CMD_REG/OTP_DATA_MSB/OTP_DATA_LSB at boot) into the
// per-chip compensation coefficients consumed by bmm350::Data. Word indices and bit layout
// re-implemented (not copied) from the Bosch BMM350_SensorAPI update_mag_off_sens() (bmm350.c,
// BSD-3-Clause), verified against
// https://github.com/boschsensortec/BMM350_SensorAPI/blob/main/bmm350.c
otp_compensation_t decode_otp(const std::array<uint16_t, OTP_WORD_COUNT>& otp)
{
    otp_compensation_t comp{};

    constexpr uint16_t LSB_MASK = 0x00FF;
    constexpr uint16_t MSB_MASK = 0xFF00;

    // OTP word indices (BMM350_TEMP_OFF_SENS=0x0D, BMM350_MAG_OFFSET_X/Y/Z=0x0E/0x0F/0x10)
    const uint32_t off_x = otp[0x0E] & 0x0FFFu;
    const uint32_t off_y = (static_cast<uint32_t>(otp[0x0E] & 0xF000u) >> 4) + (otp[0x0F] & LSB_MASK);
    const uint32_t off_z = static_cast<uint32_t>(otp[0x0F] & 0x0F00u) + (otp[0x10] & LSB_MASK);
    const uint32_t t_off = otp[0x0D] & LSB_MASK;

    comp.offset[0] = static_cast<float>(fix_sign_bits(off_x, 12));
    comp.offset[1] = static_cast<float>(fix_sign_bits(off_y, 12));
    comp.offset[2] = static_cast<float>(fix_sign_bits(off_z, 12));
    comp.t_offs    = static_cast<float>(fix_sign_bits(t_off, 8)) / 5.0f;

    // BMM350_MAG_SENS_X=0x10 (shares word 0x10 with offset_z), _Y/_Z=0x11
    const uint32_t sens_x = static_cast<uint32_t>(otp[0x10] & MSB_MASK) >> 8;
    const uint32_t sens_y = otp[0x11] & LSB_MASK;
    const uint32_t sens_z = static_cast<uint32_t>(otp[0x11] & MSB_MASK) >> 8;
    const uint32_t t_sens = static_cast<uint32_t>(otp[0x0D] & MSB_MASK) >> 8;

    comp.sens[0] = static_cast<float>(fix_sign_bits(sens_x, 8)) / 256.0f;
    comp.sens[1] = static_cast<float>(fix_sign_bits(sens_y, 8)) / 256.0f;
    comp.sens[2] = static_cast<float>(fix_sign_bits(sens_z, 8)) / 256.0f;
    comp.t_sens  = static_cast<float>(fix_sign_bits(t_sens, 8)) / 512.0f;

    // BMM350_MAG_TCO_X/Y/Z = 0x12/0x13/0x14 (low byte), TCS shares the same words (high byte)
    const uint32_t tco_x = otp[0x12] & LSB_MASK;
    const uint32_t tco_y = otp[0x13] & LSB_MASK;
    const uint32_t tco_z = otp[0x14] & LSB_MASK;

    comp.tco[0] = static_cast<float>(fix_sign_bits(tco_x, 8)) / 32.0f;
    comp.tco[1] = static_cast<float>(fix_sign_bits(tco_y, 8)) / 32.0f;
    comp.tco[2] = static_cast<float>(fix_sign_bits(tco_z, 8)) / 32.0f;

    const uint32_t tcs_x = static_cast<uint32_t>(otp[0x12] & MSB_MASK) >> 8;
    const uint32_t tcs_y = static_cast<uint32_t>(otp[0x13] & MSB_MASK) >> 8;
    const uint32_t tcs_z = static_cast<uint32_t>(otp[0x14] & MSB_MASK) >> 8;

    comp.tcs[0] = static_cast<float>(fix_sign_bits(tcs_x, 8)) / 16384.0f;
    comp.tcs[1] = static_cast<float>(fix_sign_bits(tcs_y, 8)) / 16384.0f;
    comp.tcs[2] = static_cast<float>(fix_sign_bits(tcs_z, 8)) / 16384.0f;

    // BMM350_MAG_DUT_T_0 = 0x18 (full 16bit word)
    comp.t0 = static_cast<float>(fix_sign_bits(otp[0x18], 16)) / 512.0f + 23.0f;

    // BMM350_CROSS_X_Y/Y_X = 0x15, _Z_X/_Z_Y = 0x16
    const uint32_t cross_x_y = otp[0x15] & LSB_MASK;
    const uint32_t cross_y_x = static_cast<uint32_t>(otp[0x15] & MSB_MASK) >> 8;
    const uint32_t cross_z_x = otp[0x16] & LSB_MASK;
    const uint32_t cross_z_y = static_cast<uint32_t>(otp[0x16] & MSB_MASK) >> 8;

    comp.cross[0] = static_cast<float>(fix_sign_bits(cross_x_y, 8)) / 800.0f;
    comp.cross[1] = static_cast<float>(fix_sign_bits(cross_y_x, 8)) / 800.0f;
    comp.cross[2] = static_cast<float>(fix_sign_bits(cross_z_x, 8)) / 800.0f;
    comp.cross[3] = static_cast<float>(fix_sign_bits(cross_z_y, 8)) / 800.0f;

    return comp;
}

}  // namespace

namespace m5 {
namespace unit {

// class UnitBMM350
const char UnitBMM350::name[] = "UnitBMM350";
const types::uid_t UnitBMM350::uid{"UnitBMM350"_mmh3};
const types::attr_t UnitBMM350::attr{attribute::AccessI2C};

UnitBMM350::UnitBMM350(const uint8_t addr)
    : Component(addr), _data{new m5::container::CircularBuffer<bmm350::Data>(_cfg.stored_size)}
{
    auto ccfg        = component_config();
    ccfg.clock       = 400 * 1000U;
    ccfg.stored_size = _cfg.stored_size;
    component_config(ccfg);
}

bool UnitBMM350::begin()
{
    if (_cfg.stored_size != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<bmm350::Data>(_cfg.stored_size));
    }
    if (!_data) {
        M5_LIB_LOGE("Failed to allocate");
        return false;
    }
    _data->clear();
    _pushed = _consumed = 0;

    // config_t.stored_size is the sole control surface for the ring buffer depth;
    // keep component_config().stored_size in sync with it here.
    auto ccfg        = component_config();
    ccfg.stored_size = _cfg.stored_size;
    component_config(ccfg);

    // Startup time from POR (datasheet 3ms)
    m5::utility::delay(3);

    uint8_t id{};
    if (!read_register8(CHIP_ID, id) || id != CHIP_ID_VALUE) {
        M5_LIB_LOGE("Not BMM350 %02X", id);
        return false;
    }

    if (!writeRegister8(CMD, CMD_SOFTRESET)) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }
    m5::utility::delay(24);

    otp_compensation_t comp{};
    if (!download_otp(comp)) {
        M5_LIB_LOGE("Failed to download OTP");
        return false;
    }
    _comp = comp;

    if (!writeRegister8(OTP_CMD_REG, OTP_CMD_PWR_OFF_OTP)) {
        M5_LIB_LOGE("Failed to power off OTP");
        return false;
    }

    // Magnetic reset (FGR then BR; boot must complete this once before Normal mode)
    if (!writeRegister8(PMU_CMD, PMU_CMD_FGR)) {
        M5_LIB_LOGE("Failed to issue FGR");
        return false;
    }
    m5::utility::delay(18);
    if (!writeRegister8(PMU_CMD, PMU_CMD_BR)) {
        M5_LIB_LOGE("Failed to issue BR");
        return false;
    }
    m5::utility::delay(14);

    if (!write_register8_verified(INT_CTRL, INT_CTRL_DRDY_DATA_REG_EN)) {
        M5_LIB_LOGE("Failed to initialize");
        return false;
    }

    if (!commit_odr_avg(_cfg.rate, _cfg.avg)) {
        M5_LIB_LOGE("Failed to initialize");
        return false;
    }

    return _cfg.start_periodic ? startPeriodicMeasurement() : true;
}

void UnitBMM350::update(const bool force)
{
    _updated = false;
    if (inPeriodic()) {
        const auto at = m5::utility::millis();
        if (force || !_latest || at >= _latest + _interval) {
            bmm350::Data d{};
            _updated = read_measurement(d);
            if (_updated) {
                _data->push_back(d);
                ++_pushed;
                _latest = at;
            }
        }
    }
}

bool UnitBMM350::start_periodic_measurement()
{
    if (inPeriodic()) {
        return false;
    }
    _periodic = true;
    _latest   = 0;
    _interval = interval_ms(_cfg.rate);
    return true;
}

bool UnitBMM350::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

bool UnitBMM350::startPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitBMM350, bmm350::Data>::startPeriodicMeasurement();
}

bool UnitBMM350::stopPeriodicMeasurement()
{
    return PeriodicMeasurementAdapter<UnitBMM350, bmm350::Data>::stopPeriodicMeasurement();
}

bool UnitBMM350::writeDataRate(const bmm350::DataRate rate)
{
    if (!writeRegister8(PMU_CMD, PMU_CMD_SUS)) {
        M5_LIB_LOGE("Failed to suspend");
        return false;
    }
    m5::utility::delay(6);

    if (!commit_odr_avg(rate, _cfg.avg)) {
        return false;
    }
    _cfg.rate = rate;
    _interval = interval_ms(rate);
    return true;
}

bool UnitBMM350::writeAveraging(const bmm350::Averaging avg)
{
    if (!writeRegister8(PMU_CMD, PMU_CMD_SUS)) {
        M5_LIB_LOGE("Failed to suspend");
        return false;
    }
    m5::utility::delay(6);

    if (!commit_odr_avg(_cfg.rate, avg)) {
        return false;
    }
    _cfg.avg = avg;
    return true;
}

bool UnitBMM350::commit_odr_avg(const bmm350::DataRate rate, const bmm350::Averaging avg)
{
    const auto clamped = bmm350::clamp_averaging(rate, avg);
    const uint8_t aggr = static_cast<uint8_t>((static_cast<uint8_t>(clamped) << 4) | static_cast<uint8_t>(rate));
    if (!write_register8_verified(PMU_CMD_AGGR_SET, aggr)) {
        return false;
    }

    if (!writeRegister8(PMU_CMD, PMU_CMD_UPD_OAE)) {
        M5_LIB_LOGE("Failed to update OAE");
        return false;
    }
    m5::utility::delay(1);

    if (!writeRegister8(PMU_CMD, PMU_CMD_NM)) {
        M5_LIB_LOGE("Failed to enter normal mode");
        return false;
    }
    m5::utility::delay(38);
    return true;
}

bool UnitBMM350::download_otp(bmm350::otp_compensation_t& comp)
{
    std::array<uint16_t, OTP_WORD_COUNT> words{};

    for (uint32_t i = 0; i < OTP_WORD_COUNT; ++i) {
        if (!writeRegister8(OTP_CMD_REG, static_cast<uint8_t>(OTP_CMD_DIR_READ | (i & OTP_WORD_ADDR_MASK)))) {
            M5_LIB_LOGE("Failed to issue OTP read at word %u", i);
            return false;
        }

        bool done{};
        for (uint32_t retry = 0; retry < OTP_POLL_RETRY; ++retry) {
            m5::utility::delay(1);
            uint8_t status{};
            if (!read_register8(OTP_STATUS_REG, status)) {
                M5_LIB_LOGE("Failed to read OTP status");
                return false;
            }
            if (status & OTP_STATUS_ERROR_MASK) {
                M5_LIB_LOGE("OTP status error:%02X at word %u", status, i);
                return false;
            }
            if (status & OTP_STATUS_CMD_DONE) {
                done = true;
                break;
            }
        }
        if (!done) {
            M5_LIB_LOGE("OTP read timeout at word %u", i);
            return false;
        }

        uint8_t msb{}, lsb{};
        if (!read_register8(OTP_DATA_MSB, msb) || !read_register8(OTP_DATA_LSB, lsb)) {
            M5_LIB_LOGE("Failed to read OTP data at word %u", i);
            return false;
        }
        words[i] = static_cast<uint16_t>((msb << 8) | lsb);
    }

    comp = decode_otp(words);
    return true;
}

bool UnitBMM350::read_measurement(bmm350::Data& d)
{
    uint8_t status{};
    if (!read_register8(INT_STATUS, status)) {
        M5_LIB_LOGE("Failed to read INT_STATUS");
        return false;
    }
    if (!(status & INT_STATUS_DRDY_DATA_REG)) {
        return false;  // Not ready yet (normal case, no log)
    }
    if (!read_registers(MAG_X_XLSB, d.raw.data(), d.raw.size())) {
        M5_LIB_LOGE("Failed to read data");
        return false;
    }
    d.comp = _comp;
    return true;
}

bool UnitBMM350::write_register8_verified(const uint8_t reg, const uint8_t val)
{
    for (uint32_t i = 0; i < WRITE_VERIFY_RETRY; ++i) {
        uint8_t readback{};
        if (writeRegister8(reg, val) && read_register8(reg, readback) && readback == val) {
            return true;
        }
        m5::utility::delay(1);
    }
    M5_LIB_LOGE("Failed to write and verify register:%02X val:%02X", reg, val);
    return false;
}

bool UnitBMM350::read_registers(const uint8_t reg, uint8_t* out, const uint32_t len)
{
    if (!out || len == 0 || len > READ_MAX_LEN) {
        return false;
    }
    std::array<uint8_t, READ_MAX_LEN + 2> tmp{};
    if (!readRegister(reg, tmp.data(), len + 2, 0)) {
        return false;
    }
    std::memcpy(out, tmp.data() + 2, len);
    return true;
}

bool UnitBMM350::read_register8(const uint8_t reg, uint8_t& out)
{
    return read_registers(reg, &out, 1);
}

Capability UnitBMM350::readImuRaw(m5::imu::imu_raw_data_t& out)
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

    // Compensated uT quantized to 16 LSB/uT (matches UnitBMM150's scale, see mag_res below)
    out.mag[0] = static_cast<int32_t>(std::lroundf(d->x_ut() * 16));
    out.mag[1] = static_cast<int32_t>(std::lroundf(d->y_ut() * 16));
    out.mag[2] = static_cast<int32_t>(std::lroundf(d->z_ut() * 16));
    out.temp   = d->temp_adc();
    return imuSpec();
}

void UnitBMM350::imuConvertParam(m5::imu::imu_convert_param_t& out) const
{
    out.mag_res     = 1.0f / 16;
    out.temp_res    = 0.00098128f * 256;
    out.temp_offset = -25.49f;
}

}  // namespace unit
}  // namespace m5
