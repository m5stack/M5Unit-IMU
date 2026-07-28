/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_bmi270_aux_enabled.cpp
  @brief UnitBMI270AuxEnabled (BMI270 driving a magnetometer over its auxiliary interface)
*/
#include "unit_bmi270_aux_enabled.hpp"
#include <M5Utility.hpp>
#include <algorithm>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::bmi270;
using namespace m5::unit::bmi270::command;

namespace {
// STATUS.aux_busy must clear within this many 1ms polls (datasheet-cited AUX transaction timing)
constexpr uint32_t AUX_BUSY_POLL_RETRY{3};
// BMM150 soft-reset settling time before it responds to AUX reads (datasheet 3.0ms)
constexpr uint32_t AUX_RESET_DELAY_MS{3};

// REG_STATUS (0x03) bits
constexpr uint8_t STATUS_AUX_BUSY{0x04};  // bit2

// REG_PWR_CTRL (0x7D) / REG_PWR_CONF (0x7C)
constexpr uint8_t PWR_CTRL_ACC_GYR_TEMP_ON{0x0E};
constexpr uint8_t PWR_CTRL_ACC_GYR_TEMP_AUX_ON{0x0F};

// REG_IF_CONF (0x6B)
constexpr uint8_t IF_CONF_AUX_EN{0x20};  // bit5
constexpr uint8_t IF_CONF_AUX_OFF{0x00};

// REG_AUX_IF_CONF (0x4C) presets used during the BMM150 probe/data-mode switch
constexpr uint8_t AUX_IF_CONF_MANUAL_BL1{0x80};  // manual_en=1, man_rd_burst=BL1 (setup/single-register access)
constexpr uint8_t AUX_IF_CONF_MANUAL_BL8{0x8C};  // manual_en=1, man_rd_burst=BL8 (trim burst read)
constexpr uint8_t AUX_IF_CONF_DATA_MODE{0x4F};   // manual_en=0 (data mode), fcu_write_en=1, man_rd_burst=BL8,
                                                 // aux_rd_burst=BL8 (BMI270 auto-polls the AUX target)

// REG_AUX_DEV_ID (0x4B) value for the on-chip BMM150 (7bit I2C address 0x10, shifted for the addr<<1 encoding)
constexpr uint8_t AUX_DEV_ID_BMM150{0x10 << 1};

// BMM150 command values, reached only through the AUX bridge (distinct register space)
constexpr uint8_t BMM150_CHIP_ID_VALUE{0x32};
// bit7+bit1 SoftReset + bit0 PowerControl combined (same alias as bmm150::command::SOFT_RESET_AND_POWER_ON,
// duplicated here since this AUX path talks to the BMM150 directly, not through UnitBMM150)
constexpr uint8_t BMM150_SOFT_RESET_AND_POWER_ON{0x83};
// Output data rate the bridge runs the AUX BMM150 at. Fixed rather than configurable: the BMI270
// polls the magnetometer itself, so this has to stay matched to its own AUX timing. It is also what
// bounds the usable repetitions (bmm150::repetition_fits()), so both uses read it from here
// (fully qualified: this anonymous namespace sits at global scope, outside namespace m5::unit)
constexpr m5::unit::bmm150::DataRate AUX_BMM150_RATE{m5::unit::bmm150::DataRate::Rate30Hz};
// REG_OP_MODE = rate << 3 | Opmode Normal (0x00)
constexpr uint8_t BMM150_OP_MODE_NORMAL{static_cast<uint8_t>(static_cast<uint8_t>(AUX_BMM150_RATE) << 3)};
// PowerControl bit0 cleared: BMM150 suspend mode (lowest power, registers reset on resume)
constexpr uint8_t BMM150_SUSPEND{0x00};
}  // namespace

namespace m5 {
namespace unit {

// class UnitBMI270AuxEnabled
const char UnitBMI270AuxEnabled::name[] = "UnitBMI270AuxEnabled";
const types::uid_t UnitBMI270AuxEnabled::uid{"UnitBMI270AuxEnabled"_mmh3};
const types::attr_t UnitBMI270AuxEnabled::attr{attribute::AccessI2C};

UnitBMI270AuxEnabled::UnitBMI270AuxEnabled(const uint8_t addr) : UnitBMI270(addr)
{
}

bool UnitBMI270AuxEnabled::start_periodic_measurement(const bmi270::AccelRange accel_range,
                                                      const bmi270::GyroRange gyro_range,
                                                      const bmi270::AccelODR accel_odr,
                                                      const bmi270::GyroODR gyro_odr)
{
    // Reject an unusable preset before touching any register. Failing inside the probe would be
    // indistinguishable from "no BMM150 on this board", which degrades to 6-axis and would silently
    // swallow what is really a configuration error
    if (!bmm150::repetition_fits(AUX_BMM150_RATE, bmm150::preset_rep_xy(_cfg_aux_enabled.aux_mag_preset),
                                 bmm150::preset_rep_z(_cfg_aux_enabled.aux_mag_preset))) {
        M5_LIB_LOGE("aux_mag_preset needs %uus, longer than the %uus period of the AUX BMM150's fixed rate",
                    bmm150::measurement_time_us(bmm150::preset_rep_xy(_cfg_aux_enabled.aux_mag_preset),
                                                bmm150::preset_rep_z(_cfg_aux_enabled.aux_mag_preset)),
                    bmm150::rate_interval_us(AUX_BMM150_RATE));
        return false;
    }

    if (!UnitBMI270::start_periodic_measurement(accel_range, gyro_range, accel_odr, gyro_odr)) {
        return false;
    }

    _aux_mag = probe_aux_bmm150();
    if (!_aux_mag) {
        // No BMM150 responded (normal on a board that wires nothing to the interface): switch it
        // back off and carry on with 6-axis (accel/gyro/temp) only
        writeRegister8(REG_IF_CONF, IF_CONF_AUX_OFF);
        M5_LIB_LOGI("AUX BMM150 not available; continuing with 6-axis (accel/gyro/temp) only");
    }
    return true;
}

bool UnitBMI270AuxEnabled::read_sensor_burst(bmi270::Data& d)
{
    if (!_aux_mag) {
        return UnitBMI270::read_sensor_burst(d);
    }
    // AUX data mode auto-polls the BMM150 into REG_DATA_0..DATA_7 (mag+rhall) followed by
    // the usual accel/gyro at REG_DATA_8..DATA_19; one 20 byte burst covers all three.
    std::array<uint8_t, 20> buf{};
    if (!readRegister(REG_DATA_0, buf.data(), buf.size(), 0)) {
        M5_LIB_LOGE("Failed to read data");
        return false;
    }
    std::copy(buf.begin(), buf.begin() + d.mag.raw.size(), d.mag.raw.begin());
    std::copy(buf.begin() + d.mag.raw.size(), buf.end(), d.raw.begin());
    d.mag.trim = _mag_trim;
    d.aux_mag  = true;
    return true;
}

void UnitBMI270AuxEnabled::imuConvertParam(m5::imu::imu_convert_param_t& out) const
{
    UnitBMI270::imuConvertParam(out);
    if (_aux_mag) {
        out.mag_res = 1.0f / 16.0f;  // Compensated BMM150 output, matches UnitBMM150's scale
    }
}

bool UnitBMI270AuxEnabled::sleep()
{
    // A suspended chip must not be polled, so stop measuring before parking the magnetometer
    if (inPeriodic() && !stopPeriodicMeasurement()) {
        M5_LIB_LOGE("Failed to stop before sleep");
        return false;
    }
    // The BMM150 keeps running on its own once the bridge is gone, so park it first. Best effort:
    // a failure here must not stop the BMI270 itself from being suspended.
    if (_aux_mag && !aux_write_register8(REG_BMM150_POWER_CONTROL, BMM150_SUSPEND)) {
        M5_LIB_LOGW("Failed to suspend the AUX BMM150");
    }
    _aux_mag = false;  // Re-probed by the next start_periodic_measurement()
    return UnitBMI270::sleep();
}

bool UnitBMI270AuxEnabled::probe_aux_bmm150()
{
    // Setup mode: enable the AUX interface and switch it to manual (single-register) access
    // so we can talk to the BMM150 directly (mirrors M5Unified's BMI270_Class::auxSetupMode()).
    if (!writeRegister8(REG_IF_CONF, IF_CONF_AUX_EN) || !writeRegister8(REG_PWR_CONF, 0x00) ||
        !writeRegister8(REG_PWR_CTRL, PWR_CTRL_ACC_GYR_TEMP_ON) ||
        !writeRegister8(REG_AUX_IF_CONF, AUX_IF_CONF_MANUAL_BL1) ||
        !writeRegister8(REG_AUX_DEV_ID, AUX_DEV_ID_BMM150)) {
        M5_LIB_LOGE("Failed to configure the AUX interface");
        return false;
    }

    if (!aux_write_register8(REG_BMM150_POWER_CONTROL, BMM150_SOFT_RESET_AND_POWER_ON)) {
        M5_LIB_LOGE("Failed to reset the AUX BMM150");
        return false;
    }
    m5::utility::delay(AUX_RESET_DELAY_MS);

    // The first read right after a fresh AUX target reset returns a stale value
    // (M5Unified BMI270_Class.cpp:76-77); discard it before checking the chip ID.
    uint8_t who{};
    if (!aux_read_register8(REG_BMM150_CHIP_ID, who) || !aux_read_register8(REG_BMM150_CHIP_ID, who) ||
        who != BMM150_CHIP_ID_VALUE) {
        return false;  // No AUX BMM150 wired on this board (normal case, no log)
    }

    if (!aux_read_trim(_mag_trim)) {
        M5_LIB_LOGE("Failed to read the AUX BMM150 trim registers");
        return false;
    }

    // Set the repetition (oversampling) before leaving setup mode, the way UnitBMM150 does. Without
    // this the BMM150 stays at its power-on default, which is noisier than any preset. The pair was
    // already checked against AUX_BMM150_RATE in start_periodic_measurement()
    if (!aux_write_register8(REG_BMM150_REP_XY, bmm150::preset_rep_xy(_cfg_aux_enabled.aux_mag_preset)) ||
        !aux_write_register8(REG_BMM150_REP_Z, bmm150::preset_rep_z(_cfg_aux_enabled.aux_mag_preset))) {
        M5_LIB_LOGE("Failed to set the AUX BMM150 repetition preset");
        return false;
    }

    if (!aux_write_register8(REG_BMM150_OP_MODE, BMM150_OP_MODE_NORMAL) ||
        !write_register8_verified(REG_AUX_IF_CONF, AUX_IF_CONF_DATA_MODE) ||
        !write_register8_verified(REG_AUX_RD_ADDR, REG_BMM150_DATA_X_LSB) ||
        !write_register8_verified(REG_PWR_CTRL, PWR_CTRL_ACC_GYR_TEMP_AUX_ON)) {
        M5_LIB_LOGE("Failed to switch the AUX BMM150 to data mode");
        return false;
    }
    return true;
}

bool UnitBMI270AuxEnabled::aux_wait_ready()
{
    for (uint32_t retry = 0; retry < AUX_BUSY_POLL_RETRY; ++retry) {
        uint8_t status{};
        if (!readRegister8(REG_STATUS, status, 0)) {
            return false;
        }
        if ((status & STATUS_AUX_BUSY) == 0) {
            return true;
        }
        m5::utility::delay(1);
    }
    return false;
}

bool UnitBMI270AuxEnabled::aux_write_register8(const uint8_t reg, const uint8_t val)
{
    return writeRegister8(REG_AUX_WR_DATA, val) && writeRegister8(REG_AUX_WR_ADDR, reg) && aux_wait_ready();
}

bool UnitBMI270AuxEnabled::aux_read_register8(const uint8_t reg, uint8_t& out)
{
    if (!writeRegister8(REG_AUX_IF_CONF, AUX_IF_CONF_MANUAL_BL1) || !writeRegister8(REG_AUX_RD_ADDR, reg) ||
        !aux_wait_ready()) {
        return false;
    }
    return readRegister8(REG_DATA_0, out, 0);
}

bool UnitBMI270AuxEnabled::aux_read_burst8(const uint8_t bmm150_reg, std::array<uint8_t, 8>& out)
{
    if (!writeRegister8(REG_AUX_RD_ADDR, bmm150_reg) || !aux_wait_ready()) {
        return false;
    }
    return readRegister(REG_DATA_0, out.data(), out.size(), 0);
}

bool UnitBMI270AuxEnabled::aux_read_trim(bmm150::trim_t& trim)
{
    // BMM150 trim occupies registers 0x5D-0x71 (21 bytes), but AUX manual burst reads are
    // limited to 1/2/6/8 bytes. Three overlapping 8 byte bursts starting at 0x5D/0x65/0x6A
    // cover the whole range; overlapping bytes are simply overwritten with identical values.
    constexpr uint8_t TRIM_FIRST{0x5D};
    constexpr uint8_t TRIM_SECOND{0x65};
    constexpr uint8_t TRIM_THIRD{0x6A};
    constexpr size_t TRIM_BYTES{0x71 - TRIM_FIRST + 1};  // 21

    if (!writeRegister8(REG_AUX_IF_CONF, AUX_IF_CONF_MANUAL_BL8)) {
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

}  // namespace unit
}  // namespace m5
