/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_bmi270.hpp
  @brief UnitBMI270 (6-axis IMU) unit component
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_BMI270_HPP
#define M5_UNIT_IMU_UNIT_UNIT_BMI270_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <memory>
#include "unit_bmi270_data.hpp"
#include "unit_bmm150_data.hpp"
#include "../imu/imu_data_source.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitBMI270
  @brief 6-axis (accel + gyro) IMU unit. CoreS3/AtomS3R built-in
  @note When config_t::enable_aux_bmm150 is set (the default) and an on-chip
  BMM150 is detected via the AUX interface (as wired on CoreS3/AtomS3R), this
  unit also provides compensated magnetometer data; see imuSpec()
 */
class UnitBMI270 : public Component,
                   public PeriodicMeasurementAdapter<UnitBMI270, bmi270::Data>,
                   public m5::imu::IMUDataSource {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitBMI270, 0x69);

public:
    /*!
      @struct config_t
      @brief Settings for begin()
      @note Ring buffer depth is configured via component_config() (default:
      bmi270::DEFAULT_STORED_SIZE)
     */
    struct config_t {
        //! Start periodic measurement on begin()?
        bool start_periodic{true};
        //! Accelerometer range
        bmi270::AccelRange accel_range{bmi270::AccelRange::G8};
        //! Gyroscope range
        bmi270::GyroRange gyro_range{bmi270::GyroRange::DPS2000};
        //! Accelerometer output data rate
        bmi270::AccelODR accel_odr{bmi270::AccelODR::Rate100Hz};
        //! Gyroscope output data rate (periodic measurement interval is derived from this)
        bmi270::GyroODR gyro_odr{bmi270::GyroODR::Rate100Hz};
        //! Probe for an on-chip BMM150 magnetometer via the AUX interface on begin()?
        //! If no BMM150 responds, begin() still succeeds with 6-axis (accel/gyro) only
        bool enable_aux_bmm150{true};
    };

    explicit UnitBMI270(const uint8_t addr = DEFAULT_ADDRESS);
    virtual ~UnitBMI270()
    {
    }

    //! @brief Begin the unit
    virtual bool begin() override;
    //! @brief Update the unit
    //! @param[in] force Forces an update if true, even if not time for periodic measurement
    virtual void update(const bool force = false) override;

    ///@name Settings
    ///@{
    //! @brief Gets the configuration
    //! @return Current configuration
    config_t config() const
    {
        return _cfg;
    }
    //! @brief Set the configuration
    //! @param[in] cfg Configuration
    void config(const config_t& cfg)
    {
        _cfg = cfg;
    }
    ///@}

    ///@name Range setting (applies immediately)
    ///@{
    //! @brief Write the accelerometer range
    //! @param[in] range Accelerometer range
    //! @return True if successful
    bool writeAccelRange(const bmi270::AccelRange range);
    //! @brief Write the gyroscope range
    //! @param[in] range Gyroscope range
    //! @return True if successful
    bool writeGyroRange(const bmi270::GyroRange range);
    //! @brief Read the accelerometer range from the device (REG_ACC_RANGE bits[1:0])
    //! @param[out] range Accelerometer range
    //! @return True if successful
    bool readAccelRange(bmi270::AccelRange& range);
    //! @brief Read the gyroscope range from the device (REG_GYR_RANGE bits[2:0])
    //! @param[out] range Gyroscope range
    //! @return True if successful, false if the register holds a reserved (unmapped) value
    bool readGyroRange(bmi270::GyroRange& range);
    ///@}

    ///@name Periodic measurement
    ///@{
    //! @brief Start periodic measurement in the current settings
    //! @return True if successful
    bool startPeriodicMeasurement();
    //! @brief Stop periodic measurement
    //! @return True if successful
    bool stopPeriodicMeasurement();
    ///@}

    ///@name IMUDataSource
    ///@{
    //! @brief Capability of this unit
    //! @return Accel | Gyro | Temp, plus Mag if an AUX BMM150 was detected in begin()
    virtual m5::imu::Capability imuSpec() const override
    {
        using m5::imu::Capability;
        const auto base = Capability::Accel | Capability::Gyro | Capability::Temp;
        return _aux_mag ? (base | Capability::Mag) : base;
    }
    //! @brief Read the latest raw IMU data
    //! @param[out] out Raw IMU data
    //! @return Capability of the data actually stored in out (None if no data available)
    virtual m5::imu::Capability readImuRaw(m5::imu::imu_raw_data_t& out) override;
    //! @brief Gets the conversion parameters for raw IMU data
    //! @param[out] out Conversion parameters
    virtual void imuConvertParam(m5::imu::imu_convert_param_t& out) const override;
    ///@}

protected:
    bool start_periodic_measurement();
    bool stop_periodic_measurement();
    bool read_measurement(bmi270::Data& d);
    bool write_register8_verified(const uint8_t reg, const uint8_t val);
    // Uploads the 8192 byte configuration blob (unit_bmi270_config.inl) to REG_INIT_DATA,
    // split into chunks that fit within the I2C write buffer (see .cpp for chunk size
    // rationale). Returns false (and logs) on the first failed chunk, unlike M5Unified's
    // BMI270_Class::_upload_file() which always returns true regardless of write result.
    bool upload_config();

    // AUX (on-chip BMM150) support. Called once from begin() after the ACC/GYR config
    // writes succeed; on success the AUX interface is left in data mode (BMI270 auto-polls
    // the BMM150 at 30Hz into REG_DATA_0..REG_DATA_7) and _mag_trim holds the factory trim read
    // via the AUX bridge. On failure (no BMM150 wired), the caller falls back to 6-axis.
    bool probe_aux_bmm150();
    // Polls REG_STATUS.aux_busy (bit2) until clear; gates every AUX manual transaction below
    bool aux_wait_ready();
    bool aux_write_register8(const uint8_t reg, const uint8_t val);
    bool aux_read_register8(const uint8_t reg, uint8_t& out);
    // Manual burst read of 8 bytes starting at a BMM150 register, via the AUX bridge
    bool aux_read_burst8(const uint8_t bmm150_reg, std::array<uint8_t, 8>& out);
    // Reads the BMM150 factory trim (registers 0x5D-0x71) through 3 overlapping 8 byte AUX
    // bursts (0x5D/0x65/0x6A) and maps them the same way as UnitBMM150::read_trim()
    bool aux_read_trim(bmm150::trim_t& trim);

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitBMI270, bmi270::Data);

private:
    config_t _cfg{};
    int16_t _last_temp{};             // Latest valid REG_TEMPERATURE_0 reading (register update is not per-sample)
    uint32_t _pushed{}, _consumed{};  // readImuRaw pop counters (do not disturb latest()/oldest())
    bool _aux_mag{false};             // True if an AUX BMM150 was detected and configured in begin()
    bmm150::trim_t _mag_trim{};       // Factory trim read via the AUX bridge, valid only if _aux_mag
    std::unique_ptr<m5::container::CircularBuffer<bmi270::Data>> _data{};
};

namespace bmi270 {
//! @brief Default ring buffer depth (used to construct the internal buffer and as the
//! component_config().stored_size default; see config_t)
constexpr uint32_t DEFAULT_STORED_SIZE{8};

namespace command {
///@cond
// Register addresses (I2C addr 0x68 (SDO=GND) / 0x69 (SDO=VDDIO, M5 board default))
constexpr uint8_t REG_DATA_0{0x04};         // AUX (mag) burst start: 20 bytes = mag(8)+accel(6)+gyro(6), see REG_DATA_8
constexpr uint8_t REG_STATUS{0x03};         // bit2 = aux_man_op busy (named REG_STATUS, not STATUS: ESP-IDF's
                                            // ets_sys.h has a global `typedef enum STATUS STATUS`)
constexpr uint8_t REG_DATA_8{0x0C};         // 12 bytes: accel X/Y/Z, gyro X/Y/Z (little-endian)
constexpr uint8_t REG_INT_STATUS_1{0x1D};   // bit7 acc_drdy, bit6 gyr_drdy, bit5 aux_drdy
constexpr uint8_t REG_TEMPERATURE_0{0x22};  // 2 bytes, little-endian, 0x8000 = invalid
constexpr uint8_t REG_INTERNAL_STATUS{0x21};
constexpr uint8_t REG_ACC_CONF{0x40};
constexpr uint8_t REG_ACC_RANGE{0x41};
constexpr uint8_t REG_GYR_CONF{0x42};
constexpr uint8_t REG_GYR_RANGE{0x43};
constexpr uint8_t REG_AUX_DEV_ID{0x4B};
constexpr uint8_t REG_AUX_IF_CONF{
    0x4C};  // bit7 manual_en, bit6 fcu_write_en, bits[3:2] man_rd_burst, bits[1:0] aux_rd_burst
constexpr uint8_t REG_AUX_RD_ADDR{0x4D};  // write triggers an AUX read transaction
constexpr uint8_t REG_AUX_WR_ADDR{0x4E};  // write triggers an AUX write transaction (of REG_AUX_WR_DATA)
constexpr uint8_t REG_AUX_WR_DATA{0x4F};
constexpr uint8_t REG_INIT_CTRL{0x59};
constexpr uint8_t REG_INIT_ADDR_0{0x5B};  // followed by INIT_ADDR_1 (0x5C)
constexpr uint8_t REG_INIT_DATA{0x5E};
constexpr uint8_t REG_IF_CONF{0x6B};   // bit5 aux_en
constexpr uint8_t REG_PWR_CONF{0x7C};  // bit0 adv_power_save
constexpr uint8_t REG_PWR_CTRL{0x7D};  // bit0 aux_en, bit1 gyr_en, bit2 acc_en, bit3 temp_en
constexpr uint8_t REG_CMD{0x7E};
constexpr uint8_t REG_CHIP_ID{0x00};

// BMM150 registers, reachable only indirectly through the AUX bridge above (distinct
// address space from this chip's own registers)
constexpr uint8_t REG_BMM150_CHIP_ID{0x40};
constexpr uint8_t REG_BMM150_DATA_X_LSB{0x42};
constexpr uint8_t REG_BMM150_POWER_CONTROL{0x4B};
constexpr uint8_t REG_BMM150_OP_MODE{0x4C};
///@endcond
}  // namespace command

//! @brief BMI270 chip identifier (value read from command::REG_CHIP_ID)
constexpr uint8_t CHIP_ID{0x24};

//! @brief Detect which I2C address a built-in BMI270 uses on a bus (0x68 or 0x69, board-dependent)
//! @details Probes both candidate addresses by reading CHIP_ID and reports the one where a BMI270
//! answers. Framework-agnostic: the caller injects the bus read, so the unit library stays free of
//! M5Unified / Arduino Wire. Hand the detected address to the constructor:
//! @code
//! uint8_t a{};
//! if (m5::unit::bmi270::detectAddress(read, a)) { m5::unit::UnitBMI270 unit{a}; }
//! @endcode
//! @tparam ReadReg Callable `bool(uint8_t addr, uint8_t reg, uint8_t& out)` reading one register
//!                 over the target bus and returning true on success (ACK)
//! @param  read     The read callable
//! @param[out] addr Detected address on success (unchanged on failure)
//! @return true if a BMI270 responded at 0x68 or 0x69
template <class ReadReg>
bool detectAddress(ReadReg read, uint8_t& addr)
{
    for (const uint8_t candidate : {static_cast<uint8_t>(0x68), static_cast<uint8_t>(0x69)}) {
        uint8_t id{};
        if (read(candidate, command::REG_CHIP_ID, id) && id == CHIP_ID) {
            addr = candidate;
            return true;
        }
    }
    return false;
}
}  // namespace bmi270

}  // namespace unit
}  // namespace m5
#endif
