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
  @note Accel/gyro/temp only. The chip's auxiliary (AUX) interface, which can drive an on-chip
  BMM150 magnetometer, is deliberately left untouched here -- use UnitBMI270AuxEnabled on the
  boards that wire one to it (CoreS3, AtomS3R, Mini IMU-Pro). Enabling it is not always safe:
  on UnitDoF9/UnitDoF10 the AUX pins share the main I2C bus, and switching the interface on makes
  the BMI270 a second bus master, locking the bus until the unit is power-cycled
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
        //! Accelerometer output data rate (the chip's power-on default; matches gyro_odr so the
        //! AHRS layer gets accel and gyro at the same cadence)
        bmi270::AccelODR accel_odr{bmi270::AccelODR::Rate200Hz};
        //! Gyroscope output data rate (periodic measurement interval is derived from this). 200Hz is
        //! the chip's power-on default and keeps hand-speed motion out of the under-sampled regime;
        //! lower it if power matters more than tracking fast rotation
        bmi270::GyroODR gyro_odr{bmi270::GyroODR::Rate200Hz};
        //! Accelerometer filter performance mode. PowerOptimized also switches accel_bwp from
        //! filtering to averaging, which is what makes its Avg16 and up usable
        bmi270::FilterPerf accel_filter_perf{bmi270::FilterPerf::Performance};
        //! Accelerometer bandwidth parameter (the chip's power-on default). Its 3dB cutoff scales
        //! with accel_odr -- 89Hz at 200Hz in the normal filter. startPeriodicMeasurement() rejects
        //! the averaging-only values while accel_filter_perf is Performance (bmi270::bwp_fits())
        bmi270::AccelBwp accel_bwp{bmi270::AccelBwp::NormAvg4};
        //! Gyroscope filter performance mode
        bmi270::FilterPerf gyro_filter_perf{bmi270::FilterPerf::Performance};
        //! Gyroscope bandwidth parameter (the chip's power-on default)
        bmi270::GyroBwp gyro_bwp{bmi270::GyroBwp::Norm};
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

    ///@name Range setting (stop periodic measurement first)
    ///@{
    //! @brief Write the accelerometer range
    //! @param[in] range Accelerometer range
    //! @return True if successful
    //! @warning During periodic measurement runs, an error is returned
    bool writeAccelRange(const bmi270::AccelRange range);
    //! @brief Write the gyroscope range
    //! @param[in] range Gyroscope range
    //! @return True if successful
    //! @warning During periodic measurement runs, an error is returned
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

    ///@name Reset
    ///@{
    //! @brief Soft reset (REG_CMD CMD_SOFTRESET), then re-run the mandatory post-reset initialization
    //! (config blob re-upload), leaving the chip initialized but unconfigured and stopped
    //! @return True if successful
    //! @warning During periodic measurements, an error is returned; call stopPeriodicMeasurement() first
    //! @warning All settings revert to the chip defaults; call startPeriodicMeasurement(...) to reconfigure and
    //! resume measurement
    bool softReset();
    ///@}

    ///@name Periodic measurement
    ///@{
    /*!
      @brief Start periodic measurement
      @param accel_range Accelerometer range
      @param gyro_range Gyroscope range
      @param accel_odr Accelerometer output data rate
      @param gyro_odr Gyroscope output data rate (periodic measurement interval is derived from this)
      @return True if successful
      @note UnitBMI270AuxEnabled overrides this to also (re)probe and configure its AUX BMM150
    */
    bool startPeriodicMeasurement(const bmi270::AccelRange accel_range, const bmi270::GyroRange gyro_range,
                                  const bmi270::AccelODR accel_odr, const bmi270::GyroODR gyro_odr);
    //! @brief Start periodic measurement in the current settings
    //! @return True if successful
    bool startPeriodicMeasurement();
    //! @brief Stop periodic measurement
    //! @return True if successful
    bool stopPeriodicMeasurement();
    ///@}

    ///@name Power
    ///@{
    /*!
      @brief Suspend the chip (REG_PWR_CTRL off, REG_PWR_CONF adv_power_save)
      @return True if successful
      @details Works while periodic measurement is running: it is stopped first, since a
      suspended chip must not be polled. Then accel/gyro/temp are powered down and advanced power
      save is enabled. The uploaded
      configuration file survives suspend -- unlike softReset(), which requires the full
      post-reset initialization again.
     */
    bool sleep() override;
    /*!
      @brief Resume the chip from sleep()
      @return True if successful
      @details Leaves advanced power save (configuration writes are ignored while it is active)
      and powers accel/gyro/temp back on -- measurement stays stopped. Call
      startPeriodicMeasurement() to resume sampling. Harmless without a preceding sleep().
     */
    bool wakeup() override;
    ///@}

    ///@name IMUDataSource
    ///@{
    //! @brief Capability of this unit
    //! @return Accel | Gyro | Temp. This class leaves the AUX interface alone, so never Mag --
    //! see UnitBMI270AuxEnabled for the boards that wire a magnetometer to it
    virtual m5::imu::Capability imuSpec() const override
    {
        using m5::imu::Capability;
        return Capability::Accel | Capability::Gyro | Capability::Temp;
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
    virtual bool start_periodic_measurement(const bmi270::AccelRange accel_range, const bmi270::GyroRange gyro_range,
                                            const bmi270::AccelODR accel_odr, const bmi270::GyroODR gyro_odr);
    bool start_periodic_measurement();
    bool stop_periodic_measurement();
    bool read_measurement(bmi270::Data& d);
    //! @brief Read one sample's sensor registers into d
    //! @details Split out of read_measurement() so a subclass can widen the burst without repeating
    //! the temperature handling that follows it (see UnitBMI270AuxEnabled)
    virtual bool read_sensor_burst(bmi270::Data& d);
    bool write_register8_verified(const uint8_t reg, const uint8_t val);
    // Uploads the 8192 byte configuration blob (unit_bmi270_config.inl) to REG_INIT_DATA,
    // split into chunks that fit within the I2C write buffer (see .cpp for chunk size
    // rationale). Returns false (and logs) on the first failed chunk, unlike M5Unified's
    // BMI270_Class::_upload_file() which always returns true regardless of write result.
    bool upload_config();

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitBMI270, bmi270::Data);

private:
    config_t _cfg{};
    int16_t _last_temp{};             // Latest valid REG_TEMPERATURE_0 reading (register update is not per-sample)
    uint32_t _pushed{}, _consumed{};  // readImuRaw pop counters (do not disturb latest()/oldest())
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

// BMM150 registers, reachable only indirectly through the AUX bridge above (distinct address
// space from this chip's own registers). Only UnitBMI270AuxEnabled talks to them; they live here
// because they are addressed through this chip's AUX registers
constexpr uint8_t REG_BMM150_CHIP_ID{0x40};
constexpr uint8_t REG_BMM150_DATA_X_LSB{0x42};
constexpr uint8_t REG_BMM150_POWER_CONTROL{0x4B};
constexpr uint8_t REG_BMM150_OP_MODE{0x4C};
constexpr uint8_t REG_BMM150_REP_XY{0x51};
constexpr uint8_t REG_BMM150_REP_Z{0x52};
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
