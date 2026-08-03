/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_bmm150.hpp
  @brief UnitBMM150 (3-axis magnetometer) unit component
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_BMM150_HPP
#define M5_UNIT_IMU_UNIT_UNIT_BMM150_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <memory>
#include "unit_bmm150_data.hpp"
#include "../imu/imu_data_source.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitBMM150
  @brief 3-axis magnetometer unit. No standalone GROVE unit exists for this
  chip; it is found built into some boards (e.g. classic M5Stack Core/Fire,
  paired with MPU6886) and as the auxiliary sensor of UnitBMI270
 */
class UnitBMM150 : public Component,
                   public PeriodicMeasurementAdapter<UnitBMM150, bmm150::Data>,
                   public m5::imu::IMUDataSource {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitBMM150, 0x10);

public:
    /*!
      @struct config_t
      @brief Settings for begin()
      @note Ring buffer depth is configured via component_config() (default:
      bmm150::DEFAULT_STORED_SIZE)
     */
    struct config_t {
        //! Start periodic measurement on begin()?
        bool start_periodic{true};
        //! Accuracy/power preset (controls REG_REP_XY/REG_REP_Z)
        bmm150::Preset preset{bmm150::Preset::Regular};
        //! Output data rate
        bmm150::DataRate rate{bmm150::DataRate::Rate30Hz};
    };

    //! @brief Construct with the I2C address the chip will answer on
    //! @param[in] addr I2C address (defaults to the chip's factory address)
    explicit UnitBMM150(const uint8_t addr = DEFAULT_ADDRESS);
    //! @brief Destructor
    virtual ~UnitBMM150()
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

    ///@name Setting (applies immediately)
    ///@{
    //! @brief Write the accuracy/power preset (REG_REP_XY/REG_REP_Z)
    //! @param[in] preset Preset
    //! @return True if successful, false if the preset cannot keep up with the current data rate
    //! @note Convenience wrapper around writeRepetition() with the preset's repetition values
    //! @warning During periodic measurement runs, an error is returned
    bool writePreset(const bmm150::Preset preset);
    /*!
      @brief Write the repetition (oversampling) registers directly
      @param[in] rep_xy REG_REP_XY value; the sensor performs 1 + 2*rep_xy repetitions on X/Y
      @param[in] rep_z REG_REP_Z value; the sensor performs 1 + rep_z repetitions on Z
      @return True if successful, false if the repetitions cannot keep up with the current data rate
      @details The four presets are only Bosch's recommendations -- both registers accept any 8bit
      value. More repetitions means less noise but a longer measurement, so the pair is rejected when
      it cannot finish inside one output period (bmm150::repetition_fits()); raise it here by slowing
      the rate first. Register values, not repetition counts, because nXY can only be odd
      @warning During periodic measurement runs, an error is returned
     */
    bool writeRepetition(const uint8_t rep_xy, const uint8_t rep_z);
    //! @brief Read the repetition registers from the device (REG_REP_XY/REG_REP_Z)
    //! @param[out] rep_xy REG_REP_XY value
    //! @param[out] rep_z REG_REP_Z value
    //! @return True if successful
    bool readRepetition(uint8_t& rep_xy, uint8_t& rep_z);
    //! @brief Write the output data rate
    //! @param[in] rate Data rate
    //! @return True if successful, false if the current repetitions cannot keep up with the new rate
    //! @warning During periodic measurement runs, an error is returned
    bool writeDataRate(const bmm150::DataRate rate);
    //! @brief Read the output data rate from the device (REG_OP_MODE bits[5:3])
    //! @param[out] rate Data rate
    //! @return True if successful
    bool readDataRate(bmm150::DataRate& rate);
    ///@}

    ///@name Reset
    ///@{
    //! @brief Soft reset (REG_POWER_CONTROL soft-reset bits, keeping the power control bit on),
    //! leaving the chip initialized but unconfigured and stopped
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
      @param rate Output data rate
      @param preset Accuracy/power preset (controls REG_REP_XY/REG_REP_Z)
      @return True if successful
    */
    bool startPeriodicMeasurement(const bmm150::DataRate rate, const bmm150::Preset preset);
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
      @brief Put the chip into suspend mode (REG_POWER_CONTROL power bit cleared)
      @return True if successful
      @details Works while periodic measurement is running: it is stopped first, since a
      suspended chip must not be polled. Suspend is the BMM150's lowest power state.
      @warning Suspend resets the register map, so the measurement settings do not survive it;
      startPeriodicMeasurement() re-applies the preset and data rate.
     */
    bool sleep() override;
    /*!
      @brief Bring the chip out of suspend
      @return True if successful
      @details Sets the power bit and waits for the chip to reach sleep mode, where its
      registers are writable again -- measurement stays stopped. Call
      startPeriodicMeasurement() to reconfigure and resume sampling. Harmless without a
      preceding sleep().
     */
    bool wakeup() override;
    ///@}

    ///@name IMUDataSource
    ///@{
    //! @brief Capability of this unit
    //! @return Mag (no temperature output on this chip)
    virtual m5::imu::Capability imuSpec() const override
    {
        return m5::imu::Capability::Mag;
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
    bool start_periodic_measurement(const bmm150::DataRate rate, const bmm150::Preset preset);
    bool start_periodic_measurement();
    bool stop_periodic_measurement();
    bool read_measurement(bmm150::Data& d);
    // Rate/repetition writers without the "does it fit" check. Needed because the public setters
    // validate against whatever is on the device right now, which makes a valid target pair fail
    // when applied one at a time: going from 30Hz/Regular to 10Hz/HighAccuracy has to pass through
    // one combination that does not fit whichever half is written first. start_periodic_measurement()
    // validates the pair up front instead and then writes both unchecked.
    bool write_repetition_unchecked(const uint8_t rep_xy, const uint8_t rep_z);
    bool write_data_rate_unchecked(const bmm150::DataRate rate);
    bool write_register8_verified(const uint8_t reg, const uint8_t val);
    bool read_trim(bmm150::trim_t& trim);

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitBMM150, bmm150::Data);

private:
    config_t _cfg{};
    bmm150::trim_t _trim{};
    uint32_t _pushed{}, _consumed{};  // readImuRaw pop counters (do not disturb latest()/oldest())
    std::unique_ptr<m5::container::CircularBuffer<bmm150::Data>> _data{};
};

namespace bmm150 {
//! @brief Default ring buffer depth (used to construct the internal buffer and as the
//! component_config().stored_size default; see config_t)
constexpr uint32_t DEFAULT_STORED_SIZE{4};

namespace command {
///@cond
// Register addresses (I2C addr 0x10/0x11/0x12/0x13 by CSB/SDO pins)
constexpr uint8_t REG_CHIP_ID{0x40};
constexpr uint8_t REG_DATA_X_LSB{0x42};  // 8 bytes: X/Y/Z/RHALL burst (bit0 of [6] = DRDY)
constexpr uint8_t REG_INT_STATUS{0x4A};
constexpr uint8_t REG_POWER_CONTROL{0x4B};  // bit7+bit1 = SoftReset / bit0 = PowerControl
constexpr uint8_t REG_OP_MODE{0x4C};        // bit5:3 = DataRate / bit2:1 = Opmode
constexpr uint8_t REG_REP_XY{0x51};
constexpr uint8_t REG_REP_Z{0x52};
constexpr uint8_t REG_TRIM_START{0x5D};  // 21 bytes: 0x5D..0x71 (see read_trim())
///@endcond
}  // namespace command
}  // namespace bmm150

}  // namespace unit
}  // namespace m5
#endif
