/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mag3110.hpp
  @brief UnitMAG3110 (3-axis magnetometer) unit component
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_MAG3110_HPP
#define M5_UNIT_IMU_UNIT_UNIT_MAG3110_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <memory>
#include "unit_mag3110_data.hpp"
#include "../imu/imu_data_source.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitMAG3110
  @brief NXP/Freescale MAG3110 3-axis magnetometer. No standalone GROVE unit exists; it is the
  magnetometer on the original (2017) classic M5Stack Gray (paired with an MPU6050). It sits directly
  on the host I2C bus at 0x0E (no aux bypass needed, unlike the MPU9250's AK8963).
 */
class UnitMAG3110 : public Component,
                    public PeriodicMeasurementAdapter<UnitMAG3110, mag3110::Data>,
                    public m5::imu::IMUDataSource {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMAG3110, 0x0E);

public:
    /*!
      @struct config_t
      @brief Settings for begin()
     */
    struct config_t {
        //! Start periodic measurement on begin()?
        bool start_periodic{true};
        /*!
          Output data rate. Left at the chip's fastest setting for backwards compatibility, which is
          also its most expensive one (900µA); a slower rate cuts that by up to 100x. See
          mag3110::DataRate
         */
        mag3110::DataRate rate{mag3110::DataRate::Rate80Hz};
        //! Over sampling ratio, which sets the noise floor. See mag3110::OverSampling
        mag3110::OverSampling over_sampling{mag3110::OverSampling::X16};
    };

    //! @brief Construct with the I2C address the chip will answer on
    //! @param[in] addr I2C address (defaults to the chip's factory address)
    explicit UnitMAG3110(const uint8_t addr = DEFAULT_ADDRESS);
    //! @brief Destructor
    virtual ~UnitMAG3110()
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
    config_t config() const
    {
        return _cfg;
    }
    //! @brief Set the configuration
    void config(const config_t& cfg)
    {
        _cfg = cfg;
    }
    ///@}

    ///@name Reset
    ///@{
    //! @brief Soft reset (the MAG3110 has no reset command; returns to STANDBY via REG_CTRL_REG1),
    //! leaving the chip initialized but unconfigured and stopped
    //! @return True if successful
    //! @warning During periodic measurements, an error is returned; call stopPeriodicMeasurement() first
    //! @warning Measurement stops and settings must be reapplied; call startPeriodicMeasurement() to reconfigure
    //! and resume measurement
    bool softReset();
    ///@}

    ///@name Measurement setting (stop periodic measurement first)
    ///@{
    /*!
      @brief Write the output data rate and over sampling ratio
      @param[in] rate Output data rate
      @param[in] os Over sampling ratio
      @return True if successful
      @details Applied to the chip immediately, and to the periodic measurement interval
      @warning Fails while periodic measurement is running. Every REG_CTRL_REG1 field except AC can
      only be changed in STANDBY, and a write attempted in ACTIVE is silently ignored rather than
      reported -- so this drops the chip to STANDBY itself instead of trusting inPeriodic(), which
      says nothing about AC (stopPeriodicMeasurement() leaves the chip running by design; sleep() is
      what stops it)
      @note The datasheet notes the first output period after leaving STANDBY reads slightly off
     */
    bool writeDataRate(const mag3110::DataRate rate, const mag3110::OverSampling os);
    /*!
      @brief Read the output data rate and over sampling ratio
      @param[out] rate Output data rate
      @param[out] os Over sampling ratio
      @return True if successful
      @note Decoded from REG_CTRL_REG1, so it works while measuring as well as while stopped
     */
    bool readDataRate(mag3110::DataRate& rate, mag3110::OverSampling& os);
    ///@}

    ///@name Periodic measurement
    ///@{
    /*!
      @brief Start periodic measurement (auto magnetic reset + continuous ACTIVE)
      @param[in] rate Output data rate
      @param[in] os Over sampling ratio
      @return True if successful
     */
    bool startPeriodicMeasurement(const mag3110::DataRate rate, const mag3110::OverSampling os);
    //! @brief Start periodic measurement in the current settings
    //! @return True if successful
    bool startPeriodicMeasurement();
    //! @brief Mark periodic measurement as stopped (bookkeeping only; the chip stays in ACTIVE --
    //! call sleep() to drop to STANDBY for actual quiescence)
    //! @return True on success
    bool stopPeriodicMeasurement();
    ///@}

    ///@name Power
    ///@{
    /*!
      @brief Enter STANDBY mode (REG_CTRL_REG1 AC cleared)
      @return True if successful
      @details Works while periodic measurement is running: it is stopped first, since a
      standby chip produces no data. STANDBY is the MAG3110's low-power state.
     */
    bool sleep() override;
    /*!
      @brief Leave STANDBY
      @return True (always)
      @details No-op by design: on the MAG3110 STANDBY is also the state the chip has to be in
      to be configured, so there is nothing to undo here. Call startPeriodicMeasurement() to
      enter ACTIVE continuous measurement.
     */
    bool wakeup() override;
    ///@}

    ///@name IMUDataSource
    ///@{
    //! @brief Capability of this unit
    //! @return Mag (no temperature output exposed)
    virtual m5::imu::Capability imuSpec() const override
    {
        return m5::imu::Capability::Mag;
    }
    //! @brief Read the latest raw IMU data
    //! @param[out] out Destination buffer filled with the most recent sample
    //! @return Capability of the sample (which axes were populated)
    virtual m5::imu::Capability readImuRaw(m5::imu::imu_raw_data_t& out) override;
    //! @brief Gets the conversion parameters for raw IMU data
    //! @param[out] out Destination for the scale factors this driver applies
    virtual void imuConvertParam(m5::imu::imu_convert_param_t& out) const override;
    ///@}

protected:
    bool start_periodic_measurement(const mag3110::DataRate rate, const mag3110::OverSampling os);
    bool start_periodic_measurement();
    bool stop_periodic_measurement();
    bool read_measurement(mag3110::Data& d);

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitMAG3110, mag3110::Data);

private:
    config_t _cfg{};
    // Live state, seeded from _cfg by begin() and thereafter owned by writeDataRate(); _cfg is only
    // ever read in begin()
    mag3110::DataRate _rate{mag3110::DataRate::Rate80Hz};
    mag3110::OverSampling _os{mag3110::OverSampling::X16};
    uint32_t _pushed{}, _consumed{};
    std::unique_ptr<m5::container::CircularBuffer<mag3110::Data>> _data{};
};

namespace mag3110 {
//! @brief Default ring buffer depth
constexpr uint32_t DEFAULT_STORED_SIZE{4};

namespace command {
///@cond
constexpr uint8_t REG_DR_STATUS{0x00};  // burst start: DR_STATUS + X/Y/Z (big-endian) = 7 bytes
constexpr uint8_t REG_WHO_AM_I{0x07};   // = 0xC4
constexpr uint8_t REG_CTRL_REG1{0x10};  // DR[7:5] OS[4:3] FR[2] TM[1] AC[0]
constexpr uint8_t REG_CTRL_REG2{0x11};  // AUTO_MRST_EN[7] RAW[5] Mag_RST[4]
///@endcond
}  // namespace command
}  // namespace mag3110

}  // namespace unit
}  // namespace m5
#endif
