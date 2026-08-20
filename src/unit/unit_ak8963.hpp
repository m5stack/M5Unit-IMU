/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_ak8963.hpp
  @brief UnitAK8963 (3-axis magnetometer) unit component
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_AK8963_HPP
#define M5_UNIT_IMU_UNIT_UNIT_AK8963_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <memory>
#include "unit_ak8963_data.hpp"
#include "../imu/imu_data_source.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitAK8963
  @brief 3-axis magnetometer unit. No standalone GROVE unit exists; it is the
  magnetometer embedded in the MPU9250 found on early classic M5Stack Core/Fire
  boards, reachable at 0x0C once the MPU9250's I2C bypass is enabled (see
  m5::unit::imu::enableAuxBypass)
 */
class UnitAK8963 : public Component,
                   public PeriodicMeasurementAdapter<UnitAK8963, ak8963::Data>,
                   public m5::imu::IMUDataSource {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitAK8963, 0x0C);

public:
    /*!
      @struct config_t
      @brief Settings for begin()
     */
    struct config_t {
        //! Start periodic measurement on begin()?
        bool start_periodic{true};
        /*!
          Continuous measurement rate. Cont100Hz keeps the 5mA magnetic drive on almost
          continuously; Cont8Hz cuts that to roughly a 6% duty cycle. See ak8963::MeasurementMode
         */
        ak8963::MeasurementMode measurement_mode{ak8963::MeasurementMode::Cont100Hz};
        //! Output bit width. Full scale is the same either way, only the resolution differs
        ak8963::OutputBit output_bit{ak8963::OutputBit::Bit16};
    };

    //! @brief Construct with the I2C address the chip will answer on
    //! @param[in] addr I2C address (defaults to the chip's factory address)
    explicit UnitAK8963(const uint8_t addr = DEFAULT_ADDRESS);
    //! @brief Destructor
    virtual ~UnitAK8963()
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
    //! @brief Soft reset (REG_CNTL2 SRST), leaving the chip initialized but unconfigured and stopped
    //! @return True if successful
    //! @warning During periodic measurements, an error is returned; call stopPeriodicMeasurement() first
    //! @warning All settings revert to the chip defaults; call startPeriodicMeasurement() to reconfigure and
    //! resume measurement
    bool softReset();
    ///@}

    ///@name Measurement setting (stop periodic measurement first)
    ///@{
    /*!
      @brief Write the continuous measurement rate and output bit width
      @param[in] mode Continuous measurement rate
      @param[in] bit Output bit width
      @return True if successful
      @details Applied to the chip by the next startPeriodicMeasurement(), along with the periodic
      measurement interval. Entering a continuous mode *is* measuring on this chip, so there is no
      way to park the setting in REG_CNTL1 while stopped -- what this does write is the power-down
      every mode transition has to pass through
      @warning Fails while periodic measurement is running
     */
    bool writeMeasurementMode(const ak8963::MeasurementMode mode, const ak8963::OutputBit bit);
    /*!
      @brief Read the continuous measurement rate and output bit width
      @param[out] mode Continuous measurement rate
      @param[out] bit Output bit width
      @return True if successful
      @note Returns the setting this unit measures with, not a register read: REG_CNTL1 holds
      power-down whenever measurement is stopped, so the chip cannot answer this while idle
     */
    bool readMeasurementMode(ak8963::MeasurementMode& mode, ak8963::OutputBit& bit) const;
    ///@}

    ///@name Periodic measurement
    ///@{
    /*!
      @brief Start periodic measurement
      @param[in] mode Continuous measurement rate
      @param[in] bit Output bit width
      @return True if successful
     */
    bool startPeriodicMeasurement(const ak8963::MeasurementMode mode, const ak8963::OutputBit bit);
    //! @brief Start periodic measurement in the current settings
    //! @return True if successful
    bool startPeriodicMeasurement();
    //! @brief Mark periodic measurement as stopped (bookkeeping only; the chip stays in its
    //! current mode -- call sleep() to power-down for actual quiescence)
    //! @return True on success
    bool stopPeriodicMeasurement();
    ///@}

    ///@name Power
    ///@{
    /*!
      @brief Enter power-down mode (REG_CNTL1 mode 0)
      @return True if successful
      @details Works while periodic measurement is running: it is stopped first, since a
      powered-down chip must not be polled. Power-down is the AK8963's only low-power state.
     */
    bool sleep() override;
    /*!
      @brief Leave power-down
      @return True (always)
      @details No-op by design: on the AK8963 power-down is also the state every mode change
      has to pass through, so there is nothing to undo here. Call startPeriodicMeasurement() to
      re-enter continuous measurement.
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
    //! @param[out] out Destination buffer filled with the most recent sample
    //! @return Capability of the sample (which axes were populated)
    virtual m5::imu::Capability readImuRaw(m5::imu::imu_raw_data_t& out) override;
    //! @brief Gets the conversion parameters for raw IMU data
    //! @param[out] out Destination for the scale factors this driver applies
    virtual void imuConvertParam(m5::imu::imu_convert_param_t& out) const override;
    ///@}

protected:
    bool start_periodic_measurement(const ak8963::MeasurementMode mode, const ak8963::OutputBit bit);
    bool start_periodic_measurement();
    bool stop_periodic_measurement();
    bool read_measurement(ak8963::Data& d);
    bool read_sensitivity(std::array<uint8_t, 3>& asa);

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitAK8963, ak8963::Data);

private:
    config_t _cfg{};
    // Live state, seeded from _cfg by begin() and thereafter owned by writeMeasurementMode(); _cfg
    // is only ever read in begin()
    ak8963::MeasurementMode _mode{ak8963::MeasurementMode::Cont100Hz};
    ak8963::OutputBit _bit{ak8963::OutputBit::Bit16};
    std::array<uint8_t, 3> _asa{};  // Per-axis sensitivity adjustment (read once in begin, FUSE-ROM)
    uint32_t _pushed{}, _consumed{};
    std::unique_ptr<m5::container::CircularBuffer<ak8963::Data>> _data{};
};

namespace ak8963 {
//! @brief Default ring buffer depth
constexpr uint32_t DEFAULT_STORED_SIZE{4};

namespace command {
///@cond
constexpr uint8_t REG_WIA{0x00};    // WHO_AM_I = 0x48
constexpr uint8_t REG_ST1{0x02};    // burst start: ST1 + X/Y/Z(LE) + ST2 = 8 bytes
constexpr uint8_t REG_CNTL1{0x0A};  // bits[3:0] mode, bit4 = 16bit
constexpr uint8_t REG_CNTL2{0x0B};  // bit0 = SRST
constexpr uint8_t REG_ASAX{0x10};   // ASAX/ASAY/ASAZ (FUSE-ROM only)
///@endcond
}  // namespace command
}  // namespace ak8963

}  // namespace unit
}  // namespace m5
#endif
