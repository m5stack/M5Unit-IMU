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
     */
    struct config_t {
        //! Start periodic measurement on begin()?
        bool start_periodic{true};
        //! Accuracy/power preset (controls REP_XY/REP_Z)
        bmm150::Preset preset{bmm150::Preset::Regular};
        //! Output data rate
        bmm150::DataRate rate{bmm150::DataRate::Rate30Hz};
        //! Ring buffer depth. This unit treats config_t as the sole control surface;
        //! component_config().stored_size is synced to this value in begin()
        uint32_t stored_size{4};
    };

    explicit UnitBMM150(const uint8_t addr = DEFAULT_ADDRESS);
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
    //! @brief Write the accuracy/power preset (REP_XY/REP_Z)
    //! @param[in] preset Preset
    //! @return True if successful
    bool writePreset(const bmm150::Preset preset);
    //! @brief Write the output data rate
    //! @param[in] rate Data rate
    //! @return True if successful
    bool writeDataRate(const bmm150::DataRate rate);
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
    bool start_periodic_measurement();
    bool stop_periodic_measurement();
    bool read_measurement(bmm150::Data& d);
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
namespace command {
///@cond
// Register addresses (I2C addr 0x10/0x11/0x12/0x13 by CSB/SDO pins)
constexpr uint8_t CHIP_ID       = 0x40;
constexpr uint8_t DATA_X_LSB    = 0x42;  // 8 bytes: X/Y/Z/RHALL burst (bit0 of [6] = DRDY)
constexpr uint8_t INT_STATUS    = 0x4A;
constexpr uint8_t POWER_CONTROL = 0x4B;  // bit7+bit1 = SoftReset / bit0 = PowerControl
constexpr uint8_t OP_MODE       = 0x4C;  // bit5:3 = DataRate / bit2:1 = Opmode
constexpr uint8_t REP_XY        = 0x51;
constexpr uint8_t REP_Z         = 0x52;
constexpr uint8_t TRIM_START    = 0x5D;  // 21 bytes: 0x5D..0x71 (see read_trim())

constexpr uint8_t WHOAMI_VALUE = 0x32;
constexpr uint8_t POWER_ON     = 0x01;
// bit7 SoftReset + bit1 SoftReset + bit0 PowerControl combined.
// Not modeled as an enum because it aliases distinct bit fields (m5-cpp-rules)
constexpr uint8_t SOFT_RESET_AND_POWER_ON = 0x83;
constexpr uint8_t OPMODE_NORMAL           = 0x00;
///@endcond
}  // namespace command
}  // namespace bmm150

}  // namespace unit
}  // namespace m5
#endif
