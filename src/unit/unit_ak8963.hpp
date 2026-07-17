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
    };

    explicit UnitAK8963(const uint8_t addr = DEFAULT_ADDRESS);
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

    ///@name Periodic measurement
    ///@{
    //! @brief Start periodic measurement (16-bit continuous 100Hz)
    bool startPeriodicMeasurement();
    //! @brief Stop periodic measurement
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
    virtual m5::imu::Capability readImuRaw(m5::imu::imu_raw_data_t& out) override;
    //! @brief Gets the conversion parameters for raw IMU data
    virtual void imuConvertParam(m5::imu::imu_convert_param_t& out) const override;
    ///@}

protected:
    bool start_periodic_measurement();
    bool stop_periodic_measurement();
    bool read_measurement(ak8963::Data& d);
    bool read_sensitivity(std::array<uint8_t, 3>& asa);

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitAK8963, ak8963::Data);

private:
    config_t _cfg{};
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
