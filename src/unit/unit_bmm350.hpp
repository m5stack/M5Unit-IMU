/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_bmm350.hpp
  @brief UnitBMM350 (3-axis magnetometer, U231) unit component
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_BMM350_HPP
#define M5_UNIT_IMU_UNIT_UNIT_BMM350_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <memory>
#include "unit_bmm350_data.hpp"
#include "../imu/imu_data_source.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitBMM350
  @brief 3-axis magnetometer unit (U231, GROVE)
 */
class UnitBMM350 : public Component,
                   public PeriodicMeasurementAdapter<UnitBMM350, bmm350::Data>,
                   public m5::imu::IMUDataSource {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitBMM350, 0x14);

public:
    /*!
      @struct config_t
      @brief Settings for begin()
      @note Ring buffer depth is configured via component_config() (default:
      bmm350::DEFAULT_STORED_SIZE)
     */
    struct config_t {
        //! Start periodic measurement on begin()?
        bool start_periodic{true};
        //! Output data rate
        bmm350::DataRate rate{bmm350::DataRate::Rate100Hz};
        //! Averaging factor (clamped to what the rate accepts, see clamp_averaging())
        bmm350::Averaging avg{bmm350::Averaging::X4};
    };

    explicit UnitBMM350(const uint8_t addr = DEFAULT_ADDRESS);
    virtual ~UnitBMM350()
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
    //! @brief Write the output data rate (averaging is re-clamped to the new rate)
    //! @param[in] rate Data rate
    //! @return True if successful
    bool writeDataRate(const bmm350::DataRate rate);
    //! @brief Write the averaging factor (clamped to what the current rate accepts)
    //! @param[in] avg Averaging factor
    //! @return True if successful
    bool writeAveraging(const bmm350::Averaging avg);
    //! @brief Read the output data rate from the device (REG_PMU_CMD_AGGR_SET bits[3:0])
    //! @param[out] rate Data rate
    //! @return True if successful, false if the register holds a reserved (unmapped) value
    bool readDataRate(bmm350::DataRate& rate);
    //! @brief Read the averaging factor from the device (REG_PMU_CMD_AGGR_SET bits[5:4])
    //! @param[out] avg Averaging factor
    //! @return True if successful
    bool readAveraging(bmm350::Averaging& avg);
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
    //! @return Mag | Temp
    virtual m5::imu::Capability imuSpec() const override
    {
        return m5::imu::Capability::Mag | m5::imu::Capability::Temp;
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
    bool read_measurement(bmm350::Data& d);
    bool write_register8_verified(const uint8_t reg, const uint8_t val);
    // BMM350 I2C reads are always prefixed with 2 dummy bytes; every register read
    // (including REG_CHIP_ID) must go through this wrapper.
    bool read_registers(const uint8_t reg, uint8_t* out, const uint32_t len);
    bool read_register8(const uint8_t reg, uint8_t& out);
    bool download_otp(bmm350::otp_compensation_t& comp);
    // AGGR_SET->UPD_OAE->SUS->Normal tail shared by begin() and the writeDataRate()/
    // writeAveraging() setters (Bosch bmm350_set_odr_performance() + bmm350_set_powermode():
    // the SUS step is always issued here since REG_PMU_CMD was just left at UPD_OAE). Verifies
    // REG_PMU_CMD_STATUS_0.pwr_mode_is_normal after the Normal transition.
    bool commit_odr_avg(const bmm350::DataRate rate, const bmm350::Averaging avg);

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitBMM350, bmm350::Data);

private:
    config_t _cfg{};
    bmm350::otp_compensation_t _comp{};
    uint32_t _pushed{}, _consumed{};  // readImuRaw pop counters (do not disturb latest()/oldest())
    std::unique_ptr<m5::container::CircularBuffer<bmm350::Data>> _data{};
};

namespace bmm350 {
//! @brief Default ring buffer depth (used to construct the internal buffer and as the
//! component_config().stored_size default; see config_t)
constexpr uint32_t DEFAULT_STORED_SIZE{8};

namespace command {
///@cond
// Register addresses (I2C addr 0x14 (ADSEL=LOW) / 0x15 (ADSEL=HIGH))
constexpr uint8_t REG_CHIP_ID{0x00};
constexpr uint8_t REG_PMU_CMD_AGGR_SET{0x04};  // bits[3:0]=odr, bits[5:4]=avg
constexpr uint8_t REG_PMU_CMD{0x06};
constexpr uint8_t REG_PMU_CMD_STATUS_0{0x07};  // bit3=pwr_mode_is_normal, bit4=cmd_is_illegal
constexpr uint8_t REG_INT_CTRL{0x2E};          // bit7 = drdy_data_reg_en
constexpr uint8_t REG_INT_STATUS{0x30};        // bit2 = drdy_data_reg (clears on read)
constexpr uint8_t REG_MAG_X_XLSB{0x31};        // 12 bytes: mag X/Y/Z (24bit each) + temperature (24bit)
constexpr uint8_t REG_OTP_CMD{0x50};
constexpr uint8_t REG_OTP_DATA_MSB{0x52};
constexpr uint8_t REG_OTP_DATA_LSB{0x53};
constexpr uint8_t REG_OTP_STATUS{0x55};
constexpr uint8_t REG_CMD{0x7E};
///@endcond
}  // namespace command
}  // namespace bmm350

}  // namespace unit
}  // namespace m5
#endif
