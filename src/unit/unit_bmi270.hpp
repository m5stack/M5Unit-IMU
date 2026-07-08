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
#include "../imu/imu_data_source.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitBMI270
  @brief 6-axis (accel + gyro) IMU unit. CoreS3/AtomS3R built-in
  @note The AUX interface (on-chip BMM150 magnetometer bridge, as wired on
  AtomS3R) is not yet supported by this class
 */
class UnitBMI270 : public Component,
                   public PeriodicMeasurementAdapter<UnitBMI270, bmi270::Data>,
                   public m5::imu::IMUDataSource {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitBMI270, 0x69);

public:
    /*!
      @struct config_t
      @brief Settings for begin()
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
        //! Ring buffer depth. This unit treats config_t as the sole control surface;
        //! component_config().stored_size is synced to this value in begin()
        uint32_t stored_size{8};
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
    //! @return Accel | Gyro | Temp
    virtual m5::imu::Capability imuSpec() const override
    {
        return m5::imu::Capability::Accel | m5::imu::Capability::Gyro | m5::imu::Capability::Temp;
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
    // Uploads the 8192 byte configuration blob (unit_bmi270_config.inl) to INIT_DATA,
    // split into chunks that fit within the I2C write buffer (see .cpp for chunk size
    // rationale). Returns false (and logs) on the first failed chunk, unlike M5Unified's
    // BMI270_Class::_upload_file() which always returns true regardless of write result.
    bool upload_config();

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitBMI270, bmi270::Data);

private:
    config_t _cfg{};
    int16_t _last_temp{};             // Latest valid TEMPERATURE_0 reading (register update is not per-sample)
    uint32_t _pushed{}, _consumed{};  // readImuRaw pop counters (do not disturb latest()/oldest())
    std::unique_ptr<m5::container::CircularBuffer<bmi270::Data>> _data{};
};

namespace bmi270 {
namespace command {
///@cond
// Register addresses (I2C addr 0x68 (SDO=GND) / 0x69 (SDO=VDDIO, M5 board default))
constexpr uint8_t DATA_8          = 0x0C;  // 12 bytes: accel X/Y/Z, gyro X/Y/Z (little-endian)
constexpr uint8_t INT_STATUS_1    = 0x1D;  // bit7 acc_drdy, bit6 gyr_drdy
constexpr uint8_t TEMPERATURE_0   = 0x22;  // 2 bytes, little-endian, 0x8000 = invalid
constexpr uint8_t INTERNAL_STATUS = 0x21;
constexpr uint8_t ACC_CONF        = 0x40;
constexpr uint8_t ACC_RANGE       = 0x41;
constexpr uint8_t GYR_CONF        = 0x42;
constexpr uint8_t GYR_RANGE       = 0x43;
constexpr uint8_t INIT_CTRL       = 0x59;
constexpr uint8_t INIT_ADDR_0     = 0x5B;  // followed by INIT_ADDR_1 (0x5C)
constexpr uint8_t INIT_DATA       = 0x5E;
constexpr uint8_t PWR_CONF        = 0x7C;  // bit0 adv_power_save
constexpr uint8_t PWR_CTRL        = 0x7D;  // bit0 aux_en, bit1 gyr_en, bit2 acc_en, bit3 temp_en
constexpr uint8_t CMD             = 0x7E;
constexpr uint8_t CHIP_ID         = 0x00;

constexpr uint8_t CHIP_ID_VALUE = 0x24;
constexpr uint8_t CMD_SOFTRESET = 0xB6;

// INT_STATUS_1 (0x1D) bits
constexpr uint8_t INT_STATUS_1_GYR_DRDY = 0x40;  // bit6
constexpr uint8_t INT_STATUS_1_ACC_DRDY = 0x80;  // bit7

// INTERNAL_STATUS (0x21) message field, bits[3:0]. Not enum'd: 0x00 (not_init) is a
// transient polling state, not a selectable setting (m5-cpp-rules)
constexpr uint8_t INTERNAL_STATUS_MESSAGE_MASK = 0x0F;
constexpr uint8_t INTERNAL_STATUS_INIT_OK      = 0x01;

// ACC_CONF (0x40) / GYR_CONF (0x42): bit7 filter_perf=1 (high performance), bits[5:4] bwp=2 (normal)
constexpr uint8_t CONF_FILTER_PERF = 0x80;
constexpr uint8_t CONF_BWP_NORMAL  = 0x20;

// PWR_CTRL (0x7D): temp_en | acc_en | gyr_en (aux_en left off; see AUX support note in unit_bmi270.hpp)
constexpr uint8_t PWR_CTRL_ACC_GYR_TEMP_ON = 0x0E;
///@endcond
}  // namespace command
}  // namespace bmi270

}  // namespace unit
}  // namespace m5
#endif
