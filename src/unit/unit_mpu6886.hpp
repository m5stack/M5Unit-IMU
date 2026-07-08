/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6886.hpp
  @brief UnitMPU6886 (6-axis IMU) unit component
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_MPU6886_HPP
#define M5_UNIT_IMU_UNIT_UNIT_MPU6886_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <memory>
#include "unit_mpu6886_data.hpp"
#include "../imu/imu_data_source.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitMPU6886
  @brief 6-axis (accel + gyro) IMU unit. UnitIMUMini and Core2/StickC built-in
 */
class UnitMPU6886 : public Component,
                    public PeriodicMeasurementAdapter<UnitMPU6886, mpu6886::Data>,
                    public m5::imu::IMUDataSource {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMPU6886, 0x68);

public:
    /*!
      @struct config_t
      @brief Settings for begin()
     */
    struct config_t {
        //! Start periodic measurement on begin()?
        bool start_periodic{true};
        //! Accelerometer range
        mpu6886::AccelRange accel_range{mpu6886::AccelRange::G8};
        //! Gyroscope range
        mpu6886::GyroRange gyro_range{mpu6886::GyroRange::DPS2000};
        //! Gyro/temp low pass filter
        mpu6886::Dlpf dlpf{mpu6886::Dlpf::BW176};
        //! Sample rate divider (ODR = 1000/(1+div) Hz). 3 -> 250Hz
        uint8_t sample_rate_div{3};
        //! Ring buffer depth. This unit treats config_t as the sole control surface;
        //! component_config().stored_size is synced to this value in begin()
        uint32_t stored_size{8};
    };

    explicit UnitMPU6886(const uint8_t addr = DEFAULT_ADDRESS);
    virtual ~UnitMPU6886()
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
    bool writeAccelRange(const mpu6886::AccelRange range);
    //! @brief Write the gyroscope range
    //! @param[in] range Gyroscope range
    //! @return True if successful
    bool writeGyroRange(const mpu6886::GyroRange range);
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
    bool read_measurement(mpu6886::Data& d);
    bool write_register8_verified(const uint8_t reg, const uint8_t val);

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitMPU6886, mpu6886::Data);

private:
    config_t _cfg{};
    uint32_t _pushed{}, _consumed{};  // readImuRaw pop counters (do not disturb latest()/oldest())
    std::unique_ptr<m5::container::CircularBuffer<mpu6886::Data>> _data{};
};

namespace mpu6886 {
namespace command {
///@cond
// Register addresses (I2C addr 0x68/0x69 by AD0 pin)
constexpr uint8_t GYRO_OFFS_USR = 0x13;  // 6 bytes
constexpr uint8_t SMPLRT_DIV    = 0x19;
constexpr uint8_t CONFIG        = 0x1A;
constexpr uint8_t GYRO_CONFIG   = 0x1B;
constexpr uint8_t ACCEL_CONFIG  = 0x1C;
constexpr uint8_t ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t FIFO_EN       = 0x23;
constexpr uint8_t INT_PIN_CFG   = 0x37;
constexpr uint8_t INT_ENABLE    = 0x38;
constexpr uint8_t INT_STATUS    = 0x3A;  // bit0 = DATA_RDY (read clear)
constexpr uint8_t ACCEL_XOUT_H  = 0x3B;  // 14 bytes: accel6 + temp2 + gyro6 (big-endian)
constexpr uint8_t USER_CTRL     = 0x6A;
constexpr uint8_t PWR_MGMT_1    = 0x6B;  // bit7 RESET / bit6 SLEEP / CLKSEL[2:0]
constexpr uint8_t PWR_MGMT_2    = 0x6C;
constexpr uint8_t WHO_AM_I      = 0x75;

constexpr uint8_t WHOAMI_VALUE = 0x19;
// CLKSEL: 0,6 = internal 20MHz (alias) / 1..5 = auto PLL (alias) / 7 = stop
// Not modeled as an enum because of duplicate values (m5-cpp-rules). Use 1 (auto PLL) for full performance
constexpr uint8_t CLKSEL_AUTO_PLL = 0x01;
constexpr uint8_t DEVICE_RESET    = 0x80;
///@endcond
}  // namespace command
}  // namespace mpu6886

}  // namespace unit
}  // namespace m5
#endif
