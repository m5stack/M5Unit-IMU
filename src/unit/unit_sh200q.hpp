/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_sh200q.hpp
  @brief UnitSH200Q (6-axis accel + gyro) unit component
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_SH200Q_HPP
#define M5_UNIT_IMU_UNIT_UNIT_SH200Q_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <memory>
#include "unit_sh200q_data.hpp"
#include "../imu/imu_data_source.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitSH200Q
  @brief Senodia SH200Q 6-axis IMU (accel + gyro + temp). No standalone GROVE unit exists; it is
  the built-in IMU of the earliest (2019) M5StickC, on the internal I2C bus at 0x6C.
  @warning Field reports (and the chip's EOL status) suggest the gyroscope output is of poor
  quality (large per-axis zero offsets) and that the accel and gyro axis systems may not even
  agree with each other or with the datasheet -- verify the orientation on hardware and expect to
  need gyro zero-offset calibration. The accelerometer is the dependable half of this chip.
 */
class UnitSH200Q : public Component,
                   public PeriodicMeasurementAdapter<UnitSH200Q, sh200q::Data>,
                   public m5::imu::IMUDataSource {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitSH200Q, 0x6C);

public:
    /*!
      @struct config_t
      @brief Settings for begin()
      @note Defaults match the original M5StickC library (8G / 2000dps / 256Hz / 500Hz / DLPF 50Hz)
     */
    struct config_t {
        //! Start periodic measurement on begin()?
        bool start_periodic{true};
        //! Accelerometer range
        sh200q::AccelRange accel_range{sh200q::AccelRange::G8};
        //! Gyroscope range
        sh200q::GyroRange gyro_range{sh200q::GyroRange::DPS2000};
        //! Accelerometer output data rate
        sh200q::AccelOdr accel_odr{sh200q::AccelOdr::HZ256};
        //! Gyroscope output data rate (also paces the periodic measurement)
        sh200q::GyroOdr gyro_odr{sh200q::GyroOdr::HZ500};
        //! Gyro low pass filter
        sh200q::GyroDlpf gyro_dlpf{sh200q::GyroDlpf::BW50};
    };

    explicit UnitSH200Q(const uint8_t addr = DEFAULT_ADDRESS);
    virtual ~UnitSH200Q()
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

    ///@name Range setting (applies immediately)
    ///@{
    //! @brief Write the accelerometer range
    //! @param[in] range Accelerometer range
    //! @return True if successful
    bool writeAccelRange(const sh200q::AccelRange range);
    //! @brief Write the gyroscope range
    //! @param[in] range Gyroscope range
    //! @return True if successful
    bool writeGyroRange(const sh200q::GyroRange range);
    //! @brief Read the accelerometer range from the device (REG_ACC_RANGE bits[1:0])
    //! @param[out] range Accelerometer range
    //! @return True if successful
    bool readAccelRange(sh200q::AccelRange& range);
    //! @brief Read the gyroscope range from the device (REG_GYRO_RANGE bits[2:0])
    //! @param[out] range Gyroscope range
    //! @return True if successful
    bool readGyroRange(sh200q::GyroRange& range);
    ///@}

    ///@name Periodic measurement
    ///@{
    //! @brief Start periodic measurement in the current settings
    bool startPeriodicMeasurement();
    //! @brief Stop periodic measurement
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
    virtual m5::imu::Capability readImuRaw(m5::imu::imu_raw_data_t& out) override;
    //! @brief Gets the conversion parameters for raw IMU data
    virtual void imuConvertParam(m5::imu::imu_convert_param_t& out) const override;
    ///@}

protected:
    bool start_periodic_measurement();
    bool stop_periodic_measurement();
    bool read_measurement(sh200q::Data& d);
    bool write_register8_verified(const uint8_t reg, const uint8_t val);
    bool toggle_register_bits(const uint8_t reg, const uint8_t bits);

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitSH200Q, sh200q::Data);

private:
    config_t _cfg{};
    uint32_t _pushed{}, _consumed{};  // readImuRaw pop counters (do not disturb latest()/oldest())
    std::unique_ptr<m5::container::CircularBuffer<sh200q::Data>> _data{};
};

namespace sh200q {
//! @brief Default ring buffer depth
constexpr uint32_t DEFAULT_STORED_SIZE{8};

namespace command {
///@cond
constexpr uint8_t REG_OUTPUT{0x00};       // burst: accel6 + gyro6 + temp2 (little-endian) = 14 bytes
constexpr uint8_t REG_ACC_CONFIG{0x0E};   // [7] HPF bypass / [4:3] ODR / [0] filter enable
constexpr uint8_t REG_GYRO_CONFIG{0x0F};  // [4] HPF bypass / [3:1] ODR / [0] filter enable
constexpr uint8_t REG_GYRO_DLPF{0x11};    // low bits: DLPF setting (ODR-relative bandwidth)
constexpr uint8_t REG_FIFO_CONFIG{0x12};  // [7:6] buffer mode (00 = bypass)
constexpr uint8_t REG_ACC_RANGE{0x16};    // [1:0] full scale
constexpr uint8_t REG_GYRO_RANGE{0x2B};   // [2:0] full scale
constexpr uint8_t REG_CHIP_ID{0x30};      // = 0x18
// Undocumented registers: the datasheet register map ends at 0x30. The init sequence in begin()
// mirrors the original M5StickC library (SH200Q.cpp) verbatim -- no public documentation exists
// for these, and skipping them leaves the ADC in a bad state.
constexpr uint8_t REG_ADC_RESET{0xC2};  // bit2 toggled 1 -> 0
constexpr uint8_t REG_INIT_D8{0xD8};    // bit7 toggled 1 -> 0
constexpr uint8_t REG_INIT_78{0x78};    // written 0x61 -> 0x00
constexpr uint8_t REG_SET1{0xBA};       // written 0xC0
constexpr uint8_t REG_SET2{0xCA};       // bit4 toggled 1 -> 0 ("ADC reset" in the original library)
///@endcond
}  // namespace command
}  // namespace sh200q

}  // namespace unit
}  // namespace m5
#endif
