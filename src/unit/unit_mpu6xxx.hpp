/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu6xxx.hpp
  @brief UnitMPU6xxx: shared base for the InvenSense MPU 6-axis family
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_MPU6XXX_HPP
#define M5_UNIT_IMU_UNIT_UNIT_MPU6XXX_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <memory>
#include "unit_mpu6886_data.hpp"
#include "../imu/imu_data_source.hpp"
#include "../imu/motion_events.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitMPU6xxx
  @brief Shared base for the TDK InvenSense MPU 6-axis family (accel + gyro + temp)
  @details The accel/gyro register map, init sequence, and periodic read are identical across
  MPU6886/6050/6500 (and the MPU9250 accel/gyro die), so they all live here. The only per-chip
  differences -- the accepted WHO_AM_I id and the temperature scale -- are exposed as virtual hooks
  (accepts_whoami / temperature_res / temperature_offset) that thin derived classes override. Not
  instantiated directly; use UnitMPU6886 / UnitMPU6050 / UnitMPU6500 / UnitMPU9250.
 */
class UnitMPU6xxx : public Component,
                    public PeriodicMeasurementAdapter<UnitMPU6xxx, mpu6886::Data>,
                    public m5::imu::IMUDataSource,
                    public m5::imu::MotionEventSource {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMPU6xxx, 0x68);

public:
    /*!
      @struct config_t
      @brief Settings for begin()
      @note Ring buffer depth is configured via component_config() (default:
      mpu6886::DEFAULT_STORED_SIZE)
      @note These are the settings applied by begin(). They are not updated by the runtime setters
      (writeAccelRange() and friends), so config() keeps reporting what was configured -- read the
      device with readAccelRange() etc. to find out what is in effect now
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
    };

    virtual ~UnitMPU6xxx()
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
    bool writeAccelRange(const mpu6886::AccelRange range);
    //! @brief Write the gyroscope range
    //! @param[in] range Gyroscope range
    //! @return True if successful
    //! @warning During periodic measurement runs, an error is returned
    bool writeGyroRange(const mpu6886::GyroRange range);
    //! @brief Read the accelerometer range from the device (REG_ACCEL_CONFIG bits[4:3])
    //! @param[out] range Accelerometer range
    //! @return True if successful
    bool readAccelRange(mpu6886::AccelRange& range);
    //! @brief Read the gyroscope range from the device (REG_GYRO_CONFIG bits[4:3])
    //! @param[out] range Gyroscope range
    //! @return True if successful
    bool readGyroRange(mpu6886::GyroRange& range);
    ///@}

    ///@name Sampling (stop periodic measurement first)
    ///@{
    /*!
      @brief Write the sample rate divider
      @param[in] div Divider; the sample rate becomes 1kHz / (1 + div) with the DLPF enabled
      @return True if successful
      @note The periodic measurement interval is derived from this, so it is updated too
      @warning During periodic measurement runs, an error is returned
     */
    bool writeSampleRateDiv(const uint8_t div);
    //! @brief Read the sample rate divider from the device (REG_SMPLRT_DIV)
    //! @param[out] div Divider
    //! @return True if successful
    bool readSampleRateDiv(uint8_t& div);
    //! @brief Write the digital low-pass filter bandwidth
    //! @param[in] dlpf Filter bandwidth
    //! @return True if successful
    //! @warning During periodic measurement runs, an error is returned
    bool writeDlpf(const mpu6886::Dlpf dlpf);
    //! @brief Read the digital low-pass filter bandwidth from the device (REG_CONFIG bits[2:0])
    //! @param[out] dlpf Filter bandwidth
    //! @return True if successful
    bool readDlpf(mpu6886::Dlpf& dlpf);
    ///@}

    ///@name Reset
    ///@{
    //! @brief Soft reset (REG_PWR_MGMT_1 DEVICE_RESET), leaving the chip initialized but unconfigured and stopped
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
      @param dlpf Gyro/temp low pass filter
      @param sample_rate_div Sample rate divider (ODR = 1000/(1+div) Hz)
      @return True if successful
    */
    bool startPeriodicMeasurement(const mpu6886::AccelRange accel_range, const mpu6886::GyroRange gyro_range,
                                  const mpu6886::Dlpf dlpf, const uint8_t sample_rate_div);
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
      @brief Put the chip to sleep (REG_PWR_MGMT_1 SLEEP)
      @return True if successful
      @details Works while periodic measurement is running: it is stopped first, since a
      sleeping chip must not be polled. The SLEEP bit then halts sampling and drops the chip to
      its low-power state. Unlike softReset() the configuration is preserved.
     */
    bool sleep() override;
    /*!
      @brief Wake the chip from sleep()
      @return True if successful
      @details Only clears SLEEP (by selecting the PLL clock) and waits for the sensors to
      start up -- measurement stays stopped. Call startPeriodicMeasurement() to resume
      sampling. Harmless without a preceding sleep().
     */
    bool wakeup() override;
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

    ///@name MotionEventSource
    ///@{
    //! @brief Motion event capability. Default: none. Chip-specific derived classes override.
    //! @note The WoM register layout differs across MPU-6886 / MPU-9250 / MPU-6050, so the base
    //! deliberately reports no capability; only UnitMPU6886 currently overrides with real support.
    virtual m5::imu::MotionEventKind motionEventSpec() const override
    {
        return m5::imu::MotionEventKind::None;
    }
    //! @brief Currently enabled event mask. Default: none.
    virtual m5::imu::MotionEventKind enabledEvents() const override
    {
        return m5::imu::MotionEventKind::None;
    }
    //! @brief Enable a motion event. Base returns false; MPU-6886 override enables AnyMotion (WoM).
    virtual bool enableEvent(m5::imu::MotionEventKind, const void*) override
    {
        return false;
    }
    //! @brief Disable a motion event. Base returns false; MPU-6886 override restores pre-WoM state.
    virtual bool disableEvent(m5::imu::MotionEventKind) override
    {
        return false;
    }
    //! @brief Poll and clear pending event status. Base returns false; MPU-6886 override latches INT_STATUS.
    virtual bool readEventStatus(m5::imu::MotionEventStatus&) override
    {
        return false;
    }
    ///@}

protected:
    //! @brief Protected: UnitMPU6xxx is abstract, constructed only by concrete derived chips
    explicit UnitMPU6xxx(const uint8_t addr = DEFAULT_ADDRESS);

    ///@name Chip profile (overridden by each concrete derived chip)
    ///@{
    //! @brief Accepted WHO_AM_I id(s) for this chip; begin() rejects the device unless this returns true
    virtual bool accepts_whoami(const uint8_t id) const = 0;
    //! @brief Temperature conversion: degC per LSB (imu_convert_param_t::temp_res)
    virtual float temperature_res() const = 0;
    //! @brief Temperature conversion: degC offset (imu_convert_param_t::temp_offset)
    virtual float temperature_offset() const = 0;
    ///@}

    //! @brief Virtual: UnitMPU9250 layers its aux-bypass re-open on top (the config write here
    //! includes INT_PIN_CFG=0xC0, which closes the bypass)
    virtual bool start_periodic_measurement(const mpu6886::AccelRange accel_range, const mpu6886::GyroRange gyro_range,
                                            const mpu6886::Dlpf dlpf, const uint8_t sample_rate_div);
    bool start_periodic_measurement();
    bool stop_periodic_measurement();
    bool read_measurement(mpu6886::Data& d);
    bool write_register8_verified(const uint8_t reg, const uint8_t val);

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitMPU6xxx, mpu6886::Data);

private:
    // Settings for begin(); never written to after that. What is currently on the device lives in
    // the live-state members below, so config() keeps meaning "what was configured"
    config_t _cfg{};
    // Mirrors of the registers this driver can change at runtime. imuConvertParam() needs the range
    // actually in effect (a stale factor would silently misreport every sample), and the no-argument
    // startPeriodicMeasurement() resumes from these rather than reverting to _cfg
    mpu6886::AccelRange _accel_range{};
    mpu6886::GyroRange _gyro_range{};
    mpu6886::Dlpf _dlpf{};
    uint8_t _sample_rate_div{};
    uint32_t _pushed{}, _consumed{};  // readImuRaw pop counters (do not disturb latest()/oldest())
    std::unique_ptr<m5::container::CircularBuffer<mpu6886::Data>> _data{};
};

namespace mpu6886 {
//! @brief Default ring buffer depth (used to construct the internal buffer and as the
//! component_config().stored_size default; see config_t)
constexpr uint32_t DEFAULT_STORED_SIZE{8};

namespace command {
///@cond
// Register addresses (I2C addr 0x68/0x69 by AD0 pin)
constexpr uint8_t REG_GYRO_OFFS_USR{0x13};  // 6 bytes
constexpr uint8_t REG_SMPLRT_DIV{0x19};
constexpr uint8_t REG_CONFIG{0x1A};
constexpr uint8_t REG_GYRO_CONFIG{0x1B};
constexpr uint8_t REG_ACCEL_CONFIG{0x1C};
constexpr uint8_t REG_ACCEL_CONFIG2{0x1D};
constexpr uint8_t REG_FIFO_EN{0x23};
constexpr uint8_t REG_INT_PIN_CFG{0x37};
constexpr uint8_t REG_INT_ENABLE{0x38};
constexpr uint8_t REG_INT_STATUS{0x3A};    // bit0 = DATA_RDY (read clear)
constexpr uint8_t REG_ACCEL_XOUT_H{0x3B};  // 14 bytes: accel6 + temp2 + gyro6 (big-endian)
constexpr uint8_t REG_USER_CTRL{0x6A};
constexpr uint8_t REG_PWR_MGMT_1{0x6B};  // bit7 RESET / bit6 SLEEP / CLKSEL[2:0]
constexpr uint8_t REG_PWR_MGMT_2{0x6C};
constexpr uint8_t REG_WHO_AM_I{0x75};
///@endcond
}  // namespace command
}  // namespace mpu6886

}  // namespace unit
}  // namespace m5
#endif
