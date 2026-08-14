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
#include "unit_bmm150_data.hpp"
#include "../imu/imu_data_source.hpp"
#include "../imu/motion_events.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitBMI270
  @brief 6-axis (accel + gyro) IMU unit. CoreS3/AtomS3R built-in
  @note Accel/gyro/temp only. The chip's auxiliary (AUX) interface, which can drive an on-chip
  BMM150 magnetometer, is deliberately left untouched here -- use UnitBMI270AuxBMM150 on the
  boards that wire one to it (CoreS3, AtomS3R, Mini IMU-Pro). Enabling it is not always safe:
  on UnitDoF9/UnitDoF10 the AUX pins share the main I2C bus, and switching the interface on makes
  the BMI270 a second bus master, locking the bus until the unit is power-cycled
 */
class UnitBMI270 : public Component,
                   public PeriodicMeasurementAdapter<UnitBMI270, bmi270::Data>,
                   public m5::imu::IMUDataSource,
                   public m5::imu::MotionEventSource {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitBMI270, 0x69);

public:
    /*!
      @struct config_t
      @brief Settings for begin()
      @note Ring buffer depth is configured via component_config() (default:
      bmi270::DEFAULT_STORED_SIZE)
      @note These are the settings applied by begin(). They are not updated by the runtime setters
      (writeAccelRange() and friends), so config() keeps reporting what was configured -- read the
      device with readAccelRange() etc. to find out what is in effect now
     */
    struct config_t {
        //! Start periodic measurement on begin()?
        bool start_periodic{true};
        //! Accelerometer range
        bmi270::AccelRange accel_range{bmi270::AccelRange::G8};
        //! Gyroscope range
        bmi270::GyroRange gyro_range{bmi270::GyroRange::DPS2000};
        //! Accelerometer output data rate (the chip's power-on default; matches gyro_odr so the
        //! AHRS layer gets accel and gyro at the same cadence)
        bmi270::AccelODR accel_odr{bmi270::AccelODR::Rate200Hz};
        //! Gyroscope output data rate (periodic measurement interval is derived from this). 200Hz is
        //! the chip's power-on default and keeps hand-speed motion out of the under-sampled regime;
        //! lower it if power matters more than tracking fast rotation
        bmi270::GyroODR gyro_odr{bmi270::GyroODR::Rate200Hz};
        //! Accelerometer filter performance mode. PowerOptimized also switches accel_bwp from
        //! filtering to averaging, which is what makes its Avg16 and up usable
        bmi270::FilterPerf accel_filter_perf{bmi270::FilterPerf::Performance};
        //! Accelerometer bandwidth parameter (the chip's power-on default). Its 3dB cutoff scales
        //! with accel_odr -- 89Hz at 200Hz in the normal filter. startPeriodicMeasurement() rejects
        //! the averaging-only values while accel_filter_perf is Performance (bmi270::bwp_fits())
        bmi270::AccelBwp accel_bwp{bmi270::AccelBwp::NormAvg4};
        //! Gyroscope filter performance mode
        bmi270::FilterPerf gyro_filter_perf{bmi270::FilterPerf::Performance};
        //! Gyroscope bandwidth parameter (the chip's power-on default)
        bmi270::GyroBwp gyro_bwp{bmi270::GyroBwp::Norm};
    };

    //! @brief Construct with the I2C address the chip will answer on
    //! @param[in] addr I2C address (defaults to the chip's factory address)
    explicit UnitBMI270(const uint8_t addr = DEFAULT_ADDRESS);
    //! @brief Destructor
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

    ///@name Range setting (stop periodic measurement first)
    ///@{
    //! @brief Write the accelerometer range
    //! @param[in] range Accelerometer range
    //! @return True if successful
    //! @warning During periodic measurement runs, an error is returned
    bool writeAccelRange(const bmi270::AccelRange range);
    //! @brief Write the gyroscope range
    //! @param[in] range Gyroscope range
    //! @return True if successful
    //! @warning During periodic measurement runs, an error is returned
    bool writeGyroRange(const bmi270::GyroRange range);
    //! @brief Read the accelerometer range from the device (REG_ACC_RANGE bits[1:0])
    //! @param[out] range Accelerometer range
    //! @return True if successful
    bool readAccelRange(bmi270::AccelRange& range);
    //! @brief Read the gyroscope range from the device (REG_GYR_RANGE bits[2:0])
    //! @param[out] range Gyroscope range
    //! @return True if successful, false if the register holds a reserved (unmapped) value
    bool readGyroRange(bmi270::GyroRange& range);
    ///@}

    ///@name Output data rate (stop periodic measurement first)
    ///@{
    //! @brief Write the accelerometer output data rate
    //! @param[in] odr Accelerometer output data rate
    //! @return True if successful
    //! @note config_t::accel_filter_perf and config_t::accel_bwp share REG_ACC_CONF with the rate
    //! and are rewritten from the configuration along with it
    //! @warning During periodic measurement runs, an error is returned
    bool writeAccelODR(const bmi270::AccelODR odr);
    /*!
      @brief Write the gyroscope output data rate
      @param[in] odr Gyroscope output data rate
      @return True if successful
      @note The periodic measurement interval is derived from this rate, so it is updated too
      @warning During periodic measurement runs, an error is returned
     */
    bool writeGyroODR(const bmi270::GyroODR odr);
    //! @brief Read the accelerometer output data rate from the device (REG_ACC_CONF bits[3:0])
    //! @param[out] odr Accelerometer output data rate
    //! @return True if successful, false if the register holds a reserved (unmapped) value
    bool readAccelODR(bmi270::AccelODR& odr);
    //! @brief Read the gyroscope output data rate from the device (REG_GYR_CONF bits[3:0])
    //! @param[out] odr Gyroscope output data rate
    //! @return True if successful, false if the register holds a reserved (unmapped) value
    bool readGyroODR(bmi270::GyroODR& odr);
    ///@}

    ///@name Reset
    ///@{
    //! @brief Soft reset (REG_CMD CMD_SOFTRESET), then re-run the mandatory post-reset initialization
    //! (config blob re-upload), leaving the chip initialized but unconfigured and stopped
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
      @param accel_odr Accelerometer output data rate
      @param gyro_odr Gyroscope output data rate (periodic measurement interval is derived from this)
      @param accel_filter_perf Accelerometer filter performance mode
      @param accel_bwp Accelerometer bandwidth parameter
      @param gyro_filter_perf Gyroscope filter performance mode
      @param gyro_bwp Gyroscope bandwidth parameter
      @return True if successful, false if accel_bwp is reserved in accel_filter_perf (bmi270::bwp_fits())
      @note Every field of REG_ACC_CONF/REG_GYR_CONF is an argument so that starting never depends on
      config_t, which only begin() reads
      @note UnitBMI270AuxBMM150 overrides this to also (re)probe and configure its AUX BMM150
    */
    bool startPeriodicMeasurement(const bmi270::AccelRange accel_range, const bmi270::GyroRange gyro_range,
                                  const bmi270::AccelODR accel_odr, const bmi270::GyroODR gyro_odr,
                                  const bmi270::FilterPerf accel_filter_perf, const bmi270::AccelBwp accel_bwp,
                                  const bmi270::FilterPerf gyro_filter_perf, const bmi270::GyroBwp gyro_bwp);
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
      @brief Suspend the chip (REG_PWR_CTRL off, REG_PWR_CONF adv_power_save)
      @return True if successful
      @details Works while periodic measurement is running: it is stopped first, since a
      suspended chip must not be polled. Then accel/gyro/temp are powered down and advanced power
      save is enabled. The uploaded
      configuration file survives suspend -- unlike softReset(), which requires the full
      post-reset initialization again.
     */
    bool sleep() override;
    /*!
      @brief Resume the chip from sleep()
      @return True if successful
      @details Leaves advanced power save (configuration writes are ignored while it is active)
      and powers accel/gyro/temp back on -- measurement stays stopped. Call
      startPeriodicMeasurement() to resume sampling. Harmless without a preceding sleep().
     */
    bool wakeup() override;
    ///@}

    ///@name IMUDataSource
    ///@{
    //! @brief Capability of this unit
    //! @return Accel | Gyro | Temp. This class leaves the AUX interface alone, so never Mag --
    //! see UnitBMI270AuxBMM150 for the boards that wire a magnetometer to it
    virtual m5::imu::Capability imuSpec() const override
    {
        using m5::imu::Capability;
        return Capability::Accel | Capability::Gyro | Capability::Temp;
    }
    //! @brief Read the latest raw IMU data
    //! @param[out] out Raw IMU data
    //! @return Capability of the data actually stored in out (None if no data available)
    virtual m5::imu::Capability readImuRaw(m5::imu::imu_raw_data_t& out) override;
    //! @brief Gets the conversion parameters for raw IMU data
    //! @param[out] out Conversion parameters
    virtual void imuConvertParam(m5::imu::imu_convert_param_t& out) const override;
    ///@}

    ///@name Motion event (poll only)
    ///@{
    //! @brief Bitmap of motion events this chip provides
    virtual m5::imu::MotionEventKind motionEventSpec() const override
    {
        return m5::imu::MotionEventKind::AnyMotion | m5::imu::MotionEventKind::NoMotion |
               m5::imu::MotionEventKind::SignificantMotion | m5::imu::MotionEventKind::StepCounter;
    }
    //! @brief Currently enabled motion event bitmap
    virtual m5::imu::MotionEventKind enabledEvents() const override
    {
        return _enabled_mask;
    }
    //! @brief Enable a motion event kind. cfg must point to the config struct matching kind
    //! (AnyMotionConfig for AnyMotion, NoMotionConfig for NoMotion, SignificantMotionConfig for
    //! SignificantMotion)
    //! @return true on success, false when kind is unsupported or cfg is null
    virtual bool enableEvent(m5::imu::MotionEventKind kind, const void* cfg) override;
    //! @brief Disable a motion event kind and clear its status
    virtual bool disableEvent(m5::imu::MotionEventKind kind) override;
    //! @brief Read latched motion event status (clears internal latch on read)
    virtual bool readEventStatus(m5::imu::MotionEventStatus& out) override;
    /*!
      @brief BMI270 step counter watermark hardware resolution
      @return 20 (SC_26.watermark_level implicitly carries a 20x factor per datasheet section 4.8.8)
     */
    uint16_t stepCounterResolutionSteps() const override
    {
        return 20;
    }
    ///@}

    ///@name Step counter
    ///@{
    /*!
      @brief Reset the BMI270 step counter to zero
      @note Effective only when step_counter is enabled; safe to call from any state
      @return true on success
     */
    bool resetStepCount();
    ///@}

protected:
    virtual bool start_periodic_measurement(const bmi270::AccelRange accel_range, const bmi270::GyroRange gyro_range,
                                            const bmi270::AccelODR accel_odr, const bmi270::GyroODR gyro_odr,
                                            const bmi270::FilterPerf accel_filter_perf,
                                            const bmi270::AccelBwp accel_bwp, const bmi270::FilterPerf gyro_filter_perf,
                                            const bmi270::GyroBwp gyro_bwp);
    bool start_periodic_measurement();
    bool stop_periodic_measurement();
    bool read_measurement(bmi270::Data& d);
    // Read one sample's sensor registers into d. Split out of read_measurement() so a subclass
    // can widen the burst without repeating the temperature handling that follows it (see
    // UnitBMI270AuxBMM150).
    virtual bool read_sensor_burst(bmi270::Data& d);
    bool write_register8_verified(const uint8_t reg, const uint8_t val);
    // readRegister8() without its "Failed to write" log, for the reads whose failure is expected:
    // the dummy transaction that brings the I2C interface up after power-on, the two polls that wait
    // out the ~2ms reboot after a soft reset, and the verified-write readback that is retried
    bool read_register8_quiet(const uint8_t reg, uint8_t& out);
    // Last resort for a CHIP_ID that came back wrong: soft reset the device and read it again.
    // Call only once the address has acknowledged, and only with the value it answered: the reset
    // is skipped unless that value is one the real CHIP_ID could have decayed into on a contended
    // bus (see the .cpp), so a different chip living at this address is never written to. Reaches
    // only the form where the chip still answers: a bus being held low takes no writes at all
    bool recover_wedged_interface(const uint8_t id);
    // Uploads the 8192 byte configuration blob (unit_bmi270_config.inl) to REG_INIT_DATA,
    // split into chunks that fit within the I2C write buffer (see .cpp for chunk size
    // rationale). Returns false (and logs) on the first failed chunk, unlike M5Unified's
    // BMI270_Class::_upload_file() which always returns true regardless of write result.
    bool upload_config();

    // Programs the any_motion feature engine registers (ANYMO_1/ANYMO_2) from cfg and maps its
    // output onto INT1 so INT_STATUS_0 latches it. See unit_bmi270.cpp for the register layout.
    bool write_any_motion_features(const m5::imu::AnyMotionConfig& c);
    // Clears the any_motion feature's enable bit and its INT_MAP_FEAT routing, without disturbing
    // the duration/threshold/axis fields written by write_any_motion_features()
    bool clear_any_motion_features();
    // Programs the no_motion feature engine registers (NOMO_1/NOMO_2) from cfg and maps its output
    // onto INT1 so INT_STATUS_0 latches it. See unit_bmi270.cpp for the register layout.
    bool write_no_motion_features(const m5::imu::NoMotionConfig& cfg);
    // Clears the no_motion feature's enable bit and its INT_MAP_FEAT routing, without disturbing
    // the duration/threshold/axis fields written by write_no_motion_features()
    bool clear_no_motion_features();
    // Programs the significant_motion feature engine registers (SIGMO_1/SIGMO_2) from cfg and maps
    // its output onto INT1 so INT_STATUS_0 latches it. See unit_bmi270.cpp for the register layout.
    bool write_sig_motion_features(const m5::imu::SignificantMotionConfig& cfg);
    // Clears the significant_motion feature's enable bit and its INT_MAP_FEAT routing, without
    // disturbing the block_size field written by write_sig_motion_features()
    bool clear_sig_motion_features();
    // Programs SC_26 (page FEAT_PAGE_STEP_COUNTER) from cfg: selects en_counter or en_detector per
    // cfg.mode (mutually exclusive), sets watermark_level for Counter mode, and maps step_counter_out
    // onto INT1 so INT_STATUS_0 latches it. See unit_bmi270.cpp for the register layout.
    bool write_step_counter_features(const m5::imu::StepCounterConfig& cfg);
    // Clears both en_counter and en_detector and the INT_MAP_FEAT routing, without disturbing
    // watermark_level/en_activity/reset_counter written elsewhere
    bool clear_step_counter_features();
    // Reads the 16-bit step count (REG_STEP_COUNT_0/1, little-endian)
    bool read_step_count(uint32_t& out);
    // Reads len bytes starting at ofs_bytes (an address within the currently FEAT_PAGE-selected
    // FEATURES[16] window, REG_FEATURES..REG_FEATURES+0x0F)
    bool read_feature_bytes(const uint16_t ofs_bytes, uint8_t* buf, const size_t len);
    // Writes len bytes starting at ofs_bytes (same window as read_feature_bytes())
    bool write_feature_bytes(const uint16_t ofs_bytes, const uint8_t* buf, const size_t len);
    // Current accelerometer ODR in Hz (_accel_odr decoded), used to convert AnyMotionConfig's
    // duration_ms into the feature engine's sample-count field
    uint16_t accelOdrHz() const;

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitBMI270, bmi270::Data);

private:
    // Settings for begin(); never written to after that. What is currently on the device lives in
    // the live-state members below, so config() keeps meaning "what was configured"
    config_t _cfg{};
    // Mirrors of the registers this driver can change at runtime. imuConvertParam() needs the range
    // actually in effect (a stale factor would silently misreport every sample), and the no-argument
    // startPeriodicMeasurement() resumes from these rather than reverting to _cfg
    bmi270::AccelRange _accel_range{};
    bmi270::GyroRange _gyro_range{};
    bmi270::AccelODR _accel_odr{};
    bmi270::GyroODR _gyro_odr{};
    // Bandwidth and filter performance have no runtime setter of their own, but they share
    // REG_ACC_CONF/REG_GYR_CONF with the rate: writeAccelODR() has to recompose the whole register,
    // and taking these from _cfg there would silently apply a config() change the caller only meant
    // to take effect at the next start
    bmi270::FilterPerf _accel_filter_perf{};
    bmi270::AccelBwp _accel_bwp{};
    bmi270::FilterPerf _gyro_filter_perf{};
    bmi270::GyroBwp _gyro_bwp{};
    int16_t _last_temp{};             // Latest valid REG_TEMPERATURE_0 reading (register update is not per-sample)
    uint32_t _pushed{}, _consumed{};  // readImuRaw pop counters (do not disturb latest()/oldest())
    std::unique_ptr<m5::container::CircularBuffer<bmi270::Data>> _data{};
    // Motion event bookkeeping: enable/disable state, the config last handed to enableEvent(), and
    // the locally latched status flag for INT_STATUS_0's read-to-clear any_motion_out bit.
    m5::imu::MotionEventKind _enabled_mask{m5::imu::MotionEventKind::None};
    m5::imu::AnyMotionConfig _any_motion_cfg{};
    bool _any_motion_pending{false};
    m5::imu::NoMotionConfig _no_motion_cfg{};
    bool _no_motion_pending{false};
    m5::imu::SignificantMotionConfig _sig_motion_cfg{};
    bool _sig_motion_pending{false};
    m5::imu::StepCounterConfig _step_counter_cfg{};
    uint32_t _step_count_latest{0};
    bool _step_counter_pending{false};
};

namespace bmi270 {
//! @brief Default ring buffer depth (used to construct the internal buffer and as the
//! component_config().stored_size default; see config_t)
constexpr uint32_t DEFAULT_STORED_SIZE{8};

namespace command {
///@cond
// Register addresses (I2C addr 0x68 (SDO=GND) / 0x69 (SDO=VDDIO, M5 board default))
constexpr uint8_t REG_DATA_0{0x04};         // AUX (mag) burst start: 20 bytes = mag(8)+accel(6)+gyro(6), see REG_DATA_8
constexpr uint8_t REG_STATUS{0x03};         // bit2 = aux_man_op busy (named REG_STATUS, not STATUS: ESP-IDF's
                                            // ets_sys.h has a global `typedef enum STATUS STATUS`)
constexpr uint8_t REG_DATA_8{0x0C};         // 12 bytes: accel X/Y/Z, gyro X/Y/Z (little-endian)
constexpr uint8_t REG_INT_STATUS_0{0x1C};   // bit6 any_motion_out (read-clear); see the feature engine block below
constexpr uint8_t REG_INT_STATUS_1{0x1D};   // bit7 acc_drdy, bit6 gyr_drdy, bit5 aux_drdy
constexpr uint8_t REG_TEMPERATURE_0{0x22};  // 2 bytes, little-endian, 0x8000 = invalid
constexpr uint8_t REG_INTERNAL_STATUS{0x21};
constexpr uint8_t REG_ACC_CONF{0x40};
constexpr uint8_t REG_ACC_RANGE{0x41};
constexpr uint8_t REG_GYR_CONF{0x42};
constexpr uint8_t REG_GYR_RANGE{0x43};
constexpr uint8_t REG_AUX_DEV_ID{0x4B};
// Poll rate and trigger-readout offset for the AUX target (see UnitBMI270AuxBMM150)
constexpr uint8_t REG_AUX_CONF{0x44};
constexpr uint8_t REG_AUX_IF_CONF{
    0x4C};  // bit7 manual_en, bit6 fcu_write_en, bits[3:2] man_rd_burst, bits[1:0] aux_rd_burst
constexpr uint8_t REG_AUX_RD_ADDR{0x4D};  // write triggers an AUX read transaction
constexpr uint8_t REG_AUX_WR_ADDR{0x4E};  // write triggers an AUX write transaction (of REG_AUX_WR_DATA)
constexpr uint8_t REG_AUX_WR_DATA{0x4F};
// FEATURES bank (datasheet section 4.8, register table 5.2.39/5.2.40): a 16 byte window at
// REG_FEATURES..REG_FEATURES+0x0F whose content is selected by REG_FEAT_PAGE. Unlike
// REG_INIT_ADDR_0/REG_INIT_DATA above (write-only, used once for the config blob upload), this
// window is directly byte-addressable for runtime read/modify/write of a feature's configuration.
// any_motion (page 1) occupies the last 4 bytes of the window: ANYMO_1 (duration + per-axis
// select) at REG_FEATURES_ANY_MOTION, ANYMO_2 (threshold + out_conf + enable) at
// REG_FEATURES_ANY_MOTION_2. Only any_motion is programmed by this driver; other pages/features
// are out of scope.
constexpr uint8_t REG_FEAT_PAGE{0x2F};              // FEATURES window page select, bits[2:0]
constexpr uint8_t REG_FEATURES{0x30};               // FEATURES[16] window base (0x30-0x3F)
constexpr uint8_t FEAT_PAGE_ANY_MOTION{0x01};       // Page holding ANYMO_1/ANYMO_2
constexpr uint8_t REG_FEATURES_ANY_MOTION{0x3C};    // ANYMO_1, 2 bytes, followed by ANYMO_2
constexpr uint8_t REG_FEATURES_ANY_MOTION_2{0x3E};  // ANYMO_2 alone (threshold/out_conf/enable)
// no_motion (NOMO_1/NOMO_2) and significant_motion (SIGMO_1/SIGMO_2) share the page selected by
// FEAT_PAGE_ANY_MOTION+1 (page 2); step_counter's tunable parameters (SC_1..SC_8) start a
// further page over (page 3). Values are absolute FEATURES-window register addresses, matching
// REG_FEATURES_ANY_MOTION above, for direct use with read_feature_bytes()/write_feature_bytes().
constexpr uint8_t FEAT_PAGE_NO_MOTION{0x02};                     // Page holding NOMO_1/NOMO_2
constexpr uint16_t BMI270_FEATURE_NO_MOTION_OFS_WORDS{0x30};     // NOMO_1, 2 bytes, followed by NOMO_2
constexpr uint16_t BMI270_FEATURE_NO_MOTION_2_OFS_WORDS{0x32};   // NOMO_2 alone (threshold/out_conf/enable)
constexpr uint16_t BMI270_FEATURE_SIG_MOTION_OFS_WORDS{0x34};    // SIGMO_1 (block_size, full word)
// SIGMO_2 sits 5 words after SIGMO_1, not immediately after it like NOMO_1/NOMO_2: 0x36/0x38/0x3A/
// 0x3C in between are reserved words (datasheet section 4.8.4 register table, page 2)
constexpr uint16_t BMI270_FEATURE_SIG_MOTION_2_OFS_WORDS{0x3E};  // SIGMO_2 (enable + out_conf)
constexpr uint16_t BMI270_FEATURE_STEP_COUNT_OFS_WORDS{0x30};    // SC_1
// step_counter's enable/mode/watermark control (SC_26) lives on a further page over from its own
// tuning parameters above: FEAT_PAGE 6, not the SC_1..SC_25 page (3). Value verified against
// datasheet section 4.8.8 / the FEATURES Pages register table (page 6, address 0x32) and
// cross-checked against Bosch's BMI270_SensorAPI bmi2_defs.h (BMI2_STEP_COUNT_WM_LEVEL_MASK,
// BMI2_STEP_COUNT_RST_CNT_MASK/_POS, BMI2_STEP_DET_FEAT_EN_POS, BMI2_STEP_COUNT_FEAT_EN_POS,
// BMI2_STEP_ACT_FEAT_EN_POS -- the latter three are documented relative to the register's high
// byte, i.e. +8 to get the full-word bit position). SC_1..SC_25 (FEAT_PAGE 3) are tuning
// parameters left at their config-blob defaults; this driver only ever touches SC_26.
constexpr uint8_t FEAT_PAGE_STEP_COUNTER{0x06};
constexpr uint16_t BMI270_FEATURE_STEP_COUNT_SC26_OFS{0x32};
constexpr uint8_t INT_STATUS_0_ANY_MOTION_OUT_BIT{1 << 6};
// INT_STATUS_0 also reports these feature engine outputs (datasheet section 5.2.28)
constexpr uint8_t INT_STATUS_0_NO_MOTION_OUT_BIT{1 << 5};
constexpr uint8_t INT_STATUS_0_STEP_COUNTER_OUT_BIT{1 << 1};
constexpr uint8_t INT_STATUS_0_SIG_MOTION_OUT_BIT{1 << 0};
// Step counter value (datasheet section 5.2.30/5.2.31): 2 bytes little-endian, 16-bit count
constexpr uint8_t REG_STEP_COUNT_0{0x1E};
constexpr uint8_t REG_INT_MAP_FEAT{0x56};                   // INT1_MAP_FEAT
constexpr uint8_t INT_MAP_FEAT_ANY_MOTION_OUT_BIT{1 << 6};  // any_motion_out -> INT1
// Additional INT1_MAP_FEAT bits (datasheet section 5.2.61)
constexpr uint8_t INT_MAP_FEAT_NO_MOTION_OUT_BIT{1 << 5};
constexpr uint8_t INT_MAP_FEAT_STEP_COUNTER_OUT_BIT{1 << 1};
constexpr uint8_t INT_MAP_FEAT_SIG_MOTION_OUT_BIT{1 << 0};

constexpr uint8_t REG_INIT_CTRL{0x59};
constexpr uint8_t REG_INIT_ADDR_0{0x5B};  // followed by INIT_ADDR_1 (0x5C)
constexpr uint8_t REG_INIT_DATA{0x5E};
constexpr uint8_t REG_IF_CONF{0x6B};   // bit5 aux_en
constexpr uint8_t REG_PWR_CONF{0x7C};  // bit0 adv_power_save
constexpr uint8_t REG_PWR_CTRL{0x7D};  // bit0 aux_en, bit1 gyr_en, bit2 acc_en, bit3 temp_en
constexpr uint8_t REG_CMD{0x7E};
constexpr uint8_t REG_CHIP_ID{0x00};

// BMM150 registers, reachable only indirectly through the AUX bridge above (distinct address
// space from this chip's own registers). Only UnitBMI270AuxBMM150 talks to them; they live here
// because they are addressed through this chip's AUX registers
constexpr uint8_t REG_BMM150_CHIP_ID{0x40};
constexpr uint8_t REG_BMM150_DATA_X_LSB{0x42};
constexpr uint8_t REG_BMM150_POWER_CONTROL{0x4B};
constexpr uint8_t REG_BMM150_OP_MODE{0x4C};
constexpr uint8_t REG_BMM150_REP_XY{0x51};
constexpr uint8_t REG_BMM150_REP_Z{0x52};
///@endcond
}  // namespace command

//! @brief BMI270 chip identifier (value read from command::REG_CHIP_ID)
constexpr uint8_t CHIP_ID{0x24};

//! @brief Detect which I2C address a built-in BMI270 uses on a bus (0x68 or 0x69, board-dependent)
//! @details Probes both candidate addresses by reading CHIP_ID and reports the one where a BMI270
//! answers. Framework-agnostic: the caller injects the bus read, so the unit library stays free of
//! M5Unified / Arduino Wire. Hand the detected address to the constructor:
//! @code
//! uint8_t a{};
//! if (m5::unit::bmi270::detectAddress(read, a)) { m5::unit::UnitBMI270 unit{a}; }
//! @endcode
//! @tparam ReadReg Callable `bool(uint8_t addr, uint8_t reg, uint8_t& out)` reading one register
//!                 over the target bus and returning true on success (ACK)
//! @param  read     The read callable
//! @param[out] addr Detected address on success (unchanged on failure)
//! @return true if a BMI270 responded at 0x68 or 0x69
template <class ReadReg>
bool detectAddress(ReadReg read, uint8_t& addr)
{
    for (const uint8_t candidate : {static_cast<uint8_t>(0x68), static_cast<uint8_t>(0x69)}) {
        uint8_t id{};
        if (read(candidate, command::REG_CHIP_ID, id) && id == CHIP_ID) {
            addr = candidate;
            return true;
        }
    }
    return false;
}
}  // namespace bmi270

}  // namespace unit
}  // namespace m5
#endif
