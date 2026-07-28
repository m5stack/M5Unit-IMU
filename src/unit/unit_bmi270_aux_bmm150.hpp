/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_bmi270_aux_bmm150.hpp
  @brief UnitBMI270AuxBMM150 (BMI270 driving a magnetometer over its auxiliary interface)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_BMI270_AUX_BMM150_HPP
#define M5_UNIT_IMU_UNIT_UNIT_BMI270_AUX_BMM150_HPP

#include "unit_bmi270.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitBMI270AuxBMM150
  @brief BMI270 that switches on its auxiliary (AUX) interface and drives a BMM150 through it

  @details The BMI270's AUX interface is a general-purpose I2C master built into the chip -- the
  datasheet calls it a "master interface hub for 1 I2C AUX sensor (e.g. ext. magnetometer,
  pressure)". This class implements one specific target, a Bosch BMM150: it addresses 0x10, checks
  for chip ID 0x32, reads that part's factory trim and drives its repetition/opmode registers. The
  name records that choice, since a different AUX target would need a different driver.

  Which class to instantiate is a property of the board, not of the chip:

  - This class turns the interface on and probes for the BMM150. **CoreS3 / AtomS3R / Mini IMU-Pro
    (U171)** wire one there. Built-in IMUs in general should use it -- the probe adapts to whatever
    the host actually carries, with no board table to keep up to date.
  - The plain UnitBMI270 never touches the interface. Use it where the wiring makes enabling it
    unsafe or pointless: on **UnitDoF9 / UnitDoF10** the AUX pins are tied to the main I2C bus, so
    enabling them turns the BMI270 into a second bus master and the bus locks up until the unit is
    power-cycled; on **UnitDoF6** there is simply no magnetometer.

  Naming a chip here does not promise one is present -- that is only known at runtime. On a host
  without a BMM150 (Capsule, StickS3, Core2 v1.3, ...) the probe finds nothing, the interface is
  switched back off, and the unit runs as a plain 6-axis BMI270. imuSpec() reports Mag only once a
  BMM150 has actually answered.

  @note The BMM350 in UnitDoF9 / UnitDoF10 / Unit BMM350 (U231) sits on the main bus, not on this
  interface, and is driven by UnitBMM350 instead
  @note The bridge polls the BMM150 at a fixed 30Hz, which bounds its usable oversampling; see
  config_t::aux_mag_preset
 */
class UnitBMI270AuxBMM150 : public UnitBMI270 {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitBMI270AuxBMM150, 0x69);

public:
    /*!
      @struct config_t
      @brief Settings for begin(), extending the base with the magnetometer's own
     */
    struct config_t : public UnitBMI270::config_t {
        /*!
          Accuracy/power preset written to the BMM150's REG_REP_XY/REG_REP_Z. HighAccuracy cannot be
          used here: its measurement takes 49.3ms, longer than the 33.3ms period of the bridge's
          fixed 30Hz poll, so startPeriodicMeasurement() rejects it instead of letting the data
          registers go stale
         */
        bmm150::Preset aux_mag_preset{bmm150::Preset::Regular};
    };

    explicit UnitBMI270AuxBMM150(const uint8_t addr = DEFAULT_ADDRESS);
    virtual ~UnitBMI270AuxBMM150()
    {
    }

    ///@name Settings
    ///@{
    //! @brief Gets the configuration
    //! @return Current configuration
    config_t config() const
    {
        // Only aux_mag_preset is kept here; the inherited fields are re-read from the base, which
        // updates its own copy from writeAccelRange()/writeGyroRange()/start_periodic_measurement()
        config_t cfg                            = _cfg_aux_bmm150;
        static_cast<UnitBMI270::config_t&>(cfg) = UnitBMI270::config();
        return cfg;
    }
    //! @brief Set the configuration
    //! @param[in] cfg Configuration
    //! @note The inherited fields stay in the base's own storage, so reading them back through a
    //! UnitBMI270 reference always agrees with what was set here
    void config(const config_t& cfg)
    {
        _cfg_aux_bmm150 = cfg;
        UnitBMI270::config(cfg);
    }
    ///@}

    ///@name IMUDataSource
    ///@{
    //! @brief Capability of this unit
    //! @return Accel | Gyro | Temp, plus Mag once an AUX BMM150 has been detected in
    //! startPeriodicMeasurement()
    virtual m5::imu::Capability imuSpec() const override
    {
        using m5::imu::Capability;
        const auto base = Capability::Accel | Capability::Gyro | Capability::Temp;
        return _aux_mag ? (base | Capability::Mag) : base;
    }
    //! @brief Gets the conversion parameters for raw IMU data
    //! @param[out] out Conversion parameters
    virtual void imuConvertParam(m5::imu::imu_convert_param_t& out) const override;
    ///@}

    /*!
      @brief Suspend the chip, parking the AUX BMM150 first
      @return True if successful
      @details The magnetometer keeps running on its own once the bridge is gone, so it is put into
      its own suspend mode before the BMI270 powers down. Otherwise identical to UnitBMI270::sleep()
     */
    bool sleep() override;

protected:
    //! @brief Base configuration, then probe and configure the AUX BMM150
    //! @details A probe failure is not an error: it just means this board has no magnetometer on the
    //! interface, so it is switched back off and measurement continues with 6 axes. An
    //! aux_mag_preset the fixed poll rate cannot keep up with *is* an error, and is rejected before
    //! any register is touched
    virtual bool start_periodic_measurement(const bmi270::AccelRange accel_range, const bmi270::GyroRange gyro_range,
                                            const bmi270::AccelODR accel_odr, const bmi270::GyroODR gyro_odr,
                                            const bmi270::FilterPerf accel_filter_perf,
                                            const bmi270::AccelBwp accel_bwp,
                                            const bmi270::FilterPerf gyro_filter_perf,
                                            const bmi270::GyroBwp gyro_bwp) override;
    //! @brief One 20 byte burst covering mag + accel + gyro, instead of the base's 12 byte accel/gyro read
    virtual bool read_sensor_burst(bmi270::Data& d) override;

    // AUX support. On success start_periodic_measurement() leaves the interface in data mode (the
    // BMI270 auto-polls the BMM150 at 30Hz into REG_DATA_0..REG_DATA_7) and _mag_trim holds the
    // factory trim read through the bridge.
    bool probe_aux_bmm150();
    // Polls REG_STATUS.aux_busy (bit2) until clear; gates every AUX manual transaction below
    bool aux_wait_ready();
    bool aux_write_register8(const uint8_t reg, const uint8_t val);
    bool aux_read_register8(const uint8_t reg, uint8_t& out);
    // Manual burst read of 8 bytes starting at a BMM150 register, via the AUX bridge
    bool aux_read_burst8(const uint8_t bmm150_reg, std::array<uint8_t, 8>& out);
    // Reads the BMM150 factory trim (registers 0x5D-0x71) through 3 overlapping 8 byte AUX
    // bursts (0x5D/0x65/0x6A) and maps them the same way as UnitBMM150::read_trim()
    bool aux_read_trim(bmm150::trim_t& trim);

private:
    // Only the aux_mag_preset field is authoritative here -- config() refreshes the inherited ones
    // from the base rather than reading them back out of this copy
    config_t _cfg_aux_bmm150{};
    bool _aux_mag{false};        // True once an AUX BMM150 was detected in start_periodic_measurement()
    bmm150::trim_t _mag_trim{};  // Factory trim read via the AUX bridge, valid only if _aux_mag
};

}  // namespace unit
}  // namespace m5
#endif
