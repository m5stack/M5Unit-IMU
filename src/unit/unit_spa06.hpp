/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_spa06.hpp
  @brief UnitSPA06 (SPA06-003 barometric pressure + temperature)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_SPA06_HPP
#define M5_UNIT_IMU_UNIT_UNIT_SPA06_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <memory>
#include "unit_spa06_data.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitSPA06
  @brief Goertek SPA06-003 barometer (pressure + temperature). Standalone: not fused into the AHRS;
  read pressure()/temperature()/altitude() directly. On Unit DoF10 it sits at 0x76.
 */
class UnitSPA06 : public Component, public PeriodicMeasurementAdapter<UnitSPA06, spa06::Data> {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitSPA06, 0x76);

public:
    /*!
      @struct config_t
      @brief Settings for begin()
     */
    struct config_t {
        //! Start periodic measurement on begin()?
        bool start_periodic{true};
        //! Pressure oversampling (1/2/4/8/16/32/64/128); 16 = Standard
        uint8_t pressure_oversampling{16};
        //! Temperature oversampling (1/2/4/8/16/32/64/128)
        uint8_t temperature_oversampling{1};
        //! Pressure/temperature measurement rate code PM_RATE/TMP_RATE bits (0=1/s..7=128/s)
        uint8_t rate{4};  // 16 samples/s
    };

    explicit UnitSPA06(const uint8_t addr = DEFAULT_ADDRESS);
    virtual ~UnitSPA06()
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

    ///@name Periodic measurement
    ///@{
    //! @brief Start periodic measurement in the current settings
    bool startPeriodicMeasurement();
    //! @brief Stop periodic measurement
    bool stopPeriodicMeasurement();
    ///@}

    ///@name Measurement
    ///@{
    //! @brief Latest compensated pressure (hPa)
    float pressure() const;
    //! @brief Latest compensated temperature (degC)
    float temperature() const;
    //! @brief Absolute altitude (m) versus the given sea-level pressure (QNH, hPa)
    float altitude(const float sea_level_hpa = 1013.25f) const;
    //! @brief Altitude (m) relative to the last setReference() tare point (0 m at that pressure)
    float relativeAltitude() const;
    //! @brief Tare the current pressure as the 0 m reference for relativeAltitude()
    void setReference();
    ///@}

protected:
    bool start_periodic_measurement();
    bool stop_periodic_measurement();
    bool read_measurement(spa06::Data& d);
    bool read_coefficients();

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitSPA06, spa06::Data);

private:
    config_t _cfg{};
    spa06::coeffs_t _coeffs{};
    float _kp{253952.0f};
    float _kt{253952.0f};
    float _reference_hpa{1013.25f};
    uint32_t _pushed{}, _consumed{};
    std::unique_ptr<m5::container::CircularBuffer<spa06::Data>> _data{};
};

namespace spa06 {
//! @brief Default ring buffer depth
constexpr uint32_t DEFAULT_STORED_SIZE{4};

namespace command {
///@cond
constexpr uint8_t REG_PSR_B2{0x00};    // burst start: PSR(3) + TMP(3) = 6 bytes
constexpr uint8_t REG_PRS_CFG{0x06};   // PM_RATE[7:4] PM_PRC[3:0]
constexpr uint8_t REG_TMP_CFG{0x07};   // TMP_RATE[7:4] TM_PRC[3:0]
constexpr uint8_t REG_MEAS_CFG{0x08};  // bit7 COEF_RDY, bit6 SENSOR_RDY, bit5 TMP_RDY, bit4 PRS_RDY
constexpr uint8_t REG_CFG_REG{0x09};   // bit3 TMP_SHIFT_EN, bit2 PRS_SHIFT_EN
constexpr uint8_t REG_RESET{0x0C};
constexpr uint8_t REG_ID{0x0D};
constexpr uint8_t REG_COEF{0x10};  // 0x10..0x24 (21 bytes)
///@endcond
}  // namespace command
}  // namespace spa06

}  // namespace unit
}  // namespace m5
#endif
