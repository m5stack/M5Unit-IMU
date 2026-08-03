/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mini_imu_pro.hpp
  @brief UnitMiniIMUPro (U171): 10-axis composite = BMI270 (accel/gyro) + BMM150 (mag) + BMP280 (baro)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_MINI_IMU_PRO_HPP
#define M5_UNIT_IMU_UNIT_UNIT_MINI_IMU_PRO_HPP

#include <M5UnitComponent.hpp>
// UnitBMP280 is provided by M5Unit-ENV (the barometer drivers live there)
#include <unit/unit_BMP280.hpp>
#include "unit_bmi270_aux_bmm150.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitMiniIMUPro
  @brief Unit Mini IMU-Pro (U171): BMI270 (0x68, fixed -- no address switch) + BMP280 (0x76) on the
  GROVE main bus, with a BMM150 magnetometer wired to the BMI270's AUX interface (ASDX/ASCX), not to
  the main bus. The predecessor of Unit DoF9/DoF10, with a BMM150/BMP280 instead of the BMM350/SPA06.
  @details Because the BMM150 hangs off the BMI270's AUX bridge -- the same topology as CoreS3 and
  AtomS3R -- imu() alone provides all nine axes (UnitBMI270AuxBMM150 probes and drives the AUX BMM150, and
  reports Capability::Mag once it answers). Add imu() to an AhrsController; read baro() directly (it
  is not AHRS-fused). The barometer driver (UnitBMP280) comes from M5Unit-ENV.
 */
class UnitMiniIMUPro : public Component {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMiniIMUPro, 0xFF /* dummy: parent has no device of its own */);

public:
    //! @brief Construct with the BMI270's I2C address (the composite children derive from this)
    //! @param[in] bmi_addr BMI270 I2C address (defaults to 0x68)
    explicit UnitMiniIMUPro(const uint8_t bmi_addr = 0x68);
    //! @brief Destructor
    virtual ~UnitMiniIMUPro()
    {
    }

    //! @brief Accel/gyro/mag sub-unit (add to an AhrsController; the magnetometer arrives through
    //! this unit's AUX BMM150 bridge, so there is no separate mag sub-unit)
    UnitBMI270& imu()
    {
        return _bmi;
    }
    //! @brief Barometer sub-unit (read directly; not AHRS-fused)
    UnitBMP280& baro()
    {
        return _baro;
    }

    //! @brief Begin: children are begun by the framework (all on the shared bus, any order)
    virtual bool begin() override
    {
        return _valid;
    }

protected:
    // Share this composite's bus with each child, duplicated at the child's address
    virtual std::shared_ptr<Adapter> ensure_adapter(const uint8_t ch) override;

private:
    UnitBMI270AuxBMM150 _bmi;
    UnitBMP280 _baro;
    bool _valid{};
};

}  // namespace unit
}  // namespace m5
#endif
