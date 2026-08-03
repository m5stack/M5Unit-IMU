/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_dof10.hpp
  @brief UnitDoF10 (U230): 10-axis composite = BMI270 (accel/gyro) + BMM350 (mag) + SPA06 (baro)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_DOF10_HPP
#define M5_UNIT_IMU_UNIT_UNIT_DOF10_HPP

#include <M5UnitComponent.hpp>
// UnitSPA06 is provided by M5Unit-ENV (the barometer drivers live there)
#include <unit/unit_SPA06.hpp>
#include "unit_bmi270.hpp"
#include "unit_bmm350.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitDoF10
  @brief Unit DoF10: BMI270 + BMM350 + SPA06, all directly on the GROVE main bus (no AUX bypass). The
  BMI270 address is SW1-selectable (0x68 default / 0x69); pass it to the constructor (e.g. from
  bmi270::detectAddress). BMM350 is fixed at 0x14, SPA06 is fixed at 0x76. Add imu() and mag() to an
  AhrsController; read baro() directly (not AHRS-fused).
 */
class UnitDoF10 : public Component {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitDoF10, 0xFF /* dummy: parent has no device of its own */);

public:
    //! @brief Construct with the BMI270's I2C address (the composite children derive from this)
    //! @param[in] bmi_addr BMI270 I2C address (defaults to 0x68)
    explicit UnitDoF10(const uint8_t bmi_addr = 0x68);
    //! @brief Destructor
    virtual ~UnitDoF10()
    {
    }

    //! @brief Accel/gyro sub-unit (add to an AhrsController)
    UnitBMI270& imu()
    {
        return _bmi;
    }
    //! @brief Magnetometer sub-unit (add to an AhrsController)
    UnitBMM350& mag()
    {
        return _bmm;
    }
    //! @brief Barometer sub-unit (read directly; not AHRS-fused)
    UnitSPA06& baro()
    {
        return _spa;
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
    UnitBMI270 _bmi;
    UnitBMM350 _bmm;
    UnitSPA06 _spa;
    bool _valid{};
};

}  // namespace unit
}  // namespace m5
#endif
