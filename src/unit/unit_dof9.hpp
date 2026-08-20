/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_dof9.hpp
  @brief UnitDoF9 (U229): 9-axis composite = BMI270 (accel/gyro) + BMM350 (mag)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_DOF9_HPP
#define M5_UNIT_IMU_UNIT_UNIT_DOF9_HPP

#include <M5UnitComponent.hpp>
#include "unit_bmi270.hpp"
#include "unit_bmm350.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitDoF9
  @brief Unit DoF9: BMI270 + BMM350, both directly on the GROVE main bus (no AUX bypass). The BMI270
  address is SW1-selectable (0x68 default / 0x69); pass it to the constructor (e.g. from
  bmi270::detectAddress). BMM350 is fixed at 0x14. Add imu() and mag() to an AhrsController.
 */
class UnitDoF9 : public Component {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitDoF9, 0xFF /* dummy: parent has no device of its own */);

public:
    //! @brief Construct with the BMI270's I2C address (the composite children derive from this)
    //! @param[in] bmi_addr BMI270 I2C address (defaults to 0x68)
    explicit UnitDoF9(const uint8_t bmi_addr = 0x68);
    //! @brief Destructor
    virtual ~UnitDoF9()
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
    bool _valid{};
};

}  // namespace unit
}  // namespace m5
#endif
