/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mini_imu_pro.hpp
  @brief UnitMiniIMUPro (U171): 9-axis composite = BMI270 (accel/gyro) + BMM150 (mag)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_MINI_IMU_PRO_HPP
#define M5_UNIT_IMU_UNIT_UNIT_MINI_IMU_PRO_HPP

#include <M5UnitComponent.hpp>
#include "unit_bmi270.hpp"
#include "unit_bmm150.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitMiniIMUPro
  @brief Unit Mini IMU-Pro (U171): BMI270 (0x68, fixed -- no address switch) + BMM150 (0x10), both
  directly on the GROVE main bus (no AUX bypass). The predecessor of Unit DoF9/DoF10, with a BMM150
  instead of the BMM350. Add imu() and mag() to an AhrsController.
  @note The board also carries a BMP280 barometer at 0x76; it is out of this library's scope (see
  M5Unit-ENV) and is simply left untouched on the bus. It shares 0x76 with the DoF10's SPA06, but
  UnitSPA06::begin() verifies the PROD_ID register and rejects a BMP280, so no misdetection occurs.
 */
class UnitMiniIMUPro : public Component {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMiniIMUPro, 0xFF /* dummy: parent has no device of its own */);

public:
    explicit UnitMiniIMUPro(const uint8_t bmi_addr = 0x68);
    virtual ~UnitMiniIMUPro()
    {
    }

    //! @brief Accel/gyro sub-unit (add to an AhrsController)
    UnitBMI270& imu()
    {
        return _bmi;
    }
    //! @brief Magnetometer sub-unit (add to an AhrsController)
    UnitBMM150& mag()
    {
        return _bmm;
    }

    //! @brief Begin: children are begun by the framework (all on the shared bus, any order)
    virtual bool begin() override
    {
        return _valid;
    }

protected:
    //! @brief Share this composite's bus with each child, duplicated at the child's address
    virtual std::shared_ptr<Adapter> ensure_adapter(const uint8_t ch) override;

private:
    UnitBMI270 _bmi;
    UnitBMM150 _bmm;
    bool _valid{};
};

}  // namespace unit
}  // namespace m5
#endif
