/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_dof9.cpp
  @brief UnitDoF9 composite
*/
#include "unit_dof9.hpp"
#include <M5Utility.hpp>
#include <algorithm>

namespace m5 {
namespace unit {
using namespace m5::utility::mmh3;
using namespace m5::unit::types;

const char UnitDoF9::name[] = "UnitDoF9";
const types::uid_t UnitDoF9::uid{"UnitDoF9"_mmh3};
const types::attr_t UnitDoF9::attr{attribute::AccessI2C};

UnitDoF9::UnitDoF9(const uint8_t bmi_addr) : Component(0xFF), _bmi(bmi_addr), _bmm(0x14)
{
    auto cfg         = component_config();
    cfg.max_children = 2;
    cfg.clock        = std::min(_bmi.component_config().clock, _bmm.component_config().clock);
    component_config(cfg);

    // The DoF's BMM350 is a real device on the main bus, not a BMI270 AUX magnetometer, and the
    // BMI270's AUX pins are tied to that same bus: switching the interface on would turn the BMI270
    // into a second I2C master and lock the bus until the unit is power-cycled. Hence plain
    // UnitBMI270 here rather than UnitBMI270AuxBMM150 -- do not "upgrade" it.

    _valid = add(_bmi, 0) && add(_bmm, 1);
}

std::shared_ptr<Adapter> UnitDoF9::ensure_adapter(const uint8_t ch)
{
    auto unit = child(ch);
    if (!unit) {
        M5_LIB_LOGE("Invalid channel %u", ch);
        return std::make_shared<Adapter>();  // Empty adapter
    }
    auto ad = asAdapter<AdapterI2C>(Adapter::Type::I2C);
    return ad ? std::shared_ptr<Adapter>(ad->duplicate(unit->address())) : std::make_shared<Adapter>();
}

}  // namespace unit
}  // namespace m5
