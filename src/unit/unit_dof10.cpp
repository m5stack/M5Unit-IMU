/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_dof10.cpp
  @brief UnitDoF10 composite
*/
#include "unit_dof10.hpp"
#include <M5Utility.hpp>
#include <algorithm>

namespace m5 {
namespace unit {
using namespace m5::utility::mmh3;
using namespace m5::unit::types;

const char UnitDoF10::name[] = "UnitDoF10";
const types::uid_t UnitDoF10::uid{"UnitDoF10"_mmh3};
const types::attr_t UnitDoF10::attr{attribute::AccessI2C};

UnitDoF10::UnitDoF10(const uint8_t bmi_addr) : Component(0xFF), _bmi(bmi_addr), _bmm(0x14), _spa(0x76)
{
    auto cfg         = component_config();
    cfg.max_children = 3;
    cfg.clock =
        std::min(std::min(_bmi.component_config().clock, _bmm.component_config().clock), _spa.component_config().clock);
    component_config(cfg);

    // The DoF's BMM350 is a real device on the main bus, not a BMI270 AUX magnetometer, and the
    // BMI270's AUX pins are tied to that same bus. That is measured, not assumed: on a DoF10
    // (twice from a freshly power-cycled unit) writing IF_CONF.aux_en
    // succeeds and the very next transaction is already gone. No auxiliary transaction has to be
    // triggered for it -- switching the interface on is enough, and SDA stays held low from then on.
    // Nothing the host can write brings it back: nine clock pulses, IF_CONF <- 0, PWR_CTRL <- 0 and
    // the soft reset command all fail, because none of them reach a chip that is holding the wire.
    // It outlives a host reset too, since the unit keeps its own power, so the only cure is
    // unplugging it. Hence plain UnitBMI270 here rather than UnitBMI270AuxBMM150 -- do not
    // "upgrade" it.

    _valid = add(_bmi, 0) && add(_bmm, 1) && add(_spa, 2);
}

std::shared_ptr<Adapter> UnitDoF10::ensure_adapter(const uint8_t ch)
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
