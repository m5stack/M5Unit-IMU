/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mini_imu_pro.cpp
  @brief UnitMiniIMUPro composite
*/
#include "unit_mini_imu_pro.hpp"
#include <M5Utility.hpp>
#include <algorithm>

namespace m5 {
namespace unit {
using namespace m5::utility::mmh3;
using namespace m5::unit::types;

const char UnitMiniIMUPro::name[] = "UnitMiniIMUPro";
const types::uid_t UnitMiniIMUPro::uid{"UnitMiniIMUPro"_mmh3};
const types::attr_t UnitMiniIMUPro::attr{attribute::AccessI2C};

UnitMiniIMUPro::UnitMiniIMUPro(const uint8_t bmi_addr) : Component(0xFF), _bmi(bmi_addr), _baro(0x76)
{
    auto cfg         = component_config();
    cfg.max_children = 2;
    cfg.clock        = std::min(_bmi.component_config().clock, _baro.component_config().clock);
    component_config(cfg);

    // Only the BMI270 and the BMP280 sit on the main bus. The board's BMM150 is wired to the
    // BMI270's AUX interface (ASDX/ASCX), so it is reached through UnitBMI270's AUX bridge
    // (UnitBMI270AuxEnabled probes it on start) rather than as a child of its own.
    _valid = add(_bmi, 0) && add(_baro, 1);
}

std::shared_ptr<Adapter> UnitMiniIMUPro::ensure_adapter(const uint8_t ch)
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
