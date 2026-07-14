/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_mpu9250.cpp
  @brief UnitMPU9250 composite (accel/gyro die + AK8963 mag child)
*/
#include "unit_mpu9250.hpp"
#include <M5Utility.hpp>
#include <algorithm>
#include "../imu/mag_detect.hpp"  // m5::unit::imu::enableAuxBypass

namespace m5 {
namespace unit {
using namespace m5::utility::mmh3;
using namespace m5::unit::types;

const char UnitMPU9250::name[] = "UnitMPU9250";
const types::uid_t UnitMPU9250::uid{"UnitMPU9250"_mmh3};
const types::attr_t UnitMPU9250::attr{attribute::AccessI2C};

UnitMPU9250::UnitMPU9250(const uint8_t addr, const uint8_t mag_addr) : UnitMPU6500(addr), _mag(mag_addr)
{
    auto cfg         = component_config();
    cfg.max_children = 1;
    // Share the slower of the two clocks on the common bus (accel/gyro 400k, AK8963 400k).
    cfg.clock = std::min(component_config().clock, _mag.component_config().clock);
    component_config(cfg);
    add(_mag, 0);
}

std::shared_ptr<Adapter> UnitMPU9250::ensure_adapter(const uint8_t ch)
{
    auto unit = child(ch);
    if (!unit) {
        M5_LIB_LOGE("Invalid channel %u", ch);
        return std::make_shared<Adapter>();  // Empty adapter
    }
    auto ad = asAdapter<AdapterI2C>(Adapter::Type::I2C);
    return ad ? std::shared_ptr<Adapter>(ad->duplicate(unit->address())) : std::make_shared<Adapter>();
}

bool UnitMPU9250::begin()
{
    // 1) Accel/gyro die (the MPU init writes INT_PIN_CFG=0xC0, which closes the aux bypass).
    if (!UnitMPU6500::begin()) {
        return false;
    }
    // 2) Re-open the MPU9250 aux bypass so the AK8963 answers at 0x0C. enableAuxBypass only talks to
    //    0x68 (this unit), so the address arg is ignored and this Component's own accessors are used.
    if (!m5::unit::imu::enableAuxBypass([this](uint8_t /*a*/, uint8_t r, uint8_t& o) { return readRegister8(r, o, 0); },
                                        [this](uint8_t /*a*/, uint8_t r, uint8_t v) { return writeRegister8(r, v); })) {
        M5_LIB_LOGE("MPU9250 aux bypass not enabled (WHO_AM_I != 0x71)");
        return false;
    }
    // 3) The framework begins the mag child (registered above) next, now that 0x0C is reachable.
    return true;
}

}  // namespace unit
}  // namespace m5
