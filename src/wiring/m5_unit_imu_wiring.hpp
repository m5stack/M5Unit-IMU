/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file m5_unit_imu_wiring.hpp
  @brief M5Unit-IMU specific wiring helpers (this library only; not part of M5UnitUnified's generic
  wiring)
  @details Helpers for connection cases specific to the units in this library, complementing
  M5UnitUnified's board-aware `m5::unit::wiring` (m5_unit_unified_wiring.hpp). Currently: routing a
  0x68-addressed external unit to GROVE Port C on the classic M5Stack Core family, where GROVE Port
  A *is* the internal I2C bus (built-in IMU collision).
*/
#ifndef M5_UNIT_IMU_WIRING_M5_UNIT_IMU_WIRING_HPP
#define M5_UNIT_IMU_WIRING_M5_UNIT_IMU_WIRING_HPP

#include <M5Unified.h>
#include <M5UnitUnified.h>

///@cond
// HP (high-performance) I2C port count: IDF >= 5.2 splits HP/LP counts (SOC_HP_I2C_NUM); older
// cores (e.g. Arduino core 2.x on IDF 4.4) only define SOC_I2C_NUM, which IS the HP count there.
#if defined(SOC_HP_I2C_NUM)
#define M5_UNIT_IMU_HP_I2C_NUM SOC_HP_I2C_NUM
#else
#define M5_UNIT_IMU_HP_I2C_NUM SOC_I2C_NUM
#endif
///@endcond

namespace m5 {
namespace unit {
namespace imu {
namespace wiring {

/*!
  @brief Route a GROVE unit to classic M5Stack Core Port C on a dedicated I2C bus
  @param units UnitUnified manager to register the unit with
  @param u Unit to connect
  @param bus_ready true: the Port C bus was already initialized by a previous call (shared-bus,
  multi-unit case) -- only attach the unit, do NOT re-run the bus init
  @return True if the unit was registered on the Port C bus
  @details On the classic Core family (board_M5Stack: Basic/Gray/Fire) GROVE Port A *is* the
  internal I2C bus, shared with Gray/Fire's built-in IMU at 0x68 -- the same address as this
  library's external accel/gyro units. This helper opens GROVE Port C (needs the M5GO/Fire base;
  Grove pin1 -> SCL = port_c_pin1/RXD, pin2 -> SDA = port_c_pin2/TXD) on a bus of its own.

  Why the lgfx I2C port 1: on classic Core BOTH HP I2C ports already sit with the lgfx i2c layer on
  the internal pins. In_I2C holds port 0 -- the LIVE internal bus (M5Unified keeps polling the
  IP5306 power IC there, and on Arduino the `Wire` object is that same port 0, so using Wire here
  would fight In_I2C over one peripheral with different pins -> NACKs). M5GFX's autodetect probe
  holds port 1, which is idle afterwards (display is SPI, backlight is PWM, no touch on this
  family). So this rebinds port 1 to the Port C pins through the lgfx-backed m5::I2C_Class --
  an identical mechanism on Arduino and ESP-IDF (any IDF 5.x; the lgfx layer wraps old/new drivers).

  Single-HP-I2C SoCs (ESP32-C6/H2 etc.) do not define I2C_NUM_1 and are never a classic Core, so
  on those this helper is compile-only and returns false.
 */
inline bool i2cPortC(m5::unit::UnitUnified& units, m5::unit::Component& u, const bool bus_ready = false)
{
    const int sda = M5.getPin(m5::pin_name_t::port_c_pin2);
    const int scl = M5.getPin(m5::pin_name_t::port_c_pin1);
    if (sda < 0 || scl < 0) {
        return false;  // no Port C on this board (needs the M5GO/Fire base)
    }
#if M5_UNIT_IMU_HP_I2C_NUM >= 2
    static m5::I2C_Class port_c_i2c;
    if (!bus_ready && !port_c_i2c.begin(I2C_NUM_1, sda, scl)) {
        return false;
    }
    return units.add(u, port_c_i2c);
#else
    (void)units;
    (void)bus_ready;
    return false;  // single-HP-I2C SoC: never a classic Core
#endif
}

}  // namespace wiring
}  // namespace imu
}  // namespace unit
}  // namespace m5
#endif
