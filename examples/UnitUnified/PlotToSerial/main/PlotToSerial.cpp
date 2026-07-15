/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitIMU series with PlotToSerial
  Output: Arduino IDE Serial Plotter format
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedIMU.h>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)

// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_BMM350) && !defined(USING_UNIT_DOF6) && \
    !defined(USING_UNIT_DOF9) && !defined(USING_UNIT_DOF10)
// For UnitMiniIMU (U095)
// #define USING_UNIT_MINI_IMU
// For UnitBMM350 (U231)
// #define USING_UNIT_BMM350
// For UnitDoF6 (U228) -- 6-axis BMI270
// #define USING_UNIT_DOF6
// For UnitDoF9 (U229) -- 9-axis BMI270 + BMM350
// #define USING_UNIT_DOF9
// For UnitDoF10 (U230) -- 10-axis BMI270 + BMM350 + SPA06 barometer
// #define USING_UNIT_DOF10
// Following devices will be supported in later plans:
// USING_BUILTIN_IMU
#endif

// Unit DoF BMI270 I2C address, selected by the board's SW1 switch (0x68 default / 0x69). Set this to
// match your switch -- ArduinoIDE: edit below; PlatformIO: add -D DOF_BMI270_ADDRESS=0x69 to build_flags.
#ifndef DOF_BMI270_ADDRESS
#define DOF_BMI270_ADDRESS 0x68
#endif

namespace {
m5::unit::UnitUnified Units;

#if defined(USING_UNIT_MINI_IMU)
m5::unit::UnitMiniIMU unit{};
m5::imu::imu_convert_param_t param{};
#elif defined(USING_UNIT_BMM350)
m5::unit::UnitBMM350 unit{};
#elif defined(USING_UNIT_DOF6)
// UnitDoF6 is a plain alias of UnitBMI270. The BMI270 address (0x68/0x69) follows the DoF board's SW1
// switch; build with DOF_BMI270_ADDRESS to match it (see the #define above).
m5::unit::UnitDoF6 unit{DOF_BMI270_ADDRESS};
m5::imu::imu_convert_param_t param{};
#elif defined(USING_UNIT_DOF9)
// BMI270 address (0x68/0x69) follows the DoF board's SW1 switch via DOF_BMI270_ADDRESS; BMM350 is 0x14.
m5::unit::UnitDoF9 unit{DOF_BMI270_ADDRESS};
m5::imu::imu_convert_param_t param{};
#elif defined(USING_UNIT_DOF10)
// BMI270 address (0x68/0x69) follows the DoF board's SW1 switch via DOF_BMI270_ADDRESS; BMM350 is 0x14,
// SPA06 barometer is 0x76.
m5::unit::UnitDoF10 unit{DOF_BMI270_ADDRESS};
m5::imu::imu_convert_param_t param{};
#else
#error Please choose unit!
#endif

}  // namespace

void setup()
{
    M5.begin();

    // UnitMiniIMU / UnitDoF6 / UnitDoF9 / UnitDoF10: NessoN1 -> SoftwareI2C (M5HAL), NanoC6 / NanoH2 ->
    //                                                M5.Ex_I2C, others -> Wire
    // UnitBMM350: GROVE (all boards)
    bool ok{};
#if defined(USING_UNIT_MINI_IMU) || defined(USING_UNIT_DOF6) || defined(USING_UNIT_DOF9) || defined(USING_UNIT_DOF10)
    // On classic M5Stack Core (board_M5Stack) GROVE Port A *is* the internal I2C bus, and Gray/Fire
    // carry a built-in MPU6886 or BMI270 at 0x68 -- the same address as UnitMiniIMU/UnitDoF6/UnitDoF9/
    // UnitDoF10's BMI270. Route the external unit to GROVE Port C on the free Arduino Wire bus (M5Unified uses
    // In_I2C/Ex_I2C, not Wire) so it never collides with the built-in IMU (same handling as the
    // AhrsDemo example). UnitBMM350 (0x14) does not collide, so it keeps addI2C.
    if (M5.getBoard() == m5::board_t::board_M5Stack) {
        // Grove maps connector pin1 -> SCL and pin2 -> SDA (same wires as UART RX/TX). M5Unified
        // exposes Port C as port_c_pin1 (RXD) / port_c_pin2 (TXD), so SDA = pin2, SCL = pin1. The
        // unit already carries its own 400kHz clock (set in UnitMPU6886's constructor), so reuse it.
        const int sda = M5.getPin(m5::pin_name_t::port_c_pin2);
        const int scl = M5.getPin(m5::pin_name_t::port_c_pin1);
        ok            = m5::unit::wiring::i2cWire(Units, unit, Wire, sda, scl, unit.component_config().clock);
    } else {
        ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U);
    }
#else
    ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U);
#endif

#if defined(USING_UNIT_DOF6) || defined(USING_UNIT_DOF9) || defined(USING_UNIT_DOF10)
    // The DoF BMI270 address is SW1-selectable (0x68 default / 0x69) and fixed at compile time via
    // DOF_BMI270_ADDRESS. Now that the bus above is connected (addI2C()/i2cWire() have called
    // Wire.begin() whenever their backend is Wire), probe both candidates as a diagnostic and warn if
    // the address actually present differs from the one this build targets.
    {
        uint8_t detected{DOF_BMI270_ADDRESS};
        m5::unit::bmi270::detectAddress(
            [](const uint8_t addr, const uint8_t reg, uint8_t& out) {
                Wire.beginTransmission(addr);
                Wire.write(reg);
                if (Wire.endTransmission(false) != 0) {
                    return false;
                }
                if (Wire.requestFrom(addr, static_cast<size_t>(1)) != 1) {
                    return false;
                }
                out = static_cast<uint8_t>(Wire.read());
                return true;
            },
            detected);
        if (detected != DOF_BMI270_ADDRESS) {
            M5_LOGW(
                "DoF BMI270 detected at 0x%02X but this build targets 0x%02X; rebuild with "
                "-D DOF_BMI270_ADDRESS=0x%02X",
                detected, DOF_BMI270_ADDRESS, detected);
        } else {
            M5_LOGI("DoF BMI270 at 0x%02X", DOF_BMI270_ADDRESS);
        }
    }
#endif
#if defined(USING_UNIT_DOF6)
    // DoF6 is a 6-axis BMI270 with no magnetometer -> skip the AUX BMM150 probe (silences a harmless
    // begin() error).
    {
        auto c              = unit.config();
        c.enable_aux_bmm150 = false;
        unit.config(c);
    }
#endif

    if (!ok || !Units.begin()) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

#if defined(USING_UNIT_MINI_IMU) || defined(USING_UNIT_DOF6)
    unit.imuConvertParam(param);
#elif defined(USING_UNIT_DOF9) || defined(USING_UNIT_DOF10)
    unit.imu().imuConvertParam(param);
#endif
}

void loop()
{
    M5.update();
    Units.update();
#if defined(USING_UNIT_DOF9)
    // UnitDoF9 is a composite with no measurement of its own; gate on its sub-units instead.
    if (unit.imu().updated() || unit.mag().updated()) {
        if (unit.imu().updated()) {
            const auto a = unit.imu().latest();
            // Teleplot format (https://github.com/nesnes/teleplot): one ">name:value" per line.
            M5.Log.printf(">ax:%.3f\n>ay:%.3f\n>az:%.3f\n>gx:%.2f\n>gy:%.2f\n>gz:%.2f\n", a.accel_x() * param.accel_res,
                          a.accel_y() * param.accel_res, a.accel_z() * param.accel_res, a.gyro_x() * param.gyro_res,
                          a.gyro_y() * param.gyro_res, a.gyro_z() * param.gyro_res);
        }
        if (unit.mag().updated()) {
            const auto m = unit.mag().latest();
            M5.Log.printf(">mx:%.3f\n>my:%.3f\n>mz:%.3f\n", m.x_ut(), m.y_ut(), m.z_ut());
        }
    }
#elif defined(USING_UNIT_DOF10)
    // UnitDoF10 is a composite with no measurement of its own; gate on its sub-units instead.
    if (unit.imu().updated() || unit.mag().updated() || unit.baro().updated()) {
        if (unit.imu().updated()) {
            const auto a = unit.imu().latest();
            // Teleplot format (https://github.com/nesnes/teleplot): one ">name:value" per line.
            M5.Log.printf(">ax:%.3f\n>ay:%.3f\n>az:%.3f\n>gx:%.2f\n>gy:%.2f\n>gz:%.2f\n", a.accel_x() * param.accel_res,
                          a.accel_y() * param.accel_res, a.accel_z() * param.accel_res, a.gyro_x() * param.gyro_res,
                          a.gyro_y() * param.gyro_res, a.gyro_z() * param.gyro_res);
        }
        if (unit.mag().updated()) {
            const auto m = unit.mag().latest();
            M5.Log.printf(">mx:%.3f\n>my:%.3f\n>mz:%.3f\n", m.x_ut(), m.y_ut(), m.z_ut());
        }
        if (unit.baro().updated()) {
            M5.Log.printf(">press:%.2f\n>alt:%.2f\n>btemp:%.2f\n", unit.baro().pressure(), unit.baro().altitude(),
                          unit.baro().temperature());
        }
    }
#else
    if (unit.updated()) {
        const auto d = unit.latest();
#if defined(USING_UNIT_MINI_IMU)
        // Teleplot format (https://github.com/nesnes/teleplot): one ">name:value" per line.
        M5.Log.printf(">ax:%.3f\n>ay:%.3f\n>az:%.3f\n>gx:%.2f\n>gy:%.2f\n>gz:%.2f\n>temp:%.2f\n",
                      d.accel_x() * param.accel_res, d.accel_y() * param.accel_res, d.accel_z() * param.accel_res,
                      d.gyro_x() * param.gyro_res, d.gyro_y() * param.gyro_res, d.gyro_z() * param.gyro_res,
                      d.temp_adc() * param.temp_res + param.temp_offset);
#elif defined(USING_UNIT_BMM350)
        M5.Log.printf(">mx:%.3f\n>my:%.3f\n>mz:%.3f\n", d.x_ut(), d.y_ut(), d.z_ut());
#elif defined(USING_UNIT_DOF6)
        // BMI270's Data carries no on-chip temperature reading (unlike UnitMiniIMU/MPU6886).
        M5.Log.printf(">ax:%.3f\n>ay:%.3f\n>az:%.3f\n>gx:%.2f\n>gy:%.2f\n>gz:%.2f\n", d.accel_x() * param.accel_res,
                      d.accel_y() * param.accel_res, d.accel_z() * param.accel_res, d.gyro_x() * param.gyro_res,
                      d.gyro_y() * param.gyro_res, d.gyro_z() * param.gyro_res);
#endif
    }
#endif
}

#if !defined(ARDUINO)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#if CONFIG_FREERTOS_UNICORE
// Single-core SoCs: run loop() back-to-back, but every ~2 s yield a 5 ms slice so the IDLE task
// runs and feeds the task watchdog (default 5 s).
static inline void feedIdleTaskPeriodically(void)
{
    constexpr uint32_t FEED_INTERVAL_MS{2000};
    constexpr TickType_t FEED_SLEEP_TICKS{pdMS_TO_TICKS(5)};
    static uint32_t s_next_feed_ms = 0;
    const uint32_t now_ms          = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    if (now_ms >= s_next_feed_ms) {
        s_next_feed_ms = now_ms + FEED_INTERVAL_MS;
        vTaskDelay(FEED_SLEEP_TICKS);
    }
}
#endif

extern "C" void app_main(void)
{
    setup();
    for (;;) {
#if CONFIG_FREERTOS_UNICORE
        feedIdleTaskPeriodically();
#endif
        loop();
    }
}
#endif
