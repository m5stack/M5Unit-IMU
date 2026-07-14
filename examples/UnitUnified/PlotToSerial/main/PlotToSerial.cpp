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
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_BMM350)
// For UnitMiniIMU (SKU: TBD)
#define USING_UNIT_MINI_IMU
// For UnitBMM350 (U231)
// #define USING_UNIT_BMM350
// Following devices will be supported in later plans:
// USING_UNIT_DOF6 (U228) / USING_BUILTIN_IMU
#endif

namespace {
m5::unit::UnitUnified Units;

#if defined(USING_UNIT_MINI_IMU)
m5::unit::UnitMiniIMU unit{};
m5::imu::imu_convert_param_t param{};
#elif defined(USING_UNIT_BMM350)
m5::unit::UnitBMM350 unit{};
#else
#error Please choose unit!
#endif

}  // namespace

void setup()
{
    M5.begin();

    // UnitMiniIMU: NessoN1 -> SoftwareI2C (M5HAL), NanoC6 / NanoH2 -> M5.Ex_I2C, others -> Wire
    // UnitBMM350: GROVE (all boards)
    bool ok{};
#if defined(USING_UNIT_MINI_IMU)
    // On classic M5Stack Core (board_M5Stack) GROVE Port A *is* the internal I2C bus, and Gray/Fire
    // carry a built-in MPU6886 at 0x68 -- the same address as UnitMiniIMU. Route the external unit to
    // GROVE Port C on a separate Wire1 bus there so it never collides with the built-in IMU (same
    // handling as the AhrsDemo example). UnitBMM350 (0x14) does not collide, so it keeps addI2C.
    if (M5.getBoard() == m5::board_t::board_M5Stack) {
        // Grove maps connector pin1 -> SCL and pin2 -> SDA (same wires as UART RX/TX). M5Unified
        // exposes Port C as port_c_pin1 (RXD) / port_c_pin2 (TXD), so SDA = pin2, SCL = pin1. The
        // unit already carries its own 400kHz clock (set in UnitMPU6886's constructor), so reuse it.
        const int sda = M5.getPin(m5::pin_name_t::port_c_pin2);
        const int scl = M5.getPin(m5::pin_name_t::port_c_pin1);
        ok            = m5::unit::wiring::i2cWire(Units, unit, Wire1, sda, scl, unit.component_config().clock);
    } else {
        ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U);
    }
#else
    ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U);
#endif
    if (!ok || !Units.begin()) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

#if defined(USING_UNIT_MINI_IMU)
    unit.imuConvertParam(param);
#endif
}

void loop()
{
    M5.update();
    Units.update();
    if (unit.updated()) {
        const auto d = unit.latest();
#if defined(USING_UNIT_MINI_IMU)
        // Teleplot format (https://github.com/nesnes/teleplot): one ">name:value" per line.
        M5.Log.printf(">ax:%.3f\n>ay:%.3f\n>az:%.3f\n>gx:%.2f\n>gy:%.2f\n>gz:%.2f\n", d.accel_x() * param.accel_res,
                      d.accel_y() * param.accel_res, d.accel_z() * param.accel_res, d.gyro_x() * param.gyro_res,
                      d.gyro_y() * param.gyro_res, d.gyro_z() * param.gyro_res);
#elif defined(USING_UNIT_BMM350)
        M5.Log.printf(">mx:%.3f\n>my:%.3f\n>mz:%.3f\n", d.x_ut(), d.y_ut(), d.z_ut());
#endif
    }
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
