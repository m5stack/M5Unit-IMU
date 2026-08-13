/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitIMU series with motion events
  Demonstrates the MotionEventController pattern with HW events (BMI270 AnyMotion /
  MPU-6886 WoM) and an example software plugin (SoftwareFreeFall).
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedIMU.h>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)
#include <wiring/m5_unit_imu_wiring.hpp>      // This library: classic Core Port C helper
#include "software_free_fall.hpp"

// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_DOF6) && !defined(USING_UNIT_DOF9) && \
    !defined(USING_UNIT_DOF10) && !defined(USING_UNIT_MINI_IMU_PRO)
// For UnitMiniIMU (U095, MPU6886) -- HW AnyMotion via WoM
// #define USING_UNIT_MINI_IMU
// For UnitDoF6 (U228, BMI270) -- HW AnyMotion via feature engine
// #define USING_UNIT_DOF6
// For UnitDoF9 (U229, BMI270 + BMM350)
// #define USING_UNIT_DOF9
// For UnitDoF10 (U230, BMI270 + BMM350 + SPA06)
// #define USING_UNIT_DOF10
// For UnitMiniIMUPro (U171, BMI270 + BMM150 + BMP280)
// #define USING_UNIT_MINI_IMU_PRO
// Built-in IMU support will be added in a later plan.
#endif

// Unit DoF BMI270 I2C address, selected by the board's SW1 switch (0x68 default / 0x69). Set this to
// match your switch -- ArduinoIDE: edit below; PlatformIO: add -D DOF_BMI270_ADDRESS=0x69 to build_flags.
#ifndef DOF_BMI270_ADDRESS
#define DOF_BMI270_ADDRESS 0x68
#endif

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;

#if defined(USING_UNIT_MINI_IMU)
m5::unit::UnitMiniIMU unit{};
#elif defined(USING_UNIT_DOF6)
// UnitDoF6 is a plain alias of UnitBMI270. The BMI270 address (0x68/0x69) follows the DoF board's SW1
// switch; build with DOF_BMI270_ADDRESS to match it (see the #define above).
m5::unit::UnitDoF6 unit{DOF_BMI270_ADDRESS};
#elif defined(USING_UNIT_DOF9)
// BMI270 address (0x68/0x69) follows the DoF board's SW1 switch via DOF_BMI270_ADDRESS; BMM350 is 0x14.
m5::unit::UnitDoF9 unit{DOF_BMI270_ADDRESS};
#elif defined(USING_UNIT_DOF10)
// BMI270 address (0x68/0x69) follows the DoF board's SW1 switch via DOF_BMI270_ADDRESS; BMM350 is 0x14,
// SPA06 barometer is 0x76.
m5::unit::UnitDoF10 unit{DOF_BMI270_ADDRESS};
#elif defined(USING_UNIT_MINI_IMU_PRO)
// BMI270 is fixed at 0x68 (no address switch on this board) and the BMP280 barometer is 0x76
// (driver from M5Unit-ENV). The BMM150 is not on the main bus: it is an AUX target of the BMI270,
// so imu() carries the magnetometer too.
m5::unit::UnitMiniIMUPro unit{};
#else
#error Please choose unit!
#endif

m5::imu::MotionEventController motion;

}  // namespace

void setup()
{
    M5.begin();
    M5.setTouchButtonHeightByRatio(100);
    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

    // Sensor wiring; MotionEventController hookup lives in a follow-up change.
    lcd.fillScreen(TFT_BLACK);
    lcd.setCursor(0, 0);
    lcd.setTextSize(2);
    lcd.println("MotionEvents");
    lcd.println("(scaffolding)");

    M5_LOGI("MotionEvents scaffolding ready");
}

void loop()
{
    M5.update();
    // Controller update / BtnA cycle logic lives in a follow-up change.
    delay(20);
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
