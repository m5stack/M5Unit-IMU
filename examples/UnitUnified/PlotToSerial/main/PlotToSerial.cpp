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
#if !defined(USING_UNIT_IMU_MINI)
// For UnitIMUMini (SKU: TBD)
#define USING_UNIT_IMU_MINI
// Following devices will be supported in later plans:
// USING_UNIT_BMM350 (U231) / USING_UNIT_DOF6 (U228) / USING_BUILTIN_IMU
#endif

namespace {
m5::unit::UnitUnified Units;

#if defined(USING_UNIT_IMU_MINI)
m5::unit::UnitIMUMini unit{};
#else
#error Please choose unit!
#endif

m5::imu::imu_convert_param_t param{};
}  // namespace

void setup()
{
    M5.begin();

    // UnitIMUMini: NessoN1 -> SoftwareI2C (M5HAL), NanoC6 / NanoH2 -> M5.Ex_I2C, others -> Wire
    if (!m5::unit::wiring::addI2C(Units, unit, 400 * 1000U) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

    unit.imuConvertParam(param);
    M5.Log.printf("ax,ay,az,gx,gy,gz\n");
}

void loop()
{
    M5.update();
    Units.update();
    if (unit.updated()) {
        const auto d = unit.latest();
        M5.Log.printf("%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n", d.accel_x() * param.accel_res, d.accel_y() * param.accel_res,
                      d.accel_z() * param.accel_res, d.gyro_x() * param.gyro_res, d.gyro_y() * param.gyro_res,
                      d.gyro_z() * param.gyro_res);
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
    constexpr uint32_t FEED_INTERVAL_MS   = 2000;
    constexpr TickType_t FEED_SLEEP_TICKS = pdMS_TO_TICKS(5);
    static uint32_t s_next_feed_ms        = 0;
    const uint32_t now_ms                 = static_cast<uint32_t>(esp_timer_get_time() / 1000);
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
