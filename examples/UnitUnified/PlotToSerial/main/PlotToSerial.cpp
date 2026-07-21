/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitIMU series with PlotToSerial
  Output:
   - Serial: Arduino IDE Serial Plotter format (Teleplot)
   - LCD-equipped boards additionally show the same latest sensor values on-screen
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedIMU.h>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)
#include <wiring/m5_unit_imu_wiring.hpp>      // This library: classic Core Port C helper

// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_BMM350) && !defined(USING_UNIT_DOF6) && \
    !defined(USING_UNIT_DOF9) && !defined(USING_UNIT_DOF10) && !defined(USING_UNIT_MINI_IMU_PRO)
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
// For UnitMiniIMUPro (U171) -- 10-axis BMI270 + BMM150 + BMP280 barometer
// #define USING_UNIT_MINI_IMU_PRO
// Following devices will be supported in later plans:
// USING_BUILTIN_IMU
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
#elif defined(USING_UNIT_MINI_IMU_PRO)
// BMI270 is fixed at 0x68 (no address switch on this board); BMM150 is 0x10, BMP280 barometer is
// 0x76 (driver from M5Unit-ENV).
m5::unit::UnitMiniIMUPro unit{};
m5::imu::imu_convert_param_t param{};
#else
#error Please choose unit!
#endif

M5Canvas canvas(&lcd);
bool has_lcd{};
constexpr uint32_t LCD_INTERVAL_MS{33};  // ~30fps
uint32_t last_lcd_ms{};

//! @brief Draw the latest sensor values on the LCD (flicker-free: draws off-screen, blits once)
//! @details Layout adapts to the sprite height so it stays readable from small square panels
//! (e.g. AtomS3 128x128) up to large ones (e.g. Tab5).
void draw_lcd()
{
    if (!has_lcd) {
        return;
    }

    canvas.fillScreen(TFT_BLACK);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);

    const int ts          = (canvas.height() >= 240) ? (canvas.height() / 120) : 1;
    const int line_height = 8 * ts + 2;
    int line              = 0;
    canvas.setTextSize(ts);

#if defined(USING_UNIT_MINI_IMU)
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.print("MiniIMU");
    const auto d = unit.latest();
    // One axis per line: at full scale (+-16g / +-2000dps) a combined line would overflow a 128px panel
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Ax:%+.2f", d.accel_x() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Ay:%+.2f", d.accel_y() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Az:%+.2f", d.accel_z() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gx:%+.1f", d.gyro_x() * param.gyro_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gy:%+.1f", d.gyro_y() * param.gyro_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gz:%+.1f", d.gyro_z() * param.gyro_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("T:%.1fC", d.temp_adc() * param.temp_res + param.temp_offset);
#elif defined(USING_UNIT_BMM350)
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.print("BMM350");
    const auto d = unit.latest();
    // One axis per line so the values always fit a 128px panel at text size 1
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Mx:%+.1f", d.x_ut());
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("My:%+.1f", d.y_ut());
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Mz:%+.1f", d.z_ut());
#elif defined(USING_UNIT_DOF6)
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.print("DoF6");
    const auto d = unit.latest();
    // One axis per line: at full scale (+-16g / +-2000dps) a combined line would overflow a 128px panel
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Ax:%+.2f", d.accel_x() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Ay:%+.2f", d.accel_y() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Az:%+.2f", d.accel_z() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gx:%+.1f", d.gyro_x() * param.gyro_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gy:%+.1f", d.gyro_y() * param.gyro_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gz:%+.1f", d.gyro_z() * param.gyro_res);
#elif defined(USING_UNIT_DOF9)
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.print("DoF9");
    const auto a = unit.imu().latest();
    // One axis per line: at full scale (+-16g / +-2000dps) a combined line would overflow a 128px panel
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Ax:%+.2f", a.accel_x() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Ay:%+.2f", a.accel_y() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Az:%+.2f", a.accel_z() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gx:%+.1f", a.gyro_x() * param.gyro_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gy:%+.1f", a.gyro_y() * param.gyro_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gz:%+.1f", a.gyro_z() * param.gyro_res);
    const auto m = unit.mag().latest();
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Mx:%+.1f", m.x_ut());
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("My:%+.1f", m.y_ut());
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Mz:%+.1f", m.z_ut());
#elif defined(USING_UNIT_DOF10)
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.print("DoF10");
    const auto a = unit.imu().latest();
    // One axis per line: at full scale (+-16g / +-2000dps) a combined line would overflow a 128px panel
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Ax:%+.2f", a.accel_x() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Ay:%+.2f", a.accel_y() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Az:%+.2f", a.accel_z() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gx:%+.1f", a.gyro_x() * param.gyro_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gy:%+.1f", a.gyro_y() * param.gyro_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gz:%+.1f", a.gyro_z() * param.gyro_res);
    const auto m = unit.mag().latest();
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Mx:%+.1f", m.x_ut());
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("My:%+.1f", m.y_ut());
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Mz:%+.1f", m.z_ut());
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("P:%.1fhPa", unit.baro().pressure());
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Alt:%.1fm bT:%.1fC", unit.baro().altitude(), unit.baro().temperature());
#elif defined(USING_UNIT_MINI_IMU_PRO)
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.print("IMUPro");
    const auto a = unit.imu().latest();
    // One axis per line: at full scale (+-16g / +-2000dps) a combined line would overflow a 128px panel
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Ax:%+.2f", a.accel_x() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Ay:%+.2f", a.accel_y() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Az:%+.2f", a.accel_z() * param.accel_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gx:%+.1f", a.gyro_x() * param.gyro_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gy:%+.1f", a.gyro_y() * param.gyro_res);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Gz:%+.1f", a.gyro_z() * param.gyro_res);
    const auto m = unit.mag().latest();
    // BMM150's compensated output is 16 LSB/uT (see unit_bmm150_data.hpp)
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Mx:%+.1f", m.comp_x() / 16.0f);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("My:%+.1f", m.comp_y() / 16.0f);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("Mz:%+.1f", m.comp_z() / 16.0f);
    canvas.setCursor(4, 2 + line++ * line_height);
    canvas.printf("P:%.1fhPa", unit.baro().pressure());
    canvas.setCursor(4, 2 + line++ * line_height);
    // UnitBMP280 has no altitude(); reuse M5Unit-ENV's chip-agnostic barometric-formula helper
    canvas.printf("Alt:%.1fm bT:%.1fC", m5::unit::spa06::altitude_m(unit.baro().pressure(), 1013.25f),
                  unit.baro().temperature());
#endif

    canvas.pushSprite(0, 0);
}

}  // namespace

void setup()
{
    M5.begin();

    M5.setTouchButtonHeightByRatio(100);
    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }
    has_lcd = lcd.width() >= 80;  // treat the 5x5 ATOM Matrix LED "display" as no-LCD
    if (has_lcd) {
        // White-on-black text only -> a 1-bit palette sprite is enough. This keeps the full-screen
        // sprite in internal RAM even on large panels (Tab5 1280x720: 1bit=115KB vs 8bit=921KB, which
        // would not fit) and cuts the pushSprite transfer to 1/8.
        canvas.setColorDepth(1);
        canvas.setPsram(false);
        canvas.createSprite(lcd.width(), lcd.height());
    }

    // UnitMiniIMU / UnitDoF6 / UnitDoF9 / UnitDoF10: NessoN1 -> SoftwareI2C (M5HAL), NanoC6 / NanoH2 ->
    //                                                M5.Ex_I2C, others -> Wire
    // UnitBMM350: GROVE (all boards)
    bool ok{};
#if defined(USING_UNIT_MINI_IMU) || defined(USING_UNIT_DOF6) || defined(USING_UNIT_DOF9) || \
    defined(USING_UNIT_DOF10) || defined(USING_UNIT_MINI_IMU_PRO)
    // On classic M5Stack Core (board_M5Stack) GROVE Port A *is* the internal I2C bus, and Gray/Fire
    // carry a built-in MPU6886 or BMI270 at 0x68 -- the same address as UnitMiniIMU/UnitDoF6/UnitDoF9/
    // UnitDoF10's BMI270. Route the external unit to GROVE Port C on a bus of its own so it never
    // collides with the built-in IMU (same handling as the AhrsDemo example). UnitBMM350 (0x14) does
    // not collide, so it keeps addI2C.
    if (M5.getBoard() == m5::board_t::board_M5Stack) {
        // Route to GROVE Port C on the idle lgfx I2C port (see i2cPortC in m5_unit_imu_wiring.hpp).
        ok = m5::unit::imu::wiring::i2cPortC(Units, unit);
    } else {
        ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U);
    }
#else
    ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U);
#endif

#if (defined(USING_UNIT_DOF6) || defined(USING_UNIT_DOF9) || defined(USING_UNIT_DOF10)) && defined(ARDUINO)
    // The DoF BMI270 address is SW1-selectable (0x68 default / 0x69) and fixed at compile time via
    // DOF_BMI270_ADDRESS. Now that the bus above is connected (addI2C()/i2cWire() have called
    // Wire.begin() whenever their backend is Wire), probe both candidates as a diagnostic and warn if
    // the address actually present differs from the one this build targets. Arduino-only: the probe
    // talks over Wire directly; ESP-IDF builds skip it (a mismatch still shows as a begin failure).
    // Also skipped on classic Core, where Port C runs on the lgfx I2C_Class and Wire is never begun.
    if (M5.getBoard() != m5::board_t::board_M5Stack) {
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
#elif defined(USING_UNIT_DOF9) || defined(USING_UNIT_DOF10) || defined(USING_UNIT_MINI_IMU_PRO)
    unit.imu().imuConvertParam(param);
#endif
}

void loop()
{
    M5.update();
    Units.update();

    if (has_lcd) {
        const auto now = m5::utility::millis();
        if (now - last_lcd_ms >= LCD_INTERVAL_MS) {
            last_lcd_ms = now;
            draw_lcd();
        }
    }

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
#elif defined(USING_UNIT_MINI_IMU_PRO)
    // UnitMiniIMUPro is a composite with no measurement of its own; gate on its sub-units instead.
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
            // BMM150's compensated output is 16 LSB/uT (see unit_bmm150_data.hpp)
            M5.Log.printf(">mx:%.3f\n>my:%.3f\n>mz:%.3f\n", m.comp_x() / 16.0f, m.comp_y() / 16.0f, m.comp_z() / 16.0f);
        }
        if (unit.baro().updated()) {
            // UnitBMP280 has no altitude(); reuse M5Unit-ENV's chip-agnostic barometric-formula helper
            M5.Log.printf(">press:%.2f\n>alt:%.2f\n>btemp:%.2f\n", unit.baro().pressure(),
                          m5::unit::spa06::altitude_m(unit.baro().pressure(), 1013.25f), unit.baro().temperature());
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
