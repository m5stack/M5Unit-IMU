/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example of using the IMU units via UnitPaHub

  Core ---> PaHub (0x70) -+--> ch:0 UnitMiniIMU (MPU6886 0x68)
                          |
                          +--> ch:1 UnitDoF6    (BMI270 0x68)
                          |
                          +--> ch:2 UnitDoF9    (BMI270 0x68 / BMM350 0x14)
                          |
                          +--> ch:3 UnitDoF10   (BMI270 0x68 / BMM350 0x14 / SPA06 0x76)
                          |
                          +--> ch:4 UnitBMM350  (BMM350 0x14)

  NOTICE:
  - Four units answer at 0x68 and three at 0x14, but they coexist on separate channels.
  - Leave SW1 of UnitDoF6 / DoF9 / DoF10 at the default (BMI270 0x68).
  - Classic M5Stack Core (Basic / Gray / Fire): connect the PaHub to GROVE Port C (needs the M5GO /
    Fire base). Gray / Fire carry a built-in IMU at 0x68 on the Port A bus, which would answer
    together with the 0x68 units behind the PaHub.
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedIMU.h>
#include <M5UnitUnifiedHUB.h>  // UnitPaHub
#include <algorithm>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)
#include <wiring/m5_unit_imu_wiring.hpp>      // This library: classic Core Port C helper

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;

m5::unit::UnitPaHub hub{};  // 0x70

m5::unit::UnitMiniIMU mini{};   // ch:0
m5::unit::UnitDoF6 dof6{0x68};  // ch:1 (UnitDoF6 is UnitBMI270, whose default is the chip's 0x69)
m5::unit::UnitDoF9 dof9{};      // ch:2
m5::unit::UnitDoF10 dof10{};    // ch:3
m5::unit::UnitBMM350 bmm350{};  // ch:4

m5::imu::imu_convert_param_t mini_param{};
m5::imu::imu_convert_param_t dof6_param{};
m5::imu::imu_convert_param_t dof9_param{};
m5::imu::imu_convert_param_t dof10_param{};

M5Canvas canvas(&lcd);
bool has_lcd{};
constexpr uint32_t LCD_INTERVAL_MS{100};  // 10fps is plenty for a value readout
uint32_t last_lcd_ms{};

//! @brief Log accel (g) of a BMI270 / MPU6886 sample in Teleplot format
template <typename Data>
void log_accel(const char* name, const Data& d, const m5::imu::imu_convert_param_t& p)
{
    M5.Log.printf(">%s_ax:%.3f\n>%s_ay:%.3f\n>%s_az:%.3f\n", name, d.accel_x() * p.accel_res, name,
                  d.accel_y() * p.accel_res, name, d.accel_z() * p.accel_res);
}

//! @brief Log the magnetic field (uT) of a BMM350 sample in Teleplot format
template <typename Data>
void log_mag(const char* name, const Data& d)
{
    M5.Log.printf(">%s_mx:%.2f\n>%s_my:%.2f\n>%s_mz:%.2f\n", name, d.x_ut(), name, d.y_ut(), name, d.z_ut());
}

template <typename Data>
void print_accel(const char* label, const Data& d, const m5::imu::imu_convert_param_t& p)
{
    canvas.printf("%-6s %+4.1f %+4.1f %+4.1f\n", label, d.accel_x() * p.accel_res, d.accel_y() * p.accel_res,
                  d.accel_z() * p.accel_res);
}

template <typename Data>
void print_mag(const char* label, const Data& d)
{
    canvas.printf("%-6s %+4.0f %+4.0f %+4.0f\n", label, d.x_ut(), d.y_ut(), d.z_ut());
}

//! @brief Draw the latest values of every unit (off-screen, then one blit: no flicker)
//! @details 8 lines of at most 21 characters, so text size 1 fits a 128x128 panel (AtomS3); larger
//! panels scale the text up to fill the screen (e.g. Tab5 1280x720).
void draw_lcd()
{
    constexpr int32_t COLUMNS{21};
    constexpr int32_t LINES{8};
    const int32_t ts =
        std::max<int32_t>(1, std::min<int32_t>(canvas.width() / (COLUMNS * 6), canvas.height() / (LINES * 10)));

    canvas.fillScreen(TFT_BLACK);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(ts);
    canvas.setCursor(0, 0);
    print_accel("Mini", mini.latest(), mini_param);  // accel (g)
    print_accel("DoF6", dof6.latest(), dof6_param);
    print_accel("DoF9", dof9.imu().latest(), dof9_param);
    print_mag("DoF9 M", dof9.mag().latest());  // magnetic field (uT)
    print_accel("DoF10", dof10.imu().latest(), dof10_param);
    print_mag("DoF10M", dof10.mag().latest());
    canvas.printf("%-6s %.1fhPa\n", "DoF10P", dof10.baro().pressure() / 100.0f);  // baro().pressure() is Pa
    print_mag("BMM350", bmm350.latest());
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

    if (!hub.add(mini, 0) ||    // PaHub ch:0 -> UnitMiniIMU
        !hub.add(dof6, 1) ||    // PaHub ch:1 -> UnitDoF6
        !hub.add(dof9, 2) ||    // PaHub ch:2 -> UnitDoF9
        !hub.add(dof10, 3) ||   // PaHub ch:3 -> UnitDoF10
        !hub.add(bmm350, 4)) {  // PaHub ch:4 -> UnitBMM350
        M5_LOGE("Failed to connect units");
        m5::unit::wiring::failStop();
    }
    // Run the PaHub at the lowest clock of the units behind it
    auto hcfg = hub.component_config();
    hcfg.clock =
        std::min({hcfg.clock, mini.component_config().clock, dof6.component_config().clock,
                  dof9.component_config().clock, dof10.component_config().clock, bmm350.component_config().clock});
    hub.component_config(hcfg);

    // Classic M5Stack Core -> GROVE Port C (keeps the PaHub off the built-in IMU's 0x68 bus);
    // other boards -> addI2C(): NessoN1 -> SoftwareI2C (M5HAL), NanoC6 / NanoH2 -> M5.Ex_I2C, others -> Wire
    const bool connected = (M5.getBoard() == m5::board_t::board_M5Stack) ? m5::unit::imu::wiring::i2cPortC(Units, hub)
                                                                         : m5::unit::wiring::addI2C(Units, hub);
    if (!connected || !Units.begin()) {
        M5_LOGE("Failed to begin");
        M5_LOGW("%s", Units.debugInfo().c_str());
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

    mini.imuConvertParam(mini_param);
    dof6.imuConvertParam(dof6_param);
    dof9.imu().imuConvertParam(dof9_param);
    dof10.imu().imuConvertParam(dof10_param);
}

void loop()
{
    M5.update();
    Units.update();

    if (mini.updated()) {
        log_accel("MiniIMU", mini.latest(), mini_param);
    }
    if (dof6.updated()) {
        log_accel("DoF6", dof6.latest(), dof6_param);
    }
    // UnitDoF9 / UnitDoF10 are composites with no measurement of their own; gate on their sub-units
    if (dof9.imu().updated()) {
        log_accel("DoF9", dof9.imu().latest(), dof9_param);
    }
    if (dof9.mag().updated()) {
        log_mag("DoF9", dof9.mag().latest());
    }
    if (dof10.imu().updated()) {
        log_accel("DoF10", dof10.imu().latest(), dof10_param);
    }
    if (dof10.mag().updated()) {
        log_mag("DoF10", dof10.mag().latest());
    }
    if (dof10.baro().updated()) {
        M5.Log.printf(">DoF10_press:%.2f\n", dof10.baro().pressure() / 100.0f);  // baro().pressure() is Pa
    }
    if (bmm350.updated()) {
        log_mag("BMM350", bmm350.latest());
    }

    if (has_lcd) {
        const auto now = m5::utility::millis();
        if (now - last_lcd_ms >= LCD_INTERVAL_MS) {
            last_lcd_ms = now;
            draw_lcd();
        }
    }
}

#if !defined(ARDUINO)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#if CONFIG_FREERTOS_UNICORE
static inline void feedIdleTaskPeriodically(void)
{
    constexpr uint32_t FEED_INTERVAL_MS{2000};
    constexpr TickType_t FEED_SLEEP_TICKS{pdMS_TO_TICKS(5)};
    static uint32_t s_last_feed_ms{};
    const uint32_t now_ms{static_cast<uint32_t>(esp_timer_get_time() / 1000)};
    if (now_ms - s_last_feed_ms >= FEED_INTERVAL_MS) {
        s_last_feed_ms = now_ms;
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
