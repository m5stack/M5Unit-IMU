/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitIMU series with AhrsDemo
  Output:
   - Serial: "[filter]: roll,pitch,yaw" (selected filter name prefixed to each line)
   - LCD-equipped boards additionally show a rotating 3D body-axis gizmo (X=red / Y=green /
     Z=blue) driven by the same AhrsController quaternion
   - Boards with WiFi also bring up a SoftAP "AhrsDemo" (password "AhrsDemo"); browse to
     http://192.168.4.1 for the same gizmo live in a browser (works with or without an LCD)
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedIMU.h>
#include <math/quaternion.hpp>
#include <algorithm>
#include <cstdio>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)

// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_MINI_IMU_WITH_BMM350) && !defined(USING_BUILTIN_IMU)
// For UnitMiniIMU (SKU: TBD) - 6-axis, GROVE
#define USING_UNIT_MINI_IMU
// For UnitMiniIMU + UnitBMM350 (U231) - 9-axis on GROVE (0x68 + 0x14 coexist)
// #define USING_UNIT_MINI_IMU_WITH_BMM350
// For built-in IMU (Core2 / CoreS3 / AtomS3R / M5Stack Core/Fire)
// #define USING_BUILTIN_IMU
#endif

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::imu::AhrsController ahrs;

#if defined(USING_UNIT_MINI_IMU)
m5::unit::UnitMiniIMU unit{};
#elif defined(USING_UNIT_MINI_IMU_WITH_BMM350)
m5::unit::UnitMiniIMU unit{};
m5::unit::UnitBMM350 mag_unit{};
#elif defined(USING_BUILTIN_IMU)
m5::unit::UnitMPU6886 mpu6886{};
// bmi270 is constructed inside begin_builtin() once its board-dependent I2C address is probed.
m5::unit::UnitBMM150 bmm150{};
#else
#error Please choose unit!
#endif

M5Canvas canvas(&lcd);
bool has_lcd{};
m5::imu::FilterType current_filter{m5::imu::FilterType::Madgwick};
constexpr uint32_t GIZMO_INTERVAL_MS{33};  // ~30fps
uint32_t last_gizmo_ms{};

const char* filter_name(const m5::imu::FilterType type)
{
    switch (type) {
        case m5::imu::FilterType::Complementary:
            return "Complementary";
        case m5::imu::FilterType::Madgwick:
            return "Madgwick";
        case m5::imu::FilterType::Mahony:
            return "Mahony";
        case m5::imu::FilterType::EKF:
            return "EKF";
        case m5::imu::FilterType::Fusion:
            return "Fusion";
        case m5::imu::FilterType::VQF:
            return "VQF";
    }
    return "?";
}

m5::imu::FilterType next_filter(const m5::imu::FilterType type)
{
    switch (type) {
        case m5::imu::FilterType::Complementary:
            return m5::imu::FilterType::Madgwick;
        case m5::imu::FilterType::Madgwick:
            return m5::imu::FilterType::Mahony;
        case m5::imu::FilterType::Mahony:
            return m5::imu::FilterType::EKF;
        case m5::imu::FilterType::EKF:
            return m5::imu::FilterType::Fusion;
        case m5::imu::FilterType::Fusion:
            return m5::imu::FilterType::VQF;
        case m5::imu::FilterType::VQF:
            return m5::imu::FilterType::Complementary;
    }
    return m5::imu::FilterType::Complementary;
}

void switch_filter()
{
    current_filter = next_filter(current_filter);
    ahrs.selectFilter(current_filter);
    M5_LOGI("Filter switched to %s", filter_name(current_filter));
}

#if defined(USING_BUILTIN_IMU)
//! @brief Register this board's built-in IMU (probed via M5.getBoard()) with ahrs
//! @details Mirrors M5Unified's own IMU_Class::begin() board switch (see src/imu/presets.hpp).
//! Wires the relevant unit(s) onto M5.In_I2C, begins them via Units, then hands them to the
//! matching m5::imu::presets:: function for axis-remap + ahrs.add().
bool begin_builtin()
{
    const auto board = M5.getBoard();

    // The built-in BMI270 sits at a board-dependent address (0x68 or 0x69). Detect it on the
    // internal bus and heap-construct the unit with the result; ahrs/Units keep it alive for the
    // program's lifetime (this is an example singleton, so it is intentionally never deleted).
    uint8_t bmi_addr{0x68};
    m5::unit::bmi270::detectAddress(
        [](uint8_t a, uint8_t reg, uint8_t& out) { return M5.In_I2C.readRegister(a, reg, &out, 1, 400000); },
        bmi_addr);
    auto* bmi270 = new m5::unit::UnitBMI270(bmi_addr);

    switch (board) {
        case m5::board_t::board_M5StackCore2:
            if (!Units.add(mpu6886, M5.In_I2C) || !Units.begin()) {
                return false;
            }
            return m5::imu::presets::core2(ahrs, mpu6886);

        case m5::board_t::board_M5StackCoreS3:
        case m5::board_t::board_M5StackCoreS3SE:
        case m5::board_t::board_M5StackChan:
            if (!Units.add(*bmi270, M5.In_I2C) || !Units.begin()) {
                return false;
            }
            return m5::imu::presets::coreS3(ahrs, *bmi270);

        case m5::board_t::board_M5AtomS3R:
        case m5::board_t::board_M5AtomS3RCam:
        case m5::board_t::board_M5AtomS3RExt:
            if (!Units.add(*bmi270, M5.In_I2C) || !Units.begin()) {
                return false;
            }
            return m5::imu::presets::atomS3R(ahrs, *bmi270);

        case m5::board_t::board_M5AtomMatrix:
            if (!Units.add(mpu6886, M5.In_I2C) || !Units.begin()) {
                return false;
            }
            return m5::imu::presets::atomMatrix(ahrs, mpu6886);

        case m5::board_t::board_M5Stack:
            // Classic Core/Fire both report board_M5Stack; only Fire has a discrete BMM150
            // magnetometer. mpu6886 must begin first and on its own so a missing BMM150 cannot
            // mask an mpu6886 failure (Units::begin() is all-or-nothing across every added unit).
            if (!Units.add(mpu6886, M5.In_I2C) || !Units.begin()) {
                return false;
            }
            // Probe for the discrete BMM150 (Fire only). Re-running Units.begin() here also
            // re-begins mpu6886, which is harmless -- UnitMPU6886::begin() is idempotent
            // (WHO_AM_I check + register reset/reconfigure). Mirrors M5Unified's own
            // IMU_Class::begin(), which does the same optional probe-and-discard for BMM150.
            if (Units.add(bmm150, M5.In_I2C) && Units.begin()) {
                return m5::imu::presets::m5stackCoreFire(ahrs, mpu6886, bmm150);
            }
            // Plain Core (no discrete BMM150): identity transform is already correct (see
            // presets.hpp's core2()), so register mpu6886 directly.
            return ahrs.add(mpu6886);

        default:
            M5_LOGE("Unsupported board for USING_BUILTIN_IMU: 0x%02x", static_cast<int>(board));
            M5_LOGE("Supported boards: Core2 / CoreS3(SE) / StackChan / AtomS3R(Cam/Ext) /");
            M5_LOGE("                  AtomMatrix / M5Stack Core or Fire");
            return false;
    }
}
#endif

//! @brief Half-brightness an RGB565 color (used to dim the far half of a projected axis line)
uint16_t dim565(const uint16_t color)
{
    const uint16_t r = (color >> 11) & 0x1F;
    const uint16_t g = (color >> 5) & 0x3F;
    const uint16_t b = color & 0x1F;
    return static_cast<uint16_t>(((r / 2) << 11) | ((g / 2) << 5) | (b / 2));
}

//! @brief One body axis to project: unit vector, display color, label
struct GizmoAxis {
    m5::math::Vector3 body;
    uint16_t color;
    char label;
};

//! @brief A GizmoAxis after rotation + orthographic projection to screen space
struct ProjectedAxis {
    int16_t x;
    int16_t y;
    float depth;
    uint16_t color;
    char label;
};

//! @brief Draw the filter name / roll-pitch-yaw header plus the rotating 3D axis gizmo
void draw_gizmo(const float roll, const float pitch, const float yaw)
{
    float qw{1.0f}, qx{}, qy{}, qz{};
    ahrs.quaternion(qw, qx, qy, qz);
    const m5::math::Quaternion q(qw, qx, qy, qz);

    canvas.fillScreen(TFT_BLACK);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setCursor(4, 2);
    canvas.printf("Filter: %s", filter_name(current_filter));
    canvas.setCursor(4, 14);
    canvas.printf("R:%7.2f P:%7.2f Y:%7.2f", roll, pitch, yaw);

    const int w       = canvas.width();
    const int h       = canvas.height();
    const int cx      = w / 2;
    const int cy      = h / 2 + 16;
    const float scale = (static_cast<float>(std::min(w, h - 32)) * 0.5f) - 12.0f;  // leave margin for labels

    const GizmoAxis axes[3] = {
        {{1.0f, 0.0f, 0.0f}, static_cast<uint16_t>(TFT_RED), 'X'},
        {{0.0f, 1.0f, 0.0f}, static_cast<uint16_t>(TFT_GREEN), 'Y'},
        {{0.0f, 0.0f, 1.0f}, static_cast<uint16_t>(TFT_BLUE), 'Z'},
    };
    ProjectedAxis proj[3]{};
    for (size_t i = 0; i < 3; ++i) {
        const auto rotated = q.rotate(axes[i].body);
        proj[i].x          = static_cast<int16_t>(cx + rotated.x * scale);
        proj[i].y          = static_cast<int16_t>(cy - rotated.y * scale);
        proj[i].depth      = rotated.z;
        proj[i].color      = axes[i].color;
        proj[i].label      = axes[i].label;
    }
    // Farthest (most negative Z) first, so nearer axes are drawn on top.
    std::sort(std::begin(proj), std::end(proj),
              [](const ProjectedAxis& a, const ProjectedAxis& b) { return a.depth < b.depth; });

    canvas.fillCircle(cx, cy, 3, TFT_WHITE);
    for (const auto& p : proj) {
        const uint16_t color = (p.depth < 0.0f) ? dim565(p.color) : p.color;
        canvas.drawLine(cx, cy, p.x, p.y, color);
        canvas.setTextColor(color, TFT_BLACK);
        canvas.setCursor(p.x + 2, p.y - 4);
        canvas.print(p.label);
    }
    canvas.pushSprite(0, 0);
}

}  // namespace

// -------- Optional WiFi SoftAP + browser viewer (any board that has WiFi) --------
// Brings up a SoftAP "AhrsDemo" (password "AhrsDemo") and serves a self-contained page at
// http://192.168.4.1 that polls the AhrsController quaternion and renders a live 3D body-axis
// gizmo (X=red / Y=green / Z=blue). Useful on headless boards, but runs on LCD boards too.
// ESP32-H2 (NanoH2) has no WiFi radio, so the whole block compiles out there.
#if defined(CONFIG_SOC_WIFI_SUPPORTED)
#define AHRS_WEB_VIEWER 1
#include <WiFi.h>
#include <WebServer.h>

namespace {
WebServer web_server(80);

const char AHRS_INDEX_HTML[] PROGMEM =
    R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>AhrsDemo</title><style>
body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
#scene{width:220px;height:220px;margin:36px auto;perspective:600px}
#giz{width:100%;height:100%;position:relative;transform-style:preserve-3d;transition:transform .05s linear}
.ax{position:absolute;left:50%;top:50%;height:5px;width:90px;border-radius:3px;transform-origin:0 50%}
.x{background:#f44}.y{background:#4f4;transform:rotateZ(-90deg)}.z{background:#48f;transform:rotateY(90deg)}
#rpy{font-size:15px;letter-spacing:1px}</style></head><body><h3>AhrsDemo</h3>
<div id="scene"><div id="giz"><div class="ax x"></div><div class="ax y"></div><div class="ax z"></div></div></div>
<div id="rpy">--</div><script>
var g=document.getElementById('giz'),r=document.getElementById('rpy'),d=57.29578;
function mat(q){var w=q.w,x=q.x,y=q.y,z=q.z;
return'matrix3d('+(1-2*(y*y+z*z))+','+(2*(x*y+w*z))+','+(2*(x*z-w*y))+',0,'+
(2*(x*y-w*z))+','+(1-2*(x*x+z*z))+','+(2*(y*z+w*x))+',0,'+
(2*(x*z+w*y))+','+(2*(y*z-w*x))+','+(1-2*(x*x+y*y))+',0,0,0,0,1)';}
function tick(){fetch('/q').then(function(res){return res.json();}).then(function(q){
g.style.transform=mat(q);
var rr=Math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))*d;
var pp=Math.asin(Math.max(-1,Math.min(1,2*(q.w*q.y-q.z*q.x))))*d;
var yy=Math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))*d;
r.textContent='roll '+rr.toFixed(1)+'  pitch '+pp.toFixed(1)+'  yaw '+yy.toFixed(1);
}).catch(function(){}).then(function(){setTimeout(tick,50);});}
tick();</script></body></html>)HTML";

void handle_root()
{
    web_server.send_P(200, "text/html", AHRS_INDEX_HTML);
}

void handle_quaternion()
{
    float w{1.0f}, x{}, y{}, z{};
    ahrs.quaternion(w, x, y, z);
    char buf[96];
    std::snprintf(buf, sizeof(buf), "{\"w\":%.4f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f}", w, x, y, z);
    web_server.send(200, "application/json", buf);
}

void start_web_viewer()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP("AhrsDemo", "AhrsDemo");
    web_server.on("/", handle_root);
    web_server.on("/q", handle_quaternion);
    web_server.begin();
    M5.Log.printf("Web viewer: join WiFi \"AhrsDemo\" (pass \"AhrsDemo\"), open http://%s\n",
                  WiFi.softAPIP().toString().c_str());
}
}  // namespace
#endif  // CONFIG_SOC_WIFI_SUPPORTED

void setup()
{
    M5.begin();

    M5.setTouchButtonHeightByRatio(100);
    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }
    has_lcd = lcd.width() > 0;
    if (has_lcd) {
        canvas.setColorDepth(8);
        canvas.setPsram(false);
        canvas.createSprite(lcd.width(), lcd.height());
    }

    bool ok{};
#if defined(USING_UNIT_MINI_IMU)
    // UnitMiniIMU: NessoN1 -> SoftwareI2C (M5HAL), NanoC6 / NanoH2 -> M5.Ex_I2C, others -> Wire
    ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U) && Units.begin() && ahrs.add(unit);
#elif defined(USING_UNIT_MINI_IMU_WITH_BMM350)
    // Both units share the same GROVE bus (0x68 + 0x14 coexist, no address conflict)
    ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U) && m5::unit::wiring::addI2C(Units, mag_unit, 400 * 1000U) &&
         Units.begin() && ahrs.add(unit) && ahrs.add(mag_unit);
#elif defined(USING_BUILTIN_IMU)
    ok = begin_builtin();
#endif
    if (!ok || !ahrs.begin()) {
        M5_LOGE("Failed to begin");
        M5_LOGW("%s", Units.debugInfo().c_str());
        m5::unit::wiring::failStop();
    }
    ahrs.selectFilter(current_filter);  // FilterType::Madgwick

    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());
    M5_LOGI("AHRS filter: %s (BtnA: switch)", filter_name(current_filter));
    M5.Log.printf("[filter]: roll,pitch,yaw\n");

#if defined(AHRS_WEB_VIEWER)
    start_web_viewer();
#endif
}

void loop()
{
    M5.update();
    Units.update();

#if defined(AHRS_WEB_VIEWER)
    web_server.handleClient();
#endif

    const auto cap = ahrs.update();
    if (m5::imu::any(cap)) {
        float roll{}, pitch{}, yaw{};
        ahrs.euler(roll, pitch, yaw);
        M5.Log.printf("[%s]: %.2f,%.2f,%.2f\n", filter_name(current_filter), roll, pitch, yaw);

        if (has_lcd) {
            const auto now = m5::utility::millis();
            if (now - last_gizmo_ms >= GIZMO_INTERVAL_MS) {
                last_gizmo_ms = now;
                draw_gizmo(roll, pitch, yaw);
            }
        }
    }

    if (M5.BtnA.wasPressed()) {
        switch_filter();
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
