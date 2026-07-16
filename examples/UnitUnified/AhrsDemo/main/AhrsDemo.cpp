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
   - Boards with WiFi also bring up a SoftAP "AhrsDemo" (password "ahrsdemo123"); browse to
     http://192.168.4.1 for the same gizmo live in a browser (works with or without an LCD)
  Controls:
   - BtnA click (short press): switch AHRS filter
   - BtnA double-click: zero the displayed yaw at the current heading (tare; roll/pitch unaffected,
     the filter's own internal heading is untouched)
   - BtnA hold (long press): toggle magnetometer auto-calibration (sphere-fit hard-iron) ON/OFF
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedIMU.h>
#include <math/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)

// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_MINI_IMU_WITH_BMM350) && !defined(USING_BUILTIN_IMU) && \
    !defined(USING_UNIT_DOF6) && !defined(USING_UNIT_DOF9) && !defined(USING_UNIT_DOF10)
// For UnitMiniIMU (SKU: TBD) - 6-axis, GROVE
#define USING_UNIT_MINI_IMU
// For UnitMiniIMU + UnitBMM350 (U231) - 9-axis on GROVE (0x68 + 0x14 coexist)
// #define USING_UNIT_MINI_IMU_WITH_BMM350
// For built-in IMU (Core2 / CoreS3 / AtomS3R / M5Stack Core/Fire)
// #define USING_BUILTIN_IMU
// For UnitDoF6 (U228) -- 6-axis BMI270
// #define USING_UNIT_DOF6
// For UnitDoF9 (U229) -- 9-axis BMI270 + BMM350
// #define USING_UNIT_DOF9
// For UnitDoF10 (U230) -- 10-axis BMI270 + BMM350 + SPA06 barometer
// #define USING_UNIT_DOF10
#endif

// Unit DoF BMI270 I2C address, selected by the board's SW1 switch (0x68 default / 0x69). Set this to
// match your switch -- ArduinoIDE: edit below; PlatformIO: add -D DOF_BMI270_ADDRESS=0x69 to build_flags.
#ifndef DOF_BMI270_ADDRESS
#define DOF_BMI270_ADDRESS 0x68
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
#else
#error Please choose unit!
#endif

M5Canvas text_canvas(&lcd);   // 1-bit header strip (text is white-on-black only)
M5Canvas gizmo_canvas(&lcd);  // 2-bit palette square for the colored 3D axis gizmo
constexpr int32_t HEADER_H{50};  // covers the 4 header text lines (y = 2/14/26/38)

// gizmo_canvas palette indices (a palette sprite interprets color arguments as indices)
constexpr uint8_t PAL_BLACK{0};
constexpr uint8_t PAL_RED{1};
constexpr uint8_t PAL_GREEN{2};
constexpr uint8_t PAL_BLUE{3};

bool has_lcd{};
m5::imu::FilterType current_filter{m5::imu::FilterType::Madgwick};
bool auto_cal_enabled{};  // magnetometer auto-calibration state (BtnA hold); default OFF
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

// -------- Diagnostic snapshot for the web viewer's /info panel (populated during setup) --------
// Lets you compare, from a browser, which board/chip was detected and exactly which axis remap is
// being applied -- handy when checking a board's orientation against docs/imu_orientation.
struct DiagInfo {
    m5::board_t board{};                             //!< M5.getBoard()
    bool builtin{false};                             //!< true: built-in IMU; false: external Unit
    m5::unit::imu::ImuKind imu{m5::unit::imu::ImuKind::None};  //!< probed accel/gyro chip (built-in)
    uint8_t imu_addr{0x68};                          //!< accel/gyro I2C address
    const char* unit_label{"?"};                     //!< source label when !builtin
    const char* mag_label{"none"};                   //!< magnetometer description
    m5::imu::IMUDataSource* accel_src{nullptr};      //!< source whose Accel/Gyro axis order to show
    m5::imu::IMUDataSource* mag_src{nullptr};        //!< source whose Mag axis order to show (or null)
};
DiagInfo diag;

//! @brief Non-null once IMU setup has failed; short, display-friendly reason why the accel/gyro is
//! unavailable (built-in absent / unsupported generation, or an external unit not responding).
//! setup()/loop() use it to skip failStop() and show a diagnostic notice (Serial + LCD + web /info)
//! instead of halting on a red screen.
const char* fail_reason{nullptr};

//! @brief Human-readable name for every m5::board_t (mirrors M5GFX lgfx/boards.hpp). Deprecated
//! aliases (board_M5Atom, board_M5AtomEcho, board_M5AtomEchoS3R) share a value with their
//! canonical entry and so are covered by it. Used by the diagnostic notice (every build) and the
//! web /info panel.
const char* board_name(const m5::board_t b)
{
    using B = m5::board_t;
    switch (b) {
        // --- boards with a built-in display ---
        case B::board_M5Stack:         return "M5Stack (Basic/Gray/Fire)";
        case B::board_M5StackCore2:    return "Core2";
        case B::board_M5StickC:        return "StickC";
        case B::board_M5StickCPlus:    return "StickC Plus";
        case B::board_M5StickCPlus2:   return "StickC Plus2";
        case B::board_M5StackCoreInk:  return "Core Ink";
        case B::board_M5Paper:         return "Paper";
        case B::board_M5Tough:         return "Tough";
        case B::board_M5Station:       return "Station";
        case B::board_M5StackCoreS3:   return "CoreS3";
        case B::board_M5AtomS3:        return "AtomS3";
        case B::board_M5Dial:          return "Dial";
        case B::board_M5DinMeter:      return "DinMeter";
        case B::board_M5Cardputer:     return "Cardputer";
        case B::board_M5AirQ:          return "AirQ";
        case B::board_M5VAMeter:       return "VAMeter";
        case B::board_M5StackCoreS3SE: return "CoreS3 SE";
        case B::board_M5AtomS3R:       return "AtomS3R";
        case B::board_M5PaperS3:       return "PaperS3";
        case B::board_M5CoreMP135:     return "CoreMP135";
        case B::board_M5StampPLC:      return "StampPLC";
        case B::board_M5Tab5:          return "Tab5";
        case B::board_ArduinoNessoN1:  return "Nesso N1";
        case B::board_M5CardputerADV:  return "Cardputer ADV";
        case B::board_M5UnitC6L:       return "UnitC6L";
        case B::board_M5StickS3:       return "StickS3";
        case B::board_M5StackChan:     return "StackChan";
        case B::board_M5PaperColor:    return "Paper Color";
        case B::board_M5PaperMono:     return "Paper Mono";
        case B::board_M5StopWatch:     return "StopWatch";
        // --- boards without a display ---
        case B::board_M5AtomLite:      return "ATOM Lite";
        case B::board_M5AtomPsram:     return "ATOM PSRAM";
        case B::board_M5AtomU:         return "ATOM U";
        case B::board_M5Camera:        return "Camera";
        case B::board_M5TimerCam:      return "TimerCam";
        case B::board_M5StampPico:     return "StampPico";
        case B::board_M5StampC3:       return "StampC3";
        case B::board_M5StampC3U:      return "StampC3U";
        case B::board_M5StampS3:       return "StampS3";
        case B::board_M5AtomS3Lite:    return "AtomS3 Lite";
        case B::board_M5AtomS3U:       return "AtomS3 U";
        case B::board_M5Capsule:       return "Capsule";
        case B::board_M5NanoC6:        return "NanoC6";
        case B::board_M5AtomMatrix:    return "ATOM Matrix";
        case B::board_M5AtomVoice:     return "ATOM Echo/Voice";
        case B::board_M5AtomS3RExt:    return "AtomS3R Ext";
        case B::board_M5AtomS3RCam:    return "AtomS3R Cam";
        case B::board_M5AtomVoiceS3R:  return "Atom Echo/Voice S3R";
        case B::board_M5PowerHub:      return "PowerHub";
        case B::board_M5DualKey:       return "DualKey";
        case B::board_M5UnitPoEP4:     return "UnitPoE P4";
        case B::board_M5StampS3Bat:    return "StampS3 Bat";
        case B::board_M5StampP4:       return "StampP4";
        case B::board_M5NanoH2:        return "NanoH2";
        // --- external displays ---
        case B::board_M5AtomDisplay:   return "AtomDisplay";
        case B::board_M5UnitLCD:       return "UnitLCD";
        case B::board_M5UnitOLED:      return "UnitOLED";
        case B::board_M5UnitMiniOLED:  return "UnitMiniOLED";
        case B::board_M5UnitGLASS:     return "UnitGLASS";
        case B::board_M5UnitGLASS2:    return "UnitGLASS2";
        case B::board_M5UnitRCA:       return "UnitRCA";
        case B::board_M5ModuleDisplay: return "ModuleDisplay";
        case B::board_M5ModuleRCA:     return "ModuleRCA";
        case B::board_M5UnitPoEP4HDMI: return "UnitPoE P4 HDMI";
        case B::board_FrameBuffer:     return "FrameBuffer";
        case B::board_unknown:
        default:                       return "unknown";
    }
}

#if __has_include(<WiFi.h>)  // used only by the web /info panel
const char* imu_kind_name(const m5::unit::imu::ImuKind k)
{
    using K = m5::unit::imu::ImuKind;
    switch (k) {
        case K::MPU6050: return "MPU6050";
        case K::MPU6886: return "MPU6886";
        case K::MPU9250: return "MPU9250";
        case K::BMI270:  return "BMI270";
        case K::SH200Q:  return "SH200Q";
        case K::None:
        default:         return "none";
    }
}

//! @brief Format an AxisOrder as e.g. "XYZ inv:XZ" / "YXZ inv:-" for the /info panel
void axis_order_str(const m5::imu::AxisOrder& o, char* const buf, const size_t n)
{
    static const char* const PERM[] = {"XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"};
    char inv[4] = {};
    size_t k    = 0;
    if (o.invert_x) {
        inv[k++] = 'X';
    }
    if (o.invert_y) {
        inv[k++] = 'Y';
    }
    if (o.invert_z) {
        inv[k++] = 'Z';
    }
    inv[k] = '\0';
    std::snprintf(buf, n, "%s inv:%s", PERM[static_cast<uint8_t>(o.permutation)], k ? inv : "-");
}
#endif  // __has_include(<WiFi.h>)

#if defined(USING_BUILTIN_IMU)
//! @brief Register a built-in MPU6886/MPU6050 with ahrs, applying this board's axis correction
//! @details The accel/gyro chip has already been identified as an MPU6886-family part by
//! begin_builtin(). M5.getBoard() then selects the axis-remap preset, since the mounting
//! orientation is a board property (see src/imu/presets.hpp).
bool begin_builtin_mpu6886(const m5::board_t board)
{
    if (!Units.add(mpu6886, M5.In_I2C)) {
        return false;
    }
    diag.accel_src = &mpu6886;

    // board_M5Stack (classic Core/Fire) is the only built-in config with a *separate* discrete
    // magnetometer: Fire has a BMM150, plain Core does not. Probe for it READ-ONLY here and add it
    // up front so Units.begin() runs exactly once. (Calling Units.begin() a second time re-begins
    // the already-running mpu6886; UnitMPU6886::begin() returns false when it is already periodic
    // and its DEVICE_RESET stops measurement -- so a second begin silently kills the gyro clock and
    // the AHRS filter never updates.)
    const bool has_bmm150 =
        (board == m5::board_t::board_M5Stack) &&
        (m5::unit::imu::detectBuiltinMag([](uint8_t a, uint8_t reg, uint8_t& out) {
             return M5.In_I2C.readRegister(a, reg, &out, 1, 400000);
         }) == m5::unit::imu::MagKind::BMM150);
    if (has_bmm150) {
        Units.add(bmm150, M5.In_I2C);
    }
    if (!Units.begin()) {
        return false;
    }

    switch (board) {
        case m5::board_t::board_M5StackCore2:
            return m5::imu::presets::core2(ahrs, mpu6886);

        case m5::board_t::board_M5AtomMatrix:
            return m5::imu::presets::atomMatrix(ahrs, mpu6886);

        case m5::board_t::board_M5Stack:
            if (has_bmm150) {
                diag.mag_src   = &bmm150;
                diag.mag_label = "BMM150";
                return m5::imu::presets::m5stackCoreFire(ahrs, mpu6886, bmm150);
            }
            // Plain Core (no discrete BMM150): identity is already correct.
            return ahrs.add(mpu6886);

        default:
            // AtomS3 / StickCPlus / StickCPlus2 and any other MPU6886 host: M5Unified applies no
            // axis correction (identity), same as Core2.
            return ahrs.add(mpu6886);
    }
}

//! @brief Register a built-in BMI270 with ahrs, applying this board's axis correction
//! @details The BMI270 sits at a board-dependent address (0x68 or 0x69); it is probed and the unit
//! is heap-constructed with the result (ahrs/Units keep it alive for the program's lifetime -- an
//! example singleton, intentionally never deleted). M5.getBoard() selects the axis-remap preset.
bool begin_builtin_bmi270(const m5::board_t board)
{
    uint8_t bmi_addr{0x68};
    m5::unit::bmi270::detectAddress(
        [](uint8_t a, uint8_t reg, uint8_t& out) { return M5.In_I2C.readRegister(a, reg, &out, 1, 400000); },
        bmi_addr);
    auto* bmi270 = new m5::unit::UnitBMI270(bmi_addr);
    if (!Units.add(*bmi270, M5.In_I2C) || !Units.begin()) {
        return false;
    }
    diag.accel_src = bmi270;
    diag.imu_addr  = bmi_addr;
    switch (board) {
        case m5::board_t::board_M5StackCoreS3:
        case m5::board_t::board_M5StackCoreS3SE:
        case m5::board_t::board_M5StackChan:
            return m5::imu::presets::coreS3(ahrs, *bmi270);

        case m5::board_t::board_M5AtomS3R:
        case m5::board_t::board_M5AtomS3RCam:
        case m5::board_t::board_M5AtomS3RExt:
            return m5::imu::presets::atomS3R(ahrs, *bmi270);

        case m5::board_t::board_M5AtomMatrix:
            // ATOM Matrix v1.1 replaced the MPU6886 with a BMI270 at the same orientation.
            return m5::imu::presets::atomMatrix(ahrs, *bmi270);

        default:
            // Core2 v1.3 / StickS3 / PaperS3 and other BMI270 hosts: M5Unified applies no axis
            // correction (identity).
            return ahrs.add(*bmi270);
    }
}

//! @brief Register this board's built-in IMU with ahrs, choosing the driver by probing the bus
//! @details M5.getBoard() cannot distinguish IMU generations (Core2 v1.1 MPU6886 vs v1.3 BMI270,
//! ATOM Matrix MPU6886 vs v1.1 BMI270 -- both report the same board id), so the accel/gyro chip is
//! detected on the internal bus (m5::unit::imu::detectBuiltinImu) to pick which driver to
//! construct, while M5.getBoard() still selects the axis-correction preset (the mounting
//! orientation is a board property). Mirrors M5Unified's IMU_Class::begin() chip switch.
bool begin_builtin()
{
    const auto board = M5.getBoard();
    const auto kind  = m5::unit::imu::detectBuiltinImu(
        [](uint8_t a, uint8_t reg, uint8_t& out) { return M5.In_I2C.readRegister(a, reg, &out, 1, 400000); });

    diag.board = board;
    diag.imu   = kind;

    bool ok{};
    switch (kind) {
        case m5::unit::imu::ImuKind::MPU6886:
        case m5::unit::imu::ImuKind::MPU6050:
            diag.builtin = true;
            ok           = begin_builtin_mpu6886(board);
            break;

        case m5::unit::imu::ImuKind::BMI270:
            diag.builtin = true;
            ok           = begin_builtin_bmi270(board);
            break;

        case m5::unit::imu::ImuKind::MPU9250:
            M5_LOGE("Detected an MPU9250 (original 9-axis generation). Its accel/gyro driver is not "
                    "yet implemented in M5Unit-IMU; cannot start the built-in IMU.");
            fail_reason     = "MPU9250 detected: driver not supported";
            diag.unit_label = fail_reason;  // /info "source" reports the reason
            return false;

        case m5::unit::imu::ImuKind::SH200Q:
            M5_LOGE("Detected an SH200Q (2019 generation). Its accel/gyro driver is not yet "
                    "implemented in M5Unit-IMU; cannot start the built-in IMU.");
            fail_reason     = "SH200Q detected: driver not supported";
            diag.unit_label = fail_reason;  // /info "source" reports the reason
            return false;

        case m5::unit::imu::ImuKind::None:
        default:
            M5_LOGE("No supported built-in accel/gyro IMU detected on the internal bus.");
            fail_reason     = "No built-in IMU detected";
            diag.unit_label = fail_reason;  // /info "source" reports the reason
            return false;
    }

    // A BMI270 with an AUX BMM150 carries Mag on the accel source itself; surface it for /info.
    if (ok && diag.accel_src && !diag.mag_src &&
        m5::imu::any(diag.accel_src->imuSpec() & m5::imu::Capability::Mag)) {
        diag.mag_src   = diag.accel_src;
        diag.mag_label = "BMM150 (AUX)";
    }
    if (!ok && !fail_reason) {
        // Kind was MPU6886/MPU6050/BMI270 (recognized chip), but begin_builtin_mpu6886()/
        // begin_builtin_bmi270() itself failed (e.g. a bus error) -- distinct from "no supported
        // chip" above.
        fail_reason     = "Built-in IMU begin failed";
        diag.unit_label = fail_reason;  // /info "source" reports the reason
    }
    return ok;
}
#endif

//! @brief One body axis to project: unit vector, gizmo_canvas palette index, label
struct GizmoAxis {
    m5::math::Vector3 body;
    uint8_t color;
    char label;
};

//! @brief A GizmoAxis after rotation + orthographic projection to screen space
struct ProjectedAxis {
    int16_t x;
    int16_t y;
    float depth;
    uint8_t color;
    char label;
};

//! @brief Draw the filter name / roll-pitch-yaw header plus the rotating 3D axis gizmo
void draw_gizmo(const float roll, const float pitch, const float yaw)
{
    const bool compact = text_canvas.width() < 160;  // small panels (e.g. AtomS3 128x128) need a tighter header

    float qw{1.0f}, qx{}, qy{}, qz{};
    ahrs.quaternion(qw, qx, qy, qz);
    const m5::math::Quaternion q(qw, qx, qy, qz);

    text_canvas.fillScreen(TFT_BLACK);
    text_canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    text_canvas.setCursor(4, 2);
    if (compact) {
        text_canvas.printf("%s %s", filter_name(current_filter), auto_cal_enabled ? "Cal:ON" : "Cal:--");
    } else {
        text_canvas.printf("Filter: %s", filter_name(current_filter));
    }
    text_canvas.setCursor(4, 14);
    if (compact) {
        text_canvas.printf("R:%-4.0f P:%-4.0f Y:%-4.0f", roll, pitch, yaw);
    } else {
        text_canvas.printf("R:%7.2f P:%7.2f Y:%7.2f", roll, pitch, yaw);
    }
    text_canvas.setCursor(4, 26);
    if (compact) {
        text_canvas.printf("T:%.1fC", ahrs.latest().temp);
    } else {
        text_canvas.printf("Temp:%6.2f C %s", ahrs.latest().temp, auto_cal_enabled ? "Cal:ON" : "Cal:--");
    }
#if defined(USING_UNIT_DOF10)
    text_canvas.setCursor(4, 38);
    if (compact) {
        text_canvas.printf("P:%.0f A:%.0fm", unit.baro().pressure(), unit.baro().altitude());
    } else {
        text_canvas.printf("P:%.1fhPa Alt:%.1fm", unit.baro().pressure(), unit.baro().altitude());
    }
#endif
    text_canvas.pushSprite(0, 0);

    const int w       = gizmo_canvas.width();
    const int h       = gizmo_canvas.height();
    const int cx      = w / 2;
    const int cy      = h / 2;
    const float scale = (static_cast<float>(std::min(w, h)) * 0.5f) - 12.0f;  // leave margin for labels

    const GizmoAxis axes[3] = {
        {{1.0f, 0.0f, 0.0f}, PAL_RED, 'X'},
        {{0.0f, 1.0f, 0.0f}, PAL_GREEN, 'Y'},
        {{0.0f, 0.0f, 1.0f}, PAL_BLUE, 'Z'},
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

    gizmo_canvas.fillScreen(PAL_BLACK);
    for (const auto& p : proj) {
        if (p.depth < 0.0f) {
            // Away-facing axis: thin single-pixel line.
            gizmo_canvas.drawLine(cx, cy, p.x, p.y, p.color);
        } else {
            // Near-facing axis: approximate a ~3px-wide line with three parallel 1px lines offset
            // along the line's perpendicular. gizmo_canvas.drawWideLine() is NOT usable here: on a
            // palette-depth sprite it unconditionally runs its color argument through
            // convert_to_rgb888() before drawing (LGFXBase::drawWideLine), which re-expands a raw
            // palette index as if it were an RGB332/565 color and then collapses it back to the
            // wrong palette entry (e.g. PAL_RED/PAL_GREEN both alias to PAL_BLACK). drawLine() has no
            // such pre-conversion and passes the index straight through, so it stays correct.
            const float dx   = static_cast<float>(p.x - cx);
            const float dy   = static_cast<float>(p.y - cy);
            const float len  = std::sqrt(dx * dx + dy * dy);
            const float nx   = (len > 0.001f) ? -dy / len : 1.0f;
            const float ny   = (len > 0.001f) ? dx / len : 0.0f;
            const int32_t ox = static_cast<int32_t>(std::lround(nx));
            const int32_t oy = static_cast<int32_t>(std::lround(ny));
            gizmo_canvas.drawLine(cx, cy, p.x, p.y, p.color);
            gizmo_canvas.drawLine(cx + ox, cy + oy, p.x + ox, p.y + oy, p.color);
            gizmo_canvas.drawLine(cx - ox, cy - oy, p.x - ox, p.y - oy, p.color);
        }
        gizmo_canvas.setTextColor(p.color, PAL_BLACK);
        gizmo_canvas.setCursor(p.x + 2, p.y - 4);
        gizmo_canvas.print(p.label);
    }
    gizmo_canvas.pushSprite((lcd.width() - w) / 2, HEADER_H);
}

}  // namespace

// -------- Optional WiFi SoftAP + browser viewer (any board that has WiFi) --------
// Brings up a SoftAP "AhrsDemo" (password "ahrsdemo123") and serves a self-contained page at
// http://192.168.4.1 that polls the AhrsController quaternion and renders a live 3D body-axis
// gizmo (X=red / Y=green / Z=blue). Useful on headless boards, but runs on LCD boards too.
// ESP32-H2 (NanoH2) ships no WiFi library, so <WiFi.h> is absent and the whole block compiles out
// there (mirrors M5Unified's own examples, e.g. Basic/Rtc). ESP32 classic (AtomMatrix) and every
// other WiFi-capable target keep it -- unlike CONFIG_SOC_WIFI_SUPPORTED, which is not defined on
// some Arduino cores (ESP32 classic) and so wrongly compiled the viewer out there.
#if __has_include(<WiFi.h>)
#define AHRS_WEB_VIEWER 1
#include <WiFi.h>
#include <WebServer.h>

namespace {
WebServer web_server(80);

const char AHRS_INDEX_HTML[] PROGMEM =
    R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>AhrsDemo</title><style>
body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
#gz{margin:24px auto;display:block}
#rpy{font-size:15px;letter-spacing:1px}
#inf{font-size:12px;line-height:1.7;color:#8cf;max-width:300px;margin:12px auto;text-align:left}
#inf b{color:#eee}</style></head><body><h3>AhrsDemo</h3>
<canvas id="gz" width="220" height="220"></canvas>
<div id="rpy">--</div><div id="inf"></div><script>
var cv=document.getElementById('gz'),ctx=cv.getContext('2d'),r=document.getElementById('rpy'),d=57.29578;
ctx.font='bold 14px sans-serif';ctx.textBaseline='middle';
// Rotate vector (x,y,z) by quaternion q -- the same active rotation the on-device gizmo uses.
function rv(q,x,y,z){var w=q.w,a=q.x,b=q.y,c=q.z,
tx=2*(b*z-c*y),ty=2*(c*x-a*z),tz=2*(a*y-b*x);
return[x+w*tx+(b*tz-c*ty),y+w*ty+(c*tx-a*tz),z+w*tz+(a*ty-b*tx)];}
// Same projection as draw_gizmo(): screen x=cx+rx*s, y=cy-ry*s (Y up), depth=rz; far axes dimmed.
function draw(q){var W=cv.width,H=cv.height,cx=W/2,cy=H/2,s=W/2-20;
ctx.clearRect(0,0,W,H);
var A=[[1,0,0,'#f44','X'],[0,1,0,'#4f4','Y'],[0,0,1,'#48f','Z']];
var p=A.map(function(a){var t=rv(q,a[0],a[1],a[2]);return{x:cx+t[0]*s,y:cy-t[1]*s,z:t[2],c:a[3],l:a[4]};});
p.sort(function(m,n){return m.z-n.z;});
ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(cx,cy,3,0,7);ctx.fill();
ctx.lineWidth=4;ctx.lineCap='round';
p.forEach(function(o){ctx.globalAlpha=o.z<0?0.4:1;ctx.strokeStyle=o.c;
ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(o.x,o.y);ctx.stroke();
ctx.fillStyle=o.c;ctx.fillText(o.l,o.x+4,o.y-4);ctx.globalAlpha=1;});}
function tick(){fetch('/q').then(function(res){return res.json();}).then(function(q){
draw(q);
var rr=Math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))*d;
var pp=Math.asin(Math.max(-1,Math.min(1,2*(q.w*q.y-q.z*q.x))))*d;
var yy=Math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))*d;
r.textContent='['+q.filter+']  roll '+rr.toFixed(1)+'  pitch '+pp.toFixed(1)+'  yaw '+yy.toFixed(1)+'  temp '+q.temp.toFixed(1)+'C'+(q.cal?'  Cal ON':'')+(q.press!==undefined?('  P '+q.press.toFixed(1)+'hPa Alt '+q.alt.toFixed(1)+'m'):'');
}).catch(function(){}).then(function(){setTimeout(tick,50);});}
tick();
fetch('/info').then(function(res){return res.json();}).then(function(o){
document.getElementById('inf').innerHTML=
'<b>board:</b> '+o.board+'<br><b>source:</b> '+o.source+'<br><b>mag:</b> '+o.mag+
'<br><b>accel/gyro axis:</b> '+o.accel_axis+'<br><b>mag axis:</b> '+o.mag_axis;}).catch(function(){});
</script></body></html>)HTML";

void handle_root()
{
    web_server.send_P(200, "text/html", AHRS_INDEX_HTML);
}

void handle_quaternion()
{
    float w{1.0f}, x{}, y{}, z{};
    ahrs.quaternion(w, x, y, z);
#if defined(USING_UNIT_DOF10)
    char buf[220];
    std::snprintf(buf, sizeof(buf),
                  "{\"w\":%.4f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"filter\":\"%s\",\"temp\":%.1f,\"press\":%.1f,"
                  "\"alt\":%.1f,\"cal\":%s}",
                  w, x, y, z, filter_name(current_filter), ahrs.latest().temp, unit.baro().pressure(),
                  unit.baro().altitude(), auto_cal_enabled ? "true" : "false");
#else
    char buf[180];
    std::snprintf(buf, sizeof(buf),
                  "{\"w\":%.4f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"filter\":\"%s\",\"temp\":%.1f,\"cal\":%s}", w, x,
                  y, z, filter_name(current_filter), ahrs.latest().temp, auto_cal_enabled ? "true" : "false");
#endif
    web_server.send(200, "application/json", buf);
}

void handle_info()
{
    m5::imu::AxisOrder ao{}, mo{};
    char aostr[24] = "-", mostr[24] = "-";
    if (diag.accel_src && ahrs.readAxisOrder(*diag.accel_src, ao)) {
        axis_order_str(ao, aostr, sizeof(aostr));
    }
    if (diag.mag_src && ahrs.readMagAxisOrder(*diag.mag_src, mo)) {
        axis_order_str(mo, mostr, sizeof(mostr));
    }
    char source[56];
    if (diag.builtin) {
        std::snprintf(source, sizeof(source), "Built-in %s @0x%02X", imu_kind_name(diag.imu), diag.imu_addr);
    } else {
        std::snprintf(source, sizeof(source), "%s", diag.unit_label);
    }
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "{\"board\":\"%s (0x%02X)\",\"source\":\"%s\",\"mag\":\"%s\",\"accel_axis\":\"%s\","
                  "\"mag_axis\":\"%s\"}",
                  board_name(diag.board), static_cast<int>(diag.board), source, diag.mag_label, aostr, mostr);
    web_server.send(200, "application/json", buf);
}

void start_web_viewer()
{
#if defined(CONFIG_IDF_TARGET_ESP32P4)
    // Tab5 (ESP32-P4) has no on-die radio; route WiFi to its ESP32-C6 co-processor over SDIO
    // (pins per M5Stack's Tab5 WiFi doc / M5Unified's Basic/Rtc example).
    if (M5.getBoard() == m5::board_t::board_M5Tab5) {
        WiFi.setPins(GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_11, GPIO_NUM_10, GPIO_NUM_9, GPIO_NUM_8, GPIO_NUM_15);
    }
#endif
    WiFi.mode(WIFI_AP);
    WiFi.softAP("AhrsDemo", "ahrsdemo123");
    web_server.on("/", handle_root);
    web_server.on("/q", handle_quaternion);
    web_server.on("/info", handle_info);
    web_server.begin();
    M5.Log.printf("Web viewer: join WiFi \"AhrsDemo\" (pass \"ahrsdemo123\"), open http://%s\n",
                  WiFi.softAPIP().toString().c_str());
}
}  // namespace
#endif  // __has_include(<WiFi.h>)

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
        // Two small sprites instead of one full-screen 8-bit sprite: a 1-bit header strip and a
        // 2-bit 4-color palette square for the gizmo. Even a large panel (Tab5 1280x720) then fits
        // in internal RAM (~8KB + ~112KB) next to the WiFi stack, where a full-screen 8-bit sprite
        // (921KB) would not.
        text_canvas.setColorDepth(1);
        text_canvas.setPsram(false);
        text_canvas.createSprite(lcd.width(), HEADER_H);

        const int32_t side = std::min<int32_t>(lcd.width(), lcd.height() - HEADER_H);
        gizmo_canvas.setColorDepth(2);
        gizmo_canvas.setPsram(false);
        if (!gizmo_canvas.createSprite(side, side)) {
            // Very large panel on a low-memory board: retry in PSRAM rather than showing nothing.
            gizmo_canvas.setPsram(true);
            gizmo_canvas.createSprite(side, side);
        }
        gizmo_canvas.setPaletteColor(PAL_BLACK, 0, 0, 0);
        gizmo_canvas.setPaletteColor(PAL_RED, 255, 0, 0);
        gizmo_canvas.setPaletteColor(PAL_GREEN, 0, 255, 0);
        gizmo_canvas.setPaletteColor(PAL_BLUE, 0, 0, 255);
    }

    diag.board = M5.getBoard();  // host board (begin_builtin() also fills the built-in chip fields)

    bool ok{};
#if defined(USING_UNIT_MINI_IMU)
    // UnitMiniIMU: NessoN1 -> SoftwareI2C (M5HAL), NanoC6 / NanoH2 -> M5.Ex_I2C, others -> Wire.
    // Special case: on classic M5Stack Core (board_M5Stack) GROVE Port A *is* the internal I2C bus,
    // and Gray/Fire carry a built-in MPU6886 at 0x68 -- the same address as UnitMiniIMU. Routing the
    // external unit to the default port there would collide with (or silently read) the built-in
    // IMU, so use GROVE Port C on the free Arduino Wire bus (M5Unified uses In_I2C/Ex_I2C, not Wire)
    // instead. Every other board keeps addI2C().
    if (M5.getBoard() == m5::board_t::board_M5Stack) {
        // Grove maps connector pin1 -> SCL and pin2 -> SDA (same wires as UART RX/TX). M5Unified
        // exposes Port C as port_c_pin1 (RXD) / port_c_pin2 (TXD), so SDA = pin2, SCL = pin1. The
        // unit already carries its own 400kHz clock (set in UnitMPU6886's constructor), so reuse it.
        const int sda = M5.getPin(m5::pin_name_t::port_c_pin2);
        const int scl = M5.getPin(m5::pin_name_t::port_c_pin1);
        ok = m5::unit::wiring::i2cWire(Units, unit, Wire, sda, scl, unit.component_config().clock) &&
             Units.begin() && ahrs.add(unit);
        diag.unit_label = "UnitMiniIMU (GROVE Port C @0x68)";
    } else {
        ok              = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U) && Units.begin() && ahrs.add(unit);
        diag.unit_label = "UnitMiniIMU (MPU6886 @0x68)";
    }
    diag.accel_src = &unit;
    if (!ok) {
        fail_reason     = "UnitMiniIMU not found (check GROVE cable)";
        diag.unit_label = fail_reason;  // /info "source" reports the reason
    }
#elif defined(USING_UNIT_MINI_IMU_WITH_BMM350)
    // Both units share the same GROVE bus (0x68 + 0x14 coexist, no address conflict)
    ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U) && m5::unit::wiring::addI2C(Units, mag_unit, 400 * 1000U) &&
         Units.begin() && ahrs.add(unit) && ahrs.add(mag_unit);
    diag.unit_label = "UnitMiniIMU + UnitBMM350";
    diag.accel_src  = &unit;
    diag.mag_src    = &mag_unit;
    diag.mag_label  = "BMM350";
    if (!ok) {
        fail_reason     = "UnitMiniIMU/BMM350 not found (check GROVE cable)";
        diag.unit_label = fail_reason;  // /info "source" reports the reason
    }
#elif defined(USING_UNIT_DOF6)
    // UnitDoF6 (BMI270 @0x68/0x69): same Port-C-vs-addI2C branching as UnitMiniIMU -- the DoF BMI270
    // collides with classic-Core Gray/Fire's built-in MPU6886/BMI270 at the same 0x68 address.
    if (M5.getBoard() == m5::board_t::board_M5Stack) {
        const int sda = M5.getPin(m5::pin_name_t::port_c_pin2);
        const int scl = M5.getPin(m5::pin_name_t::port_c_pin1);
        ok            = m5::unit::wiring::i2cWire(Units, unit, Wire, sda, scl, unit.component_config().clock);
    } else {
        ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U);
    }
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
    // DoF6 is a 6-axis BMI270 with no magnetometer -> skip the AUX BMM150 probe (silences a harmless
    // begin() error).
    {
        auto c              = unit.config();
        c.enable_aux_bmm150 = false;
        unit.config(c);
    }
    ok               = ok && Units.begin() && ahrs.add(unit);
    diag.unit_label  = "UnitDoF6 (BMI270)";
    diag.accel_src   = &unit;
    if (!ok) {
        fail_reason     = "UnitDoF6 not found (check GROVE cable)";
        diag.unit_label = fail_reason;  // /info "source" reports the reason
    }
#elif defined(USING_UNIT_DOF9) || defined(USING_UNIT_DOF10)
    // UnitDoF9/UnitDoF10 (BMI270 @0x68/0x69 + BMM350 @0x14[+ SPA06 @0x76]): same Port-C-vs-addI2C
    // branching as UnitMiniIMU/UnitDoF6 for the composite unit's shared bus.
    if (M5.getBoard() == m5::board_t::board_M5Stack) {
        const int sda = M5.getPin(m5::pin_name_t::port_c_pin2);
        const int scl = M5.getPin(m5::pin_name_t::port_c_pin1);
        ok            = m5::unit::wiring::i2cWire(Units, unit, Wire, sda, scl, unit.component_config().clock);
    } else {
        ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U);
    }
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
    ok = ok && Units.begin() && ahrs.add(unit.imu()) && ahrs.add(unit.mag());
#if defined(USING_UNIT_DOF10)
    diag.unit_label = "UnitDoF10 (BMI270+BMM350+SPA06)";
#else
    diag.unit_label = "UnitDoF9 (BMI270+BMM350)";
#endif
    diag.accel_src = &unit.imu();
    diag.mag_src   = &unit.mag();
    diag.mag_label = "BMM350 (0x14)";
    if (!ok) {
#if defined(USING_UNIT_DOF10)
        fail_reason = "UnitDoF10 not found (check GROVE cable)";
#else
        fail_reason = "UnitDoF9 not found (check GROVE cable)";
#endif
        diag.unit_label = fail_reason;  // /info "source" reports the reason
    }
#elif defined(USING_BUILTIN_IMU)
    ok = begin_builtin();
#endif
    if (!ok && !fail_reason) {
        // A begin path failed without recording a specific reason -- truly unexpected, but record
        // a generic one so the diagnostic mode below still engages instead of halting.
        fail_reason = "begin failed";
    }
    if (!fail_reason && !ahrs.begin()) {
        fail_reason = "begin failed";
    }
    if (fail_reason) {
        // No IMU to run: skip failStop() (a solid red screen tells the user nothing) and finish
        // setup anyway -- the boot log, the LCD notice below, and the web /info panel all carry
        // the reason instead.
        M5_LOGW("IMU unavailable: %s", fail_reason);
        M5_LOGW("%s", Units.debugInfo().c_str());
    } else {
        ahrs.selectFilter(current_filter);  // FilterType::Madgwick

        M5_LOGI("M5UnitUnified initialized");
        M5_LOGI("%s", Units.debugInfo().c_str());
        M5_LOGI("AHRS filter: %s (BtnA: switch)", filter_name(current_filter));
        M5.Log.printf("[filter]: roll,pitch,yaw\n");
    }

#if defined(AHRS_WEB_VIEWER)
    start_web_viewer();
#endif

    if (has_lcd && fail_reason) {
        // Static one-shot notice: nothing else draws to `lcd` on this path (loop() returns before
        // draw_gizmo()), so this stays on screen for the life of the program.
        lcd.fillScreen(TFT_BLACK);
        lcd.setTextColor(TFT_WHITE, TFT_BLACK);
        lcd.setTextSize(lcd.width() < 160 ? 1 : 2);
        lcd.setCursor(4, 4);
        lcd.printf("AhrsDemo\n%s\nboard: %s\n", fail_reason, board_name(diag.board));
#if defined(AHRS_WEB_VIEWER)
        lcd.printf("WiFi 'AhrsDemo' -> 192.168.4.1/info\n");
#endif
    }
}

void loop()
{
    M5.update();

    if (fail_reason) {
        // IMU unavailable: nothing to drain/fuse/draw, and BtnA is meaningless without a
        // running filter. Still service the web viewer (its /info panel is the most useful
        // diagnostic here); the static LCD notice drawn in setup() and the boot log carry the reason.
#if defined(AHRS_WEB_VIEWER)
        web_server.handleClient();
#endif
        return;
    }

    Units.update();

#if defined(AHRS_WEB_VIEWER)
    web_server.handleClient();
#endif

    const auto cap = ahrs.update();
    if (m5::imu::any(cap)) {
        float roll{}, pitch{}, yaw{};
        ahrs.euler(roll, pitch, yaw);
        M5.Log.printf("[%s]: %.2f,%.2f,%.2f  temp:%.1f\n", filter_name(current_filter), roll, pitch, yaw,
                      ahrs.latest().temp);

        if (has_lcd) {
            const auto now = m5::utility::millis();
            if (now - last_gizmo_ms >= GIZMO_INTERVAL_MS) {
                last_gizmo_ms = now;
                draw_gizmo(roll, pitch, yaw);
            }
        }
    }

    if (M5.BtnA.wasSingleClicked()) {  // short press, no follow-up click: switch filter
        switch_filter();
    }
    if (M5.BtnA.wasDoubleClicked()) {  // double click: zero the displayed yaw at the current heading
        ahrs.tareYaw();
        M5_LOGI("Yaw tared to current heading (clearYawTare()-only API restores the absolute heading)");
    }
    if (M5.BtnA.wasHold()) {  // long press: toggle mag auto-calibration (sphere-fit hard-iron)
        auto_cal_enabled = !auto_cal_enabled;
        ahrs.writeAutoCalibration(0, 0, auto_cal_enabled ? 64 : 0);
        M5_LOGI("Mag auto-calibration %s%s", auto_cal_enabled ? "ON" : "OFF",
                auto_cal_enabled ? " (wave the device in a figure-eight to converge)" : " (converged offset kept)");
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
