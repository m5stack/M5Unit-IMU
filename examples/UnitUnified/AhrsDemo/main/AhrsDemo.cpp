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
   - BtnB click (boards that have a BtnB): with VQF selected, toggle its magnetic disturbance
     rejection, which this library leaves off by default. Watch the MagDis field: it reads 1
     until VQF adopts a magnetic reference, which needs several seconds of sustained rotation,
     and returns to 1 whenever the field is disturbed afterwards (hold a magnet nearby to see it)
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedIMU.h>
#include <math/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)
#include <wiring/m5_unit_imu_wiring.hpp>      // This library: classic Core Port C helper

// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_MINI_IMU_WITH_BMM350) && !defined(USING_BUILTIN_IMU) && \
    !defined(USING_BUILTIN_IMU_WITH_BMM350) && !defined(USING_UNIT_DOF6) && !defined(USING_UNIT_DOF9) &&         \
    !defined(USING_UNIT_DOF10) && !defined(USING_UNIT_MINI_IMU_PRO)
// For UnitMiniIMU (SKU: TBD) - 6-axis, GROVE
#define USING_UNIT_MINI_IMU
// For UnitMiniIMU + UnitBMM350 (U231) - 9-axis on GROVE (0x68 + 0x14 coexist)
// #define USING_UNIT_MINI_IMU_WITH_BMM350
// For built-in IMU (Core2 / CoreS3 / AtomS3R / M5Stack Core/Fire)
// #define USING_BUILTIN_IMU
// For built-in IMU + UnitBMM350 (U231) on GROVE -- 9-axis for a host whose IMU has no magnetometer
// #define USING_BUILTIN_IMU_WITH_BMM350
// For UnitDoF6 (U228) -- 6-axis BMI270
// #define USING_UNIT_DOF6
// For UnitDoF9 (U229) -- 9-axis BMI270 + BMM350
// #define USING_UNIT_DOF9
// For UnitDoF10 (U230) -- 10-axis BMI270 + BMM350 + SPA06 barometer
// #define USING_UNIT_DOF10
// For UnitMiniIMUPro (U171) -- 10-axis BMI270 + BMM150 + BMP280 barometer
// #define USING_UNIT_MINI_IMU_PRO
#endif

// The BMM350 variant runs the whole built-in path unchanged and adds the GROVE magnetometer on top,
// so every USING_BUILTIN_IMU site below applies to it as well.
#if defined(USING_BUILTIN_IMU_WITH_BMM350) && !defined(USING_BUILTIN_IMU)
#define USING_BUILTIN_IMU
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
m5::unit::UnitSH200Q sh200q{};    // 2019.3 M5StickC / 2019.7 Fire (there paired with the BMM150)
m5::unit::UnitMPU6050 mpu6050{};  // 2017 Gray (paired with the MAG3110)
m5::unit::UnitMAG3110 mag3110{};  // 2017 Gray's discrete magnetometer
m5::unit::UnitMPU9250 mpu9250{};  // original Fire / 2017.12 Gray / some early Basic (internal AK8963)
#if defined(USING_BUILTIN_IMU_WITH_BMM350)
m5::unit::UnitBMM350 mag_unit{};  // GROVE Unit BMM350 (U231), 0x14 -- 9-axis for a mag-less host
#endif
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

M5Canvas text_canvas(&lcd);      // 1-bit header strip (text is white-on-black only)
M5Canvas gizmo_canvas(&lcd);     // 2-bit palette square for the colored 3D axis gizmo
constexpr int32_t HEADER_H{50};  // covers the 4 header text lines (y = 2/14/26/38)

// gizmo_canvas palette indices (a palette sprite interprets color arguments as indices)
constexpr uint8_t PAL_BLACK{0};
constexpr uint8_t PAL_RED{1};
constexpr uint8_t PAL_GREEN{2};
constexpr uint8_t PAL_BLUE{3};

bool has_lcd{};
m5::imu::FilterType current_filter{m5::imu::FilterType::Madgwick};
bool auto_cal_enabled{};                   // magnetometer auto-calibration state (BtnA hold); default OFF
constexpr uint32_t GIZMO_INTERVAL_MS{33};  // ~30fps
uint32_t last_gizmo_ms{};

// -------- Post-select settle boost for Madgwick/Mahony --------
// Both start from identity after (re)selection and pull toward the mag heading with their
// paper-default gains (beta 0.1 / Kp 1.0) -- taking tens of seconds to settle, which looks like a
// filter-dependent heading error next to the faster filters. Temporarily raising the gain right
// after selection is the initialization strategy Madgwick's own report recommends (large beta
// until converged, then the operational value) and what the Fusion library bakes in as its
// 3-second startup ramp; this mirrors that for the two filters without a built-in ramp. The
// steady-state gains stay at the paper defaults.
// Side effect: while boosted, accelerometer noise passes into the attitude far more strongly --
// roll/pitch swing about +-3 deg for the first 3 seconds even when the device sits still, and do
// so in a regular sample-to-sample alternation (a high-gain correction loop overshoots each noisy
// sample and comes back). This is the cost of fast convergence, not a sensor or driver fault.
constexpr uint32_t SETTLE_BOOST_MS{3000};
constexpr float MADGWICK_BETA_BOOST{2.5f};  // initialization value suggested in Madgwick's report
constexpr float MADGWICK_BETA_DEFAULT{0.1f};
constexpr float MAHONY_KP_BOOST{5.0f};
constexpr float MAHONY_KP_DEFAULT{1.0f};
uint32_t settle_until_ms{};  // 0: no boost active

/*!
  @brief Remove the gyroscope's static bias once, at boot, before any filter integrates it
  @details Every filter accumulates an uncorrected gyro bias straight into yaw, so a few dps of
  zero-rate offset shows up as a heading that creeps even while the device sits still. VQF can
  estimate and remove the bias by itself, but only while it detects rest -- and upstream refuses
  to call a sample "at rest" whenever a low-passed gyro axis exceeds VQFParams::biasClip (2 dps
  by default), which several M5 IMUs exceed outright (the MPU6886's zero-rate output is
  specified around +-5 dps). Calibrating here brings the residual inside that window so VQF's
  own estimator engages, and helps the five filters that have no bias estimation at all.
  @note AhrsController::calibrateGyro() consumes samples the sources have already buffered, so
  Units.update() has to run first to fill them.
  @warning Assumes the device is stationary during boot; anything else calibrates in the motion.
 */
void calibrate_gyro_at_boot()
{
    constexpr int PRIME_SAMPLES{32};  // fill the source buffers before draining them
    constexpr uint32_t PRIME_INTERVAL_MS{10};
    constexpr uint32_t CALIBRATE_DURATION_MS{50};

    for (int i = 0; i < PRIME_SAMPLES; ++i) {
        Units.update();
        m5::utility::delay(PRIME_INTERVAL_MS);
    }
    if (!ahrs.calibrateGyro(CALIBRATE_DURATION_MS)) {
        M5_LOGW("Gyro calibration skipped (no gyro source or no samples)");
    }
}

//! @brief Start the settle boost if the just-selected filter needs one (call right after selectFilter())
void begin_settle_boost()
{
    settle_until_ms = 0;
    switch (current_filter) {
        case m5::imu::FilterType::Madgwick:
            static_cast<m5::imu::MadgwickFilter*>(ahrs.filter())->writeBeta(MADGWICK_BETA_BOOST);
            settle_until_ms = m5::utility::millis() + SETTLE_BOOST_MS;
            break;
        case m5::imu::FilterType::Mahony:
            static_cast<m5::imu::MahonyFilter*>(ahrs.filter())->writeKp(MAHONY_KP_BOOST);
            settle_until_ms = m5::utility::millis() + SETTLE_BOOST_MS;
            break;
        default:
            break;  // The other filters converge fast (or ramp internally, like Fusion)
    }
}

//! @brief Restore the paper-default gain once the boost window has elapsed (call from loop())
void end_settle_boost_if_due()
{
    if (!settle_until_ms || m5::utility::millis() < settle_until_ms) {
        return;
    }
    settle_until_ms = 0;
    switch (current_filter) {
        case m5::imu::FilterType::Madgwick:
            static_cast<m5::imu::MadgwickFilter*>(ahrs.filter())->writeBeta(MADGWICK_BETA_DEFAULT);
            break;
        case m5::imu::FilterType::Mahony:
            static_cast<m5::imu::MahonyFilter*>(ahrs.filter())->writeKp(MAHONY_KP_DEFAULT);
            break;
        default:
            break;
    }
}

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
    // selectFilter() resets the new filter's attitude to identity; a yaw tare captured against
    // the OLD filter's heading would keep rotating the fresh estimate by a now-meaningless
    // offset (it looks like a filter-dependent axis shift). Drop it; double-click re-tares.
    ahrs.clearYawTare();
    begin_settle_boost();
    M5_LOGI("Filter switched to %s", filter_name(current_filter));
}

// -------- Diagnostic snapshot for the web viewer's /info panel (populated during setup) --------
// Lets you compare, from a browser, which board/chip was detected and exactly which axis remap is
// being applied -- handy when checking a board's orientation against docs/imu_orientation.
struct DiagInfo {
    m5::board_t board{};                                       //!< M5.getBoard()
    bool builtin{false};                                       //!< true: built-in IMU; false: external Unit
    m5::unit::imu::ImuKind imu{m5::unit::imu::ImuKind::None};  //!< probed accel/gyro chip (built-in)
    uint8_t imu_addr{0x68};                                    //!< accel/gyro I2C address
    const char* unit_label{"?"};                               //!< source label when !builtin
    const char* mag_label{"none"};                             //!< magnetometer description
    m5::imu::IMUDataSource* accel_src{nullptr};                //!< source whose Accel/Gyro axis order to show
    m5::imu::IMUDataSource* mag_src{nullptr};                  //!< source whose Mag axis order to show (or null)
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
        case B::board_M5Stack:
            return "M5Stack (Basic/Gray/Fire)";
        case B::board_M5StackCore2:
            return "Core2";
        case B::board_M5StickC:
            return "StickC";
        case B::board_M5StickCPlus:
            return "StickC Plus";
        case B::board_M5StickCPlus2:
            return "StickC Plus2";
        case B::board_M5StackCoreInk:
            return "Core Ink";
        case B::board_M5Paper:
            return "Paper";
        case B::board_M5Tough:
            return "Tough";
        case B::board_M5Station:
            return "Station";
        case B::board_M5StackCoreS3:
            return "CoreS3";
        case B::board_M5AtomS3:
            return "AtomS3";
        case B::board_M5Dial:
            return "Dial";
        case B::board_M5DinMeter:
            return "DinMeter";
        case B::board_M5Cardputer:
            return "Cardputer";
        case B::board_M5AirQ:
            return "AirQ";
        case B::board_M5VAMeter:
            return "VAMeter";
        case B::board_M5StackCoreS3SE:
            return "CoreS3 SE";
        case B::board_M5AtomS3R:
            return "AtomS3R";
        case B::board_M5PaperS3:
            return "PaperS3";
        case B::board_M5CoreMP135:
            return "CoreMP135";
        case B::board_M5StampPLC:
            return "StampPLC";
        case B::board_M5Tab5:
            return "Tab5";
        case B::board_ArduinoNessoN1:
            return "Nesso N1";
        case B::board_M5CardputerADV:
            return "Cardputer ADV";
        case B::board_M5UnitC6L:
            return "UnitC6L";
        case B::board_M5StickS3:
            return "StickS3";
        case B::board_M5StackChan:
            return "StackChan";
        case B::board_M5PaperColor:
            return "Paper Color";
        case B::board_M5PaperMono:
            return "Paper Mono";
        case B::board_M5StopWatch:
            return "StopWatch";
        // --- boards without a display ---
        case B::board_M5AtomLite:
            return "ATOM Lite";
        case B::board_M5AtomPsram:
            return "ATOM PSRAM";
        case B::board_M5AtomU:
            return "ATOM U";
        case B::board_M5Camera:
            return "Camera";
        case B::board_M5TimerCam:
            return "TimerCam";
        case B::board_M5StampPico:
            return "StampPico";
        case B::board_M5StampC3:
            return "StampC3";
        case B::board_M5StampC3U:
            return "StampC3U";
        case B::board_M5StampS3:
            return "StampS3";
        case B::board_M5AtomS3Lite:
            return "AtomS3 Lite";
        case B::board_M5AtomS3U:
            return "AtomS3 U";
        case B::board_M5Capsule:
            return "Capsule";
        case B::board_M5NanoC6:
            return "NanoC6";
        case B::board_M5AtomMatrix:
            return "ATOM Matrix";
        case B::board_M5AtomVoice:
            return "ATOM Echo/Voice";
        case B::board_M5AtomS3RExt:
            return "AtomS3R Ext";
        case B::board_M5AtomS3RCam:
            return "AtomS3R Cam";
        case B::board_M5AtomVoiceS3R:
            return "Atom Echo/Voice S3R";
        case B::board_M5PowerHub:
            return "PowerHub";
        case B::board_M5DualKey:
            return "DualKey";
        case B::board_M5UnitPoEP4:
            return "UnitPoE P4";
        case B::board_M5StampS3Bat:
            return "StampS3 Bat";
        case B::board_M5StampP4:
            return "StampP4";
        case B::board_M5NanoH2:
            return "NanoH2";
        // --- external displays ---
        case B::board_M5AtomDisplay:
            return "AtomDisplay";
        case B::board_M5UnitLCD:
            return "UnitLCD";
        case B::board_M5UnitOLED:
            return "UnitOLED";
        case B::board_M5UnitMiniOLED:
            return "UnitMiniOLED";
        case B::board_M5UnitGLASS:
            return "UnitGLASS";
        case B::board_M5UnitGLASS2:
            return "UnitGLASS2";
        case B::board_M5UnitRCA:
            return "UnitRCA";
        case B::board_M5ModuleDisplay:
            return "ModuleDisplay";
        case B::board_M5ModuleRCA:
            return "ModuleRCA";
        case B::board_M5UnitPoEP4HDMI:
            return "UnitPoE P4 HDMI";
        case B::board_FrameBuffer:
            return "FrameBuffer";
        case B::board_unknown:
        default:
            return "unknown";
    }
}

#if __has_include(<WiFi.h>) && !defined(CONFIG_IDF_TARGET_ESP32H2)  // used only by the web /info panel
const char* imu_kind_name(const m5::unit::imu::ImuKind k)
{
    using K = m5::unit::imu::ImuKind;
    switch (k) {
        case K::MPU6050:
            return "MPU6050";
        case K::MPU6886:
            return "MPU6886";
        case K::MPU9250:
            return "MPU9250";
        case K::BMI270:
            return "BMI270";
        case K::SH200Q:
            return "SH200Q";
        case K::None:
        default:
            return "none";
    }
}

//! @brief Format an AxisOrder as e.g. "XYZ inv:XZ" / "YXZ inv:-" for the /info panel
void axis_order_str(const m5::imu::AxisOrder& o, char* const buf, const size_t n)
{
    static const char* const PERM[] = {"XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"};
    char inv[4]                     = {};
    size_t k                        = 0;
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
#endif  // __has_include(<WiFi.h>) && !ESP32H2

#if defined(USING_BUILTIN_IMU)
//! @brief Register a built-in MPU6886 with ahrs, applying this board's axis correction
//! @details The accel/gyro chip has already been identified as an MPU6886 by begin_builtin().
//! M5.getBoard() then selects the axis-remap preset, since the mounting orientation is a board
//! property (see src/imu/presets.hpp).
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
    const bool has_bmm150 = (board == m5::board_t::board_M5Stack) &&
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
        [](uint8_t a, uint8_t reg, uint8_t& out) { return M5.In_I2C.readRegister(a, reg, &out, 1, 400000); }, bmi_addr);
    auto* bmi270 = new m5::unit::UnitBMI270AuxBMM150(bmi_addr);
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

//! @brief Register a built-in MPU6050 with ahrs (the 2017 original Gray, paired with a MAG3110)
//! @details The MPU6050 (WHO_AM_I 0x68) needs its own driver instance -- UnitMPU6886 rejects its
//! id. Its only built-in host is the 2017 Gray, where a discrete MAG3110 sits at 0x0E; it is probed
//! and added up front so Units.begin() runs exactly once (same constraint as
//! begin_builtin_mpu6886()). No axis-remap preset exists for this generation (M5Unified has no
//! reference table), so both sources stay at identity until confirmed on 2017 hardware
//! (docs/imu_orientation).
bool begin_builtin_mpu6050(const m5::board_t board)
{
    if (!Units.add(mpu6050, M5.In_I2C)) {
        return false;
    }
    diag.accel_src = &mpu6050;

    const bool has_mag3110 = (board == m5::board_t::board_M5Stack) &&
                             (m5::unit::imu::detectBuiltinMag([](uint8_t a, uint8_t reg, uint8_t& out) {
                                  return M5.In_I2C.readRegister(a, reg, &out, 1, 400000);
                              }) == m5::unit::imu::MagKind::MAG3110);
    if (has_mag3110) {
        Units.add(mag3110, M5.In_I2C);
    }
    if (!Units.begin()) {
        return false;
    }

    if (!ahrs.add(mpu6050)) {
        return false;
    }
    if (has_mag3110) {
        diag.mag_src   = &mag3110;
        diag.mag_label = "MAG3110";
        return ahrs.add(mag3110);  // identity mag remap (unverified generation)
    }
    return true;
}

//! @brief Register a built-in MPU9250 (accel/gyro + internal AK8963 mag) with ahrs
//! @details The original Fire, the 2017.12-2019.6 Gray, and some early Basic lots carry the
//! MPU9250. UnitMPU9250 is a composite: the unit itself is the accel/gyro die, its AK8963 mag die
//! is a framework child begun automatically by Units.begin(). The AK8963's YXZ + Z-inversion mag
//! remap is gated on the accel chip being an MPU9250 (mirrors M5Unified), not on the board; the
//! accel/gyro stay at identity on board_M5Stack.
bool begin_builtin_mpu9250()
{
    if (!Units.add(mpu9250, M5.In_I2C) || !Units.begin()) {
        return false;
    }
    diag.accel_src = &mpu9250;
    diag.mag_src   = &mpu9250.mag();
    diag.mag_label = "AK8963";
    return ahrs.add(mpu9250) && m5::imu::presets::mpu9250Ak8963(ahrs, mpu9250.mag());
}

//! @brief Register a built-in SH200Q with ahrs (2019.3 M5StickC, or a 2019.7 Fire with a BMM150)
//! @details No accel/gyro axis-remap preset is applied yet: field reports suggest the SH200Q's
//! accel and gyro axis systems may not agree with each other (or the datasheet), so identity is
//! used until the orientation is confirmed on hardware (docs/imu_orientation). Expect a large gyro
//! zero offset. On board_M5Stack (Fire 2019.7) the discrete BMM150 is probed and added up front so
//! Units.begin() runs exactly once (same constraint as begin_builtin_mpu6886()).
bool begin_builtin_sh200q(const m5::board_t board)
{
    if (!Units.add(sh200q, M5.In_I2C)) {
        return false;
    }
    diag.accel_src = &sh200q;
    diag.imu_addr  = 0x6C;

    const bool has_bmm150 = (board == m5::board_t::board_M5Stack) &&
                            (m5::unit::imu::detectBuiltinMag([](uint8_t a, uint8_t reg, uint8_t& out) {
                                 return M5.In_I2C.readRegister(a, reg, &out, 1, 400000);
                             }) == m5::unit::imu::MagKind::BMM150);
    if (has_bmm150) {
        Units.add(bmm150, M5.In_I2C);
    }
    if (!Units.begin()) {
        return false;
    }

    if (has_bmm150) {
        diag.mag_src   = &bmm150;
        diag.mag_label = "BMM150";
        return m5::imu::presets::m5stackCoreFire(ahrs, sh200q, bmm150);
    }
    return ahrs.add(sh200q);
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
            diag.builtin = true;
            ok           = begin_builtin_mpu6886(board);
            break;

        case m5::unit::imu::ImuKind::MPU6050:
            diag.builtin = true;
            ok           = begin_builtin_mpu6050(board);
            break;

        case m5::unit::imu::ImuKind::BMI270:
            diag.builtin = true;
            ok           = begin_builtin_bmi270(board);
            break;

        case m5::unit::imu::ImuKind::MPU9250:
            diag.builtin = true;
            ok           = begin_builtin_mpu9250();
            break;

        case m5::unit::imu::ImuKind::SH200Q:
            diag.builtin = true;
            ok           = begin_builtin_sh200q(board);
            break;

        case m5::unit::imu::ImuKind::None:
        default:
            M5_LOGE("No supported built-in accel/gyro IMU detected on the internal bus.");
            fail_reason     = "No built-in IMU detected";
            diag.unit_label = fail_reason;  // /info "source" reports the reason
            return false;
    }

    // A BMI270 with an AUX BMM150 carries Mag on the accel source itself; surface it for /info.
    if (ok && diag.accel_src && !diag.mag_src && m5::imu::any(diag.accel_src->imuSpec() & m5::imu::Capability::Mag)) {
        diag.mag_src   = diag.accel_src;
        diag.mag_label = "BMM150 (AUX)";
    }
    if (!ok && !fail_reason) {
        // Kind was a recognized chip, but its begin_builtin_*() helper itself failed (e.g. a bus
        // error) -- distinct from "no supported chip" above.
        fail_reason     = "Built-in IMU begin failed";
        diag.unit_label = fail_reason;  // /info "source" reports the reason
    }
    return ok;
}
#endif

//! @brief Attach a GROVE unit to this board's external I2C bus
//! @param u Unit to attach
//! @param attach_to_open_bus Pass true for the second and later units sharing one bus
//! @return False if the unit could not be attached
//! @details Classic M5Stack Core is the exception every configuration has to make: its default GROVE
//! port IS the internal bus, where a 0x68 unit collides with the built-in IMU, so those boards route
//! to GROVE Port C (needs the M5GO/Fire base). Every other board takes the wiring helper's default
//! port. Units carry their own clock, but 400kHz is forced here to match the DoF/MiniIMU defaults.
bool wire_grove_unit(m5::unit::Component& u, const bool attach_to_open_bus = false)
{
    if (M5.getBoard() == m5::board_t::board_M5Stack) {
        return m5::unit::imu::wiring::i2cPortC(Units, u, attach_to_open_bus);
    }
    return m5::unit::wiring::addI2C(Units, u, 400 * 1000U);
}

#if defined(USING_UNIT_DOF6) || defined(USING_UNIT_DOF9) || defined(USING_UNIT_DOF10)
//! @brief Warn when the DoF board's SW1 address differs from the one this build targets
//! @details The DoF BMI270 address is SW1-selectable (0x68 default / 0x69) and fixed at compile time
//! via DOF_BMI270_ADDRESS, so a flipped switch shows up as a bare "not found". Probe both candidates
//! once the bus is connected and name the mismatch instead. Arduino-only: the probe talks over Wire
//! directly, and it is skipped on classic Core, where Port C runs on the lgfx I2C_Class and Wire is
//! never begun. ESP-IDF builds and classic Core still see the mismatch as a begin failure.
void warn_if_dof_address_differs()
{
#if defined(ARDUINO)
    if (M5.getBoard() == m5::board_t::board_M5Stack) {
        return;
    }
    // The probe below reads through Wire, so it only sees the bus on hosts whose backend is Wire.
    // On NessoN1 (SoftwareI2C) and NanoC6/NanoH2 (ExI2C), the bus lives elsewhere -- a Wire read
    // would either see nothing or the wrong device. A mismatch still surfaces as a begin failure.
    if (m5::unit::wiring::i2cPins().backend != m5::unit::wiring::I2CPins::Backend::Wire) {
        return;
    }
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
#endif
}
#endif

//! @brief Record why a begin path failed, so the diagnostic mode and /info can report it
//! @param reason Message shown on the LCD notice and in the web panel
void note_begin_failure(const char* reason)
{
    fail_reason     = reason;
    diag.unit_label = reason;
}

#if defined(USING_UNIT_MINI_IMU)
//! @brief UnitMiniIMU (MPU6886 @0x68) on the GROVE bus -- 6-axis
bool begin_unit_mini_imu()
{
    const bool ok   = wire_grove_unit(unit) && Units.begin() && ahrs.add(unit);
    diag.unit_label = (M5.getBoard() == m5::board_t::board_M5Stack) ? "UnitMiniIMU (GROVE Port C @0x68)"
                                                                    : "UnitMiniIMU (MPU6886 @0x68)";
    diag.accel_src  = &unit;
    if (!ok) {
        note_begin_failure("UnitMiniIMU not found (check GROVE cable)");
    }
    return ok;
}
#elif defined(USING_UNIT_MINI_IMU_WITH_BMM350)
//! @brief UnitMiniIMU + Unit BMM350 (U231) -- 9-axis from two units on ONE GROVE bus
//! @details Split the port with a hub (0x68 + 0x14 coexist). Both wire_grove_unit() calls land on the
//! same bus; the second only attaches. UnitMiniIMU keeps the identity default, while the BMM350's
//! frame is rotated 180 degrees about X relative to it -- presets::unitBMM350() corrects that.
bool begin_unit_mini_imu_with_bmm350()
{
    const bool ok = wire_grove_unit(unit) && wire_grove_unit(mag_unit, true) && Units.begin() && ahrs.add(unit) &&
                    m5::imu::presets::unitBMM350(ahrs, mag_unit);
    diag.unit_label = "UnitMiniIMU + UnitBMM350";
    diag.accel_src  = &unit;
    diag.mag_src    = &mag_unit;
    diag.mag_label  = "BMM350";
    if (!ok) {
        note_begin_failure("UnitMiniIMU/BMM350 not found (check GROVE cable)");
    }
    return ok;
}
#elif defined(USING_UNIT_DOF6)
//! @brief Unit DoF6 (U228, BMI270 @0x68/0x69) -- 6-axis
bool begin_unit_dof6()
{
    bool ok = wire_grove_unit(unit);
    warn_if_dof_address_differs();
    ok              = ok && Units.begin() && ahrs.add(unit);
    diag.unit_label = "UnitDoF6 (BMI270)";
    diag.accel_src  = &unit;
    if (!ok) {
        note_begin_failure("UnitDoF6 not found (check GROVE cable)");
    }
    return ok;
}
#elif defined(USING_UNIT_DOF9) || defined(USING_UNIT_DOF10)
//! @brief Unit DoF9 / DoF10 (U229 / U230) -- BMI270 + BMM350 [+ SPA06 barometer]
//! @details The BMM350's frame is rotated 180 degrees about X relative to the BMI270's; the preset
//! registers both sources and applies that correction (presets.hpp records how it was derived).
//! Without it the heading is wrong in a way that changes with attitude, not by a fixed offset.
bool begin_unit_dof9_dof10()
{
    bool ok = wire_grove_unit(unit);
    warn_if_dof_address_differs();
    ok = ok && Units.begin() && m5::imu::presets::unitDoF(ahrs, unit.imu(), unit.mag());
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
        note_begin_failure("UnitDoF10 not found (check GROVE cable)");
#else
        note_begin_failure("UnitDoF9 not found (check GROVE cable)");
#endif
    }
    return ok;
}
#elif defined(USING_UNIT_MINI_IMU_PRO)
//! @brief Unit Mini IMU-Pro (U171) -- BMI270 @0x68 fixed + BMM150 + BMP280
//! @details No SW1 on this board, so no address probe. The BMM150 is an AUX target of the BMI270, so
//! imu() alone carries all nine axes.
bool begin_unit_mini_imu_pro()
{
    const bool ok   = wire_grove_unit(unit) && Units.begin() && ahrs.add(unit.imu());
    diag.unit_label = "UnitMiniIMUPro (BMI270+BMM150+BMP280)";
    diag.accel_src  = &unit.imu();
    diag.mag_src    = &unit.imu();
    diag.mag_label  = "BMM150 (BMI270 AUX)";
    if (!ok) {
        note_begin_failure("UnitMiniIMUPro not found (check GROVE cable)");
    }
    return ok;
}
#endif

#if defined(USING_BUILTIN_IMU_WITH_BMM350)
//! @brief Built-in accel/gyro plus a GROVE Unit BMM350 -- 9-axis for a host with no magnetometer
//! @details Two constraints shape this, and both are easy to "simplify" back into bugs.
//!
//! 1. The magnetometer is added BEFORE begin_builtin(): each begin_builtin_*() calls Units.begin()
//!    exactly once, and a second call would re-begin (and thereby stop) the running accel/gyro.
//! 2. It attaches to M5.Ex_I2C rather than going through wire_grove_unit(): this is the one
//!    configuration where the built-in IMU already owns M5.In_I2C, so letting M5Unified hand out
//!    both buses keeps the pairing explicit. M5Unified assigns them separate peripherals anyway
//!    (M5Unified.cpp, _begin_rtc_imu: Core2 gets In_I2C on I2C_NUM_1 and Ex_I2C on I2C_NUM_0), so
//!    the two never contend.
//!
//! Ex_I2C has to be started by hand, and that part is easy to miss: M5Unified only calls
//! Ex_I2C.begin() when config_t asks for an external RTC or IMU (same function), and this example
//! uses a plain M5.begin(). isEnabled() does not cover it -- it reports that the port/pins are
//! known, not that the bus was initialised -- so without the begin() below the first BMM350
//! transaction fails with "Not BMM350 00".
//!
//! 0x14 collides with nothing, so no port juggling is needed even on classic Core, where Ex_I2C and
//! In_I2C are the same physical bus.
bool begin_builtin_with_bmm350()
{
    const bool mag_wired = M5.Ex_I2C.isEnabled() && M5.Ex_I2C.begin() && Units.add(mag_unit, M5.Ex_I2C);
    if (!mag_wired) {
        M5_LOGE("Failed to attach the GROVE BMM350 to Ex_I2C");
    }
    if (!begin_builtin()) {
        return false;
    }
    if (diag.mag_src) {
        // This host already carries a magnetometer, so the GROVE one is redundant: both get fused.
        // Say so rather than letting the /info panel imply a single source.
        M5_LOGW("Built-in magnetometer present (%s); the GROVE BMM350 is fused on top of it",
                diag.mag_label ? diag.mag_label : "?");
    }
    if (!mag_wired || !m5::imu::presets::unitBMM350(ahrs, mag_unit)) {
        note_begin_failure("UnitBMM350 not found (check GROVE cable)");
        return false;
    }
    diag.mag_src   = &mag_unit;
    diag.mag_label = "BMM350 (GROVE)";
    return true;
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
    // With VQF selected, spend the spare room on its internal state as well: R = rest detected
    // (gyro bias is being estimated), D = magnetic disturbance detected (mag ignored for heading),
    // "-" when rejection is off and the flag therefore means nothing. Both are invisible
    // otherwise, and they explain behaviour that otherwise looks like a bug. The mag
    // auto-calibration state stays on the line either way -- it is what BtnA hold toggles.
    const bool show_vqf_state = (current_filter == m5::imu::FilterType::VQF) && (ahrs.filter() != nullptr);
    if (compact) {
        if (show_vqf_state) {
            const auto* vqf = static_cast<m5::imu::VQFFilter*>(ahrs.filter());
            text_canvas.printf("T:%.0fC C%d R%d D%c", ahrs.latest().temp, auto_cal_enabled ? 1 : 0,
                               vqf->restDetected() ? 1 : 0,
                               vqf->readMagDisturbanceRejection() ? (vqf->magDisturbanceDetected() ? '1' : '0') : '-');
        } else {
            text_canvas.printf("T:%.1fC", ahrs.latest().temp);
        }
    } else if (show_vqf_state) {
        const auto* vqf = static_cast<m5::imu::VQFFilter*>(ahrs.filter());
        text_canvas.printf("T:%4.1fC %s Rest:%d MagDis:%c", ahrs.latest().temp, auto_cal_enabled ? "Cal:ON" : "Cal:--",
                           vqf->restDetected() ? 1 : 0,
                           vqf->readMagDisturbanceRejection() ? (vqf->magDisturbanceDetected() ? '1' : '0') : '-');
    } else {
        text_canvas.printf("Temp:%6.2f C %s", ahrs.latest().temp, auto_cal_enabled ? "Cal:ON" : "Cal:--");
    }
#if defined(USING_UNIT_DOF10) || defined(USING_UNIT_MINI_IMU_PRO)
    const float baro_alt = unit.baro().altitude();
    const float hpa      = unit.baro().pressure() / 100.0f;  // baro().pressure() is Pa
    text_canvas.setCursor(4, 38);
    if (compact) {
        text_canvas.printf("P:%.0f A:%.0fm", hpa, baro_alt);
    } else {
        text_canvas.printf("P:%.1fhPa Alt:%.1fm", hpa, baro_alt);
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
// ESP32-H2 (NanoH2) has no WiFi radio, but __has_include alone cannot exclude it: PlatformIO's
// LDF adds the WiFi library to the include path from the raw #include line without evaluating
// #if, and on core 3.x WiFi.h then compiles empty there (its innards are SOC_WIFI_SUPPORTED-
// gated) -- hence the explicit H2 target check. Not SOC_WIFI_SUPPORTED here: that is undefined on
// some Arduino cores (ESP32 classic) and on ESP32-P4 (Tab5), whose WiFi is hosted, so it would
// wrongly compile the viewer out on those.
#if __has_include(<WiFi.h>) && !defined(CONFIG_IDF_TARGET_ESP32H2)
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
r.textContent='['+q.filter+']  roll '+rr.toFixed(1)+'  pitch '+pp.toFixed(1)+'  yaw '+yy.toFixed(1)+'  temp '+q.temp.toFixed(1)+'C  Cal '+(q.cal?'ON':'OFF')+(q.press!==undefined?('  P '+q.press.toFixed(1)+'hPa Alt '+q.alt.toFixed(1)+'m'):'');
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
#if defined(USING_UNIT_DOF10) || defined(USING_UNIT_MINI_IMU_PRO)
    const float baro_alt = unit.baro().altitude();
    const float hpa      = unit.baro().pressure() / 100.0f;  // baro().pressure() is Pa
    char buf[220];
    std::snprintf(buf, sizeof(buf),
                  "{\"w\":%.4f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"filter\":\"%s\",\"temp\":%.1f,\"press\":%.1f,"
                  "\"alt\":%.1f,\"cal\":%s}",
                  w, x, y, z, filter_name(current_filter), ahrs.latest().temp, hpa, baro_alt,
                  auto_cal_enabled ? "true" : "false");
#else
    char buf[180];
    std::snprintf(buf, sizeof(buf),
                  "{\"w\":%.4f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"filter\":\"%s\",\"temp\":%.1f,\"cal\":%s}", w, x, y,
                  z, filter_name(current_filter), ahrs.latest().temp, auto_cal_enabled ? "true" : "false");
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
#endif  // __has_include(<WiFi.h>) && !ESP32H2

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
    ok = begin_unit_mini_imu();
#elif defined(USING_UNIT_MINI_IMU_WITH_BMM350)
    ok = begin_unit_mini_imu_with_bmm350();
#elif defined(USING_UNIT_DOF6)
    ok = begin_unit_dof6();
#elif defined(USING_UNIT_DOF9) || defined(USING_UNIT_DOF10)
    ok = begin_unit_dof9_dof10();
#elif defined(USING_UNIT_MINI_IMU_PRO)
    ok = begin_unit_mini_imu_pro();
#elif defined(USING_BUILTIN_IMU_WITH_BMM350)
    ok = begin_builtin_with_bmm350();
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
        calibrate_gyro_at_boot();           // keep the raw bias out of every filter's integrator
        ahrs.selectFilter(current_filter);  // FilterType::Madgwick
        begin_settle_boost();               // the boot-time convergence benefits the same way

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

    end_settle_boost_if_due();

    const auto cap = ahrs.update();
    if (m5::imu::any(cap)) {
        float roll{}, pitch{}, yaw{};
        ahrs.euler(roll, pitch, yaw);
        if (current_filter == m5::imu::FilterType::VQF) {
            // VQF is the only filter that reports what it is doing internally; surfacing it makes
            // the two behaviours that are otherwise invisible observable: whether the gyro bias is
            // actually being estimated (rest), and whether the magnetometer is currently being
            // ignored because the field looks disturbed (magdist). See VQFFilter's Doxygen.
            const auto* vqf              = static_cast<m5::imu::VQFFilter*>(ahrs.filter());
            const m5::math::Vector3 bias = vqf->readGyroBias();
            // The disturbance flag is only maintained while rejection is enabled (off by
            // default); print "-" rather than a stale value that would read as a real detection
            char magdist[48]{"-"};
            if (vqf->readMagDisturbanceRejection()) {
                // Also report the field VQF is judging: the magnitude is rotation-invariant, but a
                // dip that swings while the device turns means the magnetometer's axes disagree
                // with the accel/gyro's -- and then no reference can ever be adopted.
                float mag_norm{}, mag_dip{};
                vqf->readMagNormDip(mag_norm, mag_dip);
                snprintf(magdist, sizeof(magdist), "%d norm:%.1f dip:%.1f", vqf->magDisturbanceDetected() ? 1 : 0,
                         mag_norm, mag_dip);
            }
            // dev:<gyro>,<accel> are the rest-detection deviations relative to their thresholds:
            // both must stay below 1.0 for rest (and therefore rest bias estimation) to arm, so
            // they show which sensor is holding a seemingly stationary device back.
            float dev_gyro{}, dev_accel{};
            vqf->readRestDeviations(dev_gyro, dev_accel);
            // |mag| straight from the controller's calibrated sample: unlike VQF's norm it is
            // computed whatever the filter is doing, so it stays readable while hard-iron
            // auto-calibration converges. A well-calibrated magnetometer holds it constant at the
            // local field strength (~25-65uT) however the device is turned; a value that swings as
            // you rotate is the signature of a hard-iron offset.
            const m5::imu::imu_physical_data_t& s = ahrs.latest();
            const float mag_magnitude = std::sqrt(s.mag[0] * s.mag[0] + s.mag[1] * s.mag[1] + s.mag[2] * s.mag[2]);
            M5.Log.printf(
                "[%s]: %.2f,%.2f,%.2f  temp:%.1f  cal:%d |mag|:%.1f rest:%d dev:%.2f,%.2f magdist:%s "
                "bias:%.3f,%.3f,%.3f\n",
                filter_name(current_filter), roll, pitch, yaw, s.temp, auto_cal_enabled ? 1 : 0, mag_magnitude,
                vqf->restDetected() ? 1 : 0, dev_gyro, dev_accel, magdist, bias.x, bias.y, bias.z);
        } else {
            M5.Log.printf("[%s]: %.2f,%.2f,%.2f  temp:%.1f\n", filter_name(current_filter), roll, pitch, yaw,
                          ahrs.latest().temp);
        }

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
        // Read-modify-write: only the mag strength is toggled here. Passing 0 for the other two
        // would silently switch off the gyro auto-calibration that begin() enables by default,
        // and its zero-rate offset would go straight back into yaw.
        uint8_t accel{}, gyro{}, mag{};
        ahrs.readAutoCalibration(accel, gyro, mag);
        ahrs.writeAutoCalibration(accel, gyro, auto_cal_enabled ? 64 : 0);
        M5_LOGI("Mag auto-calibration %s%s", auto_cal_enabled ? "ON" : "OFF",
                auto_cal_enabled ? " (wave the device in a figure-eight to converge)" : " (converged offset kept)");
    }

    // BtnB: let VQF's magnetic disturbance rejection be tried out. It is off by default in this
    // library (see VQFFilter::writeMagDisturbanceRejection()), so without a control like this
    // there is no way to experience it short of editing and reflashing. Single-button boards
    // (AtomS3, StickC, ...) still build and run -- M5.BtnB simply never fires there, and the
    // MagDis field keeps reading "-".
    // Note that selectFilter() constructs a fresh VQFFilter, so switching filters and coming back
    // returns the setting to its default.
    if (M5.BtnB.wasSingleClicked() && current_filter == m5::imu::FilterType::VQF && ahrs.filter() != nullptr) {
        auto* vqf            = static_cast<m5::imu::VQFFilter*>(ahrs.filter());
        const bool enable_it = !vqf->readMagDisturbanceRejection();
        vqf->writeMagDisturbanceRejection(enable_it);
        M5_LOGI("VQF mag disturbance rejection %s%s", enable_it ? "ON" : "OFF",
                enable_it ? " (rotate smoothly >20dps for 5s to let it adopt a reference)" : "");
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
