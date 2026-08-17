/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitIMU series with motion events
  Demonstrates the MotionEventController pattern with HW events (BMI270 AnyMotion / NoMotion /
  SignificantMotion / StepCounter, or MPU-6886 WoM) and an example software plugin (SoftwareFreeFall).

  Output:
   - Serial: "[FIRED] <event name>" per event, and a "[DBG] ax=... enb=... slot=..." dump every second
   - LCD-equipped boards additionally flash the background color and show the event name (or step count)
   - LCD panels wider than 200px also draw a debug strip at the bottom (accel xyz + enabled/last-fired masks)
  Controls:
   - BtnA click: cycle through the event slot table (see slots[] below for the full list)
   - BtnA hold: re-arm the current slot (re-writes the chip config; useful if events get stuck), and
     resets the chip's step counter to zero if the current slot uses StepCounter
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedIMU.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)
#include <wiring/m5_unit_imu_wiring.hpp>      // This library: classic Core Port C helper
#include "software_free_fall.hpp"
#if defined(USING_BUILTIN_IMU)
#include <imu/imu_detect.hpp>
#endif

// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_DOF6) && !defined(USING_UNIT_DOF9) && \
    !defined(USING_UNIT_DOF10) && !defined(USING_UNIT_MINI_IMU_PRO) && !defined(USING_BUILTIN_IMU)
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
// For built-in IMU (probed at runtime: BMI270 or MPU-6886)
// #define USING_BUILTIN_IMU
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
#elif defined(USING_BUILTIN_IMU)
// Built-in IMU on M5.In_I2C: probed at setup() to pick the driver (BMI270 or MPU-6886). Both
// storage instances are declared up front; only the one that matches the probe is added to Units.
m5::unit::UnitBMI270 hw_bmi_storage{};
m5::unit::UnitMPU6886 hw_mpu_storage{};
#else
#error Please choose unit!
#endif

m5::imu::MotionEventController motion;

// --- HW source that feeds MotionEventController + supplies accel samples for the SW plugin ---
// Plain units (MiniIMU/DoF6/MiniIMUPro's own accel/gyro chip) expose MotionEventSource directly;
// composite units (DoF9/DoF10) delegate to their BMI270 sub-unit via imu(). The built-in case is
// only known at runtime (BMI270 or MPU-6886), so hw_source_ptr there is the base MotionEventSource
// pointer, and bmi_ptr/mpu_ptr additionally track the concrete driver for the SW plugin sample
// feed and imuConvertParam(), neither of which is on the MotionEventSource interface.
#if defined(USING_BUILTIN_IMU)
m5::imu::MotionEventSource* hw_source_ptr = nullptr;
m5::unit::UnitBMI270* bmi_ptr             = nullptr;
m5::unit::UnitMPU6886* mpu_ptr            = nullptr;
#elif defined(USING_UNIT_MINI_IMU)
m5::unit::UnitMiniIMU* hw_source_ptr = &unit;
#elif defined(USING_UNIT_DOF6)
m5::unit::UnitBMI270* hw_source_ptr = &unit;
#elif defined(USING_UNIT_DOF9)
m5::unit::UnitBMI270* hw_source_ptr = &unit.imu();
#elif defined(USING_UNIT_DOF10)
m5::unit::UnitBMI270* hw_source_ptr = &unit.imu();
#elif defined(USING_UNIT_MINI_IMU_PRO)
m5::unit::UnitBMI270* hw_source_ptr = &unit.imu();
#endif
m5::imu::imu_convert_param_t convert_param{};

// --- SW extension (example plugin; see software_free_fall.hpp) ---
// Loosened from the plugin defaults (0.3G / 100ms) so a bench-thrown device triggers reliably;
// tune tighter (0.3G / 100ms or stricter) for real drop-detection.
example::SoftwareFreeFall sw_ff{example::SoftwareFreeFall::Config{0.5f, 50}};

// --- Event configuration shared by every slot that enables AnyMotion ---
// 500 mg matches the datasheet default and behaves well on both chips: BMI270's per-axis feature
// engine trips on clear shakes rather than every micro-movement, and MPU-6886's coarser 4 mg/LSB
// threshold still fires on deliberate shaking. Tune lower (200-300) for wearables where subtle
// gestures should trigger, higher (700-1000) for wall-mount / industrial wake-on-vibration.
// AnyMotionConfig is not an aggregate under C++11 (default member initializers), so build via IIFE.
const m5::imu::AnyMotionConfig any_cfg = [] {
    m5::imu::AnyMotionConfig c;
    c.threshold_mg = 500;
    return c;
}();

// --- BMI270-only event configs (NoMotion / SignificantMotion / StepCounter). MPU-6886 has no HW
// support for these; slots that use them are masked off there via SLOT_COUNT_SAFE. Not aggregates
// under C++11 (default member initializers), so built via IIFE like any_cfg above.
const m5::imu::NoMotionConfig no_motion_cfg = [] {
    m5::imu::NoMotionConfig c;
    c.threshold_mg = 70;    // datasheet default
    c.duration_ms  = 1000;  // 1 sec still; the 5 sec datasheet default is too long for a bench demo
    return c;
}();
const m5::imu::SignificantMotionConfig sig_motion_cfg = [] {
    m5::imu::SignificantMotionConfig c;
    c.duration_ms = 5000;  // Android SIGNIFICANT_MOTION standard block size
    return c;
}();
const m5::imu::StepCounterConfig step_counter_cfg = [] {
    m5::imu::StepCounterConfig c;
    c.mode            = m5::imu::StepCounterConfig::Mode::Counter;
    c.watermark_steps = 20;  // BMI270 hardware granularity is 20 steps; align intent with actual fire count
    return c;
}();
const m5::imu::StepCounterConfig step_detector_cfg = [] {
    m5::imu::StepCounterConfig c;
    c.mode            = m5::imu::StepCounterConfig::Mode::Detector;
    c.watermark_steps = 0;  // Detector mode ignores watermark_steps
    return c;
}();

// --- BMI270-only event configs (ActivityChange / PickUpGesture / WristGesture). All three are
// empty structs with no tunable params yet, so aggregate default-init is fine (no IIFE needed).
const m5::imu::ActivityChangeConfig activity_cfg{};
const m5::imu::PickUpGestureConfig pickup_cfg{};
const m5::imu::WristGestureConfig wrist_gesture_cfg{};

// --- Slot cycle table: BtnA click advances through these ---
struct EventSpec {
    m5::imu::MotionEventKind kind;
    const void* cfg;
};
struct Slot {
    const char* label;
    uint8_t count;
    EventSpec entries[4];
};
const Slot slots[] = {
    {"AnyMotion", 1, {{m5::imu::MotionEventKind::AnyMotion, &any_cfg}}},
    {"FreeFall(SW)", 1, {{example::FreeFallBit, nullptr}}},
    // The combo slot only works reliably on chips whose HW AnyMotion path does not disturb the
    // accelerometer output used by the SW plugin. MPU-6886 Wake-on-Motion puts the accelerometer
    // into low-power cycle mode; a brief motion spike can bypass SoftwareFreeFall's pre-fall
    // gravity guard and cause a spurious FreeFall right after the shake. slot_count masks it off
    // at runtime for that chip.
    {"AnyMotion+FreeFall", 2, {{m5::imu::MotionEventKind::AnyMotion, &any_cfg}, {example::FreeFallBit, nullptr}}},
    {"NoMotion", 1, {{m5::imu::MotionEventKind::NoMotion, &no_motion_cfg}}},
    {"SignificantMotion", 1, {{m5::imu::MotionEventKind::SignificantMotion, &sig_motion_cfg}}},
    {"StepCounter", 1, {{m5::imu::MotionEventKind::StepCounter, &step_counter_cfg}}},
    {"StepDetector", 1, {{m5::imu::MotionEventKind::StepCounter, &step_detector_cfg}}},
    // Detector mode fires on every single step, giving a demo-friendly overlap with the SW plugin;
    // Counter mode's 20-step watermark would fire too rarely to see the combo in a short bench test.
    {"StepCounter+FreeFall",
     2,
     {{m5::imu::MotionEventKind::StepCounter, &step_detector_cfg}, {example::FreeFallBit, nullptr}}},
    // BMI270-only events below (ActivityChange / PickUpGesture / WristGesture); MPU-6886 host stays
    // at SLOT_COUNT_SAFE and never reaches these slots.
    {"ActivityChange", 1, {{m5::imu::MotionEventKind::ActivityChange, &activity_cfg}}},
    // Detector mode reused here for the same demo-friendliness reason as StepCounter+FreeFall above.
    {"Activity+Step+NoMotion",
     3,
     {{m5::imu::MotionEventKind::ActivityChange, &activity_cfg},
      {m5::imu::MotionEventKind::StepCounter, &step_detector_cfg},
      {m5::imu::MotionEventKind::NoMotion, &no_motion_cfg}}},
    {"PickUp", 1, {{m5::imu::MotionEventKind::PickUpGesture, &pickup_cfg}}},
    {"WristGesture", 1, {{m5::imu::MotionEventKind::WristGesture, &wrist_gesture_cfg}}},
    {"PickUp+WristGesture",
     2,
     {{m5::imu::MotionEventKind::PickUpGesture, &pickup_cfg},
      {m5::imu::MotionEventKind::WristGesture, &wrist_gesture_cfg}}},
};
// slots that both chips handle correctly (AnyMotion, FreeFall SW). MPU-6886 has no HW support for
// NoMotion / SignificantMotion / StepCounter at all, so every slot beyond this stays BMI270-only.
constexpr size_t SLOT_COUNT_SAFE{2};
// full slot table, including the AnyMotion + FreeFall combo (only enabled where it works)
constexpr size_t SLOT_COUNT_FULL{sizeof(slots) / sizeof(slots[0])};

// Runtime slot count; adjusted in setup() based on chip capability
size_t slot_count{SLOT_COUNT_FULL};
size_t current_slot{0};

// --- LCD: header strip (slot label, redrawn only on slot change) + event status area (redrawn per fire)
// + optional debug strip at the bottom (accel xyz + masks) on panels wider than 200px ---
bool has_lcd{};
M5Canvas header_sprite(&lcd);
M5Canvas event_sprite(&lcd);
M5Canvas debug_sprite(&lcd);
int32_t header_h{};
int32_t debug_h{};

// --- Debug cache (updated every loop, sampled by the LCD strip and periodic Serial dump) ---
float last_ax{0.0f};
float last_ay{0.0f};
float last_az{0.0f};
float last_g{0.0f};  // magnitude sqrt(ax^2 + ay^2 + az^2)
m5::imu::MotionEventKind last_fired{m5::imu::MotionEventKind::None};

// event_sprite palette indices (a palette sprite interprets color arguments as indices)
enum : uint8_t {
    PAL_IDLE_BG      = 0,
    PAL_ANY_BG       = 1,
    PAL_FF_BG        = 2,
    PAL_COMBO_BG     = 3,
    PAL_TEXT_LT      = 4,
    PAL_TEXT_DK      = 5,
    PAL_STEP_BG      = 6,
    PAL_NOMOTION_BG  = 7,
    PAL_SIGMOTION_BG = 8,
    PAL_ACT_BG       = 9,
    PAL_WRIST_BG     = 10,
};

//! @brief True if any entry of the given slot enables StepCounter
bool slot_uses_step_counter(const size_t idx)
{
    const Slot& s = slots[idx];
    for (uint8_t i = 0; i < s.count; ++i) {
        if (m5::imu::any(s.entries[i].kind & m5::imu::MotionEventKind::StepCounter)) {
            return true;
        }
    }
    return false;
}

//! @brief Reset the BMI270 hardware step counter to zero (no-op on MPU-6886, which has no step counter)
void reset_step_count()
{
#if defined(USING_BUILTIN_IMU)
    if (bmi_ptr) {
        bmi_ptr->resetStepCount();
    }
#elif defined(USING_UNIT_MINI_IMU)
    // MPU-6886 has no HW step counter; StepCounter slots are unreachable here (slot_count == SLOT_COUNT_SAFE)
#else
    hw_source_ptr->resetStepCount();
#endif
}

//! @brief Disable every event, then enable the entries of the given slot; refresh the header
//! @param idx Slot index to activate
//! @param reset_step If true and the slot uses StepCounter, zero the chip's step counter first
void activate(const size_t idx, const bool reset_step)
{
    motion.disableAll();
    const Slot& s = slots[idx];
    bool all_ok   = true;
    for (uint8_t i = 0; i < s.count; ++i) {
        if (!motion.enable(s.entries[i].kind, s.entries[i].cfg)) {
            M5_LOGW("enable slot=%s entry=%u failed (chip unsupported)", s.label, i);
            all_ok = false;
        }
    }
    if (reset_step && slot_uses_step_counter(idx)) {
        reset_step_count();
    }
    M5_LOGI("[MotionEvents] active slot = %s (%u event%s, %s)", s.label, s.count, s.count > 1 ? "s" : "",
            all_ok ? "all enabled" : "partial");

    if (!has_lcd) {
        return;
    }
    header_sprite.fillScreen(TFT_BLACK);
    header_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    const int32_t line_h = header_sprite.fontHeight() + 2;
    header_sprite.setCursor(4, 2);
    header_sprite.printf("Slot: %s%s", s.label, all_ok ? "" : " (partial)");
    header_sprite.setCursor(4, 2 + line_h);
    header_sprite.print("BtnA: next");
    header_sprite.pushSprite(0, 0);
}

//! @brief Map ActivityType to a short LCD-friendly label
const char* activity_type_str(const m5::imu::ActivityType t)
{
    switch (t) {
        case m5::imu::ActivityType::Stationary:
            return "Still";
        case m5::imu::ActivityType::Walking:
            return "Walk";
        case m5::imu::ActivityType::Running:
            return "Run";
        default:
            return "?";  // Unknown / not yet classified
    }
}

//! @brief Map GestureType to a short LCD-friendly label
const char* gesture_type_str(const m5::imu::GestureType t)
{
    switch (t) {
        case m5::imu::GestureType::WristPushArmDown:
            return "PushDown";
        case m5::imu::GestureType::WristPivotUp:
            return "PivotUp";
        case m5::imu::GestureType::WristShake:
            return "Shake";
        case m5::imu::GestureType::WristFlickIn:
            return "FlickIn";
        case m5::imu::GestureType::WristFlickOut:
            return "FlickOut";
        default:
            return "?";  // None / not yet classified
    }
}

//! @brief Pick the text size that fits `text` inside `width`, starting from a width-proportional guess
uint8_t fit_text_size(M5Canvas& spr, const char* text, const int32_t width)
{
    uint8_t size = static_cast<uint8_t>(std::max<int32_t>(1, width / 80));
    spr.setTextSize(size);
    while (size > 1 && spr.textWidth(text) > width - 8) {
        --size;
        spr.setTextSize(size);
    }
    return size;
}

//! @brief Render the event status area: background color + centered event name (flicker-free: draws
//! off-screen into event_sprite, blits once via pushSprite)
void render_event(const m5::imu::MotionEventStatus& status)
{
    if (!has_lcd) {
        return;
    }

    const auto fired        = status.fired;
    const bool has_any      = m5::imu::any(fired & m5::imu::MotionEventKind::AnyMotion);
    const bool has_ff       = m5::imu::any(fired & example::FreeFallBit);
    const bool has_step     = m5::imu::any(fired & m5::imu::MotionEventKind::StepCounter);
    const bool has_no       = m5::imu::any(fired & m5::imu::MotionEventKind::NoMotion);
    const bool has_sig      = m5::imu::any(fired & m5::imu::MotionEventKind::SignificantMotion);
    const bool has_activity = m5::imu::any(fired & m5::imu::MotionEventKind::ActivityChange);
    const bool has_pickup   = m5::imu::any(fired & m5::imu::MotionEventKind::PickUpGesture);
    const bool has_wrist    = m5::imu::any(fired & m5::imu::MotionEventKind::WristGesture);

    char step_label[32];
    char act_label[32];
    char gest_label[32];
    const char* name;
    uint8_t bg_pal;
    uint8_t text_pal;
    if (has_any && has_ff) {
        name     = "AnyMotion + FreeFall";
        bg_pal   = PAL_COMBO_BG;
        text_pal = PAL_TEXT_LT;
    } else if (has_any) {
        name     = "AnyMotion";
        bg_pal   = PAL_ANY_BG;
        text_pal = PAL_TEXT_LT;
    } else if (has_ff) {
        name     = "FreeFall";
        bg_pal   = PAL_FF_BG;
        text_pal = PAL_TEXT_DK;
    } else if (has_step) {
        std::snprintf(step_label, sizeof(step_label), "Steps: %u", static_cast<unsigned>(status.step_count));
        name     = step_label;
        bg_pal   = PAL_STEP_BG;
        text_pal = PAL_TEXT_LT;
    } else if (has_sig) {
        name     = "SigMotion";
        bg_pal   = PAL_SIGMOTION_BG;
        text_pal = PAL_TEXT_LT;
    } else if (has_no) {
        name     = "NoMotion";
        bg_pal   = PAL_NOMOTION_BG;
        text_pal = PAL_TEXT_LT;
    } else if (has_activity) {
        std::snprintf(act_label, sizeof(act_label), "Activity: %s", activity_type_str(status.activity_type));
        name     = act_label;
        bg_pal   = PAL_ACT_BG;
        text_pal = PAL_TEXT_LT;
    } else if (has_pickup) {
        name     = "PickUp";
        bg_pal   = PAL_WRIST_BG;
        text_pal = PAL_TEXT_LT;
    } else if (has_wrist) {
        std::snprintf(gest_label, sizeof(gest_label), "Gesture: %s", gesture_type_str(status.gesture_type));
        name     = gest_label;
        bg_pal   = PAL_WRIST_BG;
        text_pal = PAL_TEXT_LT;
    } else {
        name     = slots[current_slot].label;
        bg_pal   = PAL_IDLE_BG;
        text_pal = PAL_TEXT_LT;
    }

    event_sprite.fillSprite(bg_pal);
    event_sprite.setTextColor(text_pal, bg_pal);
    fit_text_size(event_sprite, name, event_sprite.width());
    const int32_t text_w = event_sprite.textWidth(name);
    const int32_t text_h = event_sprite.fontHeight();
    event_sprite.setCursor((event_sprite.width() - text_w) / 2, (event_sprite.height() - text_h) / 2);
    event_sprite.print(name);
    event_sprite.pushSprite(0, header_h);
}

//! @brief Log every fired bit this update() cycle carries (may be more than one at once)
void log_fired(const m5::imu::MotionEventStatus& status)
{
    const auto fired = status.fired;
    if (m5::imu::any(fired & m5::imu::MotionEventKind::AnyMotion)) {
        M5_LOGI("[FIRED] AnyMotion");
    }
    if (m5::imu::any(fired & example::FreeFallBit)) {
        M5_LOGI("[FIRED] FreeFall(SW)");
    }
    if (m5::imu::any(fired & m5::imu::MotionEventKind::NoMotion)) {
        M5_LOGI("[FIRED] NoMotion");
    }
    if (m5::imu::any(fired & m5::imu::MotionEventKind::SignificantMotion)) {
        M5_LOGI("[FIRED] SignificantMotion");
    }
    if (m5::imu::any(fired & m5::imu::MotionEventKind::StepCounter)) {
        M5_LOGI("[FIRED] StepCounter %u", static_cast<unsigned>(status.step_count));
    }
    if (m5::imu::any(fired & m5::imu::MotionEventKind::ActivityChange)) {
        M5_LOGI("[FIRED] ActivityChange %s", activity_type_str(status.activity_type));
    }
    if (m5::imu::any(fired & m5::imu::MotionEventKind::PickUpGesture)) {
        M5_LOGI("[FIRED] PickUp");
    }
    if (m5::imu::any(fired & m5::imu::MotionEventKind::WristGesture)) {
        M5_LOGI("[FIRED] WristGesture %s", gesture_type_str(status.gesture_type));
    }
}

#if defined(USING_BUILTIN_IMU)
//! @brief Read the latest accel sample for the SW plugin from whichever driver the probe selected
m5::imu::imu_physical_data_t read_accel_sample()
{
    m5::imu::imu_physical_data_t sample{};
    if (bmi_ptr) {
        const auto d    = bmi_ptr->latest();
        sample.accel[0] = d.accel_x() * convert_param.accel_res;
        sample.accel[1] = d.accel_y() * convert_param.accel_res;
        sample.accel[2] = d.accel_z() * convert_param.accel_res;
    } else if (mpu_ptr) {
        const auto d    = mpu_ptr->latest();
        sample.accel[0] = d.accel_x() * convert_param.accel_res;
        sample.accel[1] = d.accel_y() * convert_param.accel_res;
        sample.accel[2] = d.accel_z() * convert_param.accel_res;
    }
    return sample;
}
#endif

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
        lcd.fillScreen(TFT_BLACK);

        // Header strip: white-on-black text only -> a 1-bit palette sprite is enough.
        header_sprite.setColorDepth(1);
        header_sprite.setPsram(false);
        // Text size scales mildly with panel width so the header stays legible on large panels
        // (e.g. Tab5 1280x720) without needing much room on small ones (e.g. AtomS3 128x128).
        const uint8_t header_ts     = static_cast<uint8_t>(std::max<int32_t>(1, lcd.width() / 200));
        const int32_t header_line_h = 8 * header_ts + 4;
        header_sprite.createSprite(lcd.width(), header_line_h * 2 + 4);
        header_sprite.setTextSize(header_ts);
        header_h = header_sprite.height();

        // Debug strip (bottom): accel xyz + enabled/last-fired masks. 1-bit palette is enough
        // (white-on-black text). Skip it on narrow panels; the periodic Serial dump still fires.
        if (lcd.width() >= 200) {
            const uint8_t debug_ts = static_cast<uint8_t>(std::max<int32_t>(1, lcd.width() / 300));
            debug_sprite.setColorDepth(1);
            debug_sprite.setPsram(false);
            debug_sprite.createSprite(lcd.width(), 8 * debug_ts * 2 + 6);
            debug_sprite.setTextSize(debug_ts);
            debug_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
            debug_h = debug_sprite.height();
        }

        // Event status area: minimum required area (below the header, above the debug strip), 4-bit
        // palette (9 colors used).
        const int32_t event_h = lcd.height() - header_h - debug_h;
        event_sprite.setColorDepth(4);
        event_sprite.setPsram(false);
        // Internal RAM may not fit a large panel (e.g. Tab5 1280x720) at 4bit; fall back to PSRAM.
        if (static_cast<size_t>(lcd.width()) * static_cast<size_t>(event_h) / 2 > 100000) {
            event_sprite.setPsram(true);
        }
        if (!event_sprite.createSprite(lcd.width(), event_h)) {
            event_sprite.setPsram(true);
            event_sprite.createSprite(lcd.width(), event_h);
        }
        event_sprite.createPalette();
        event_sprite.setPaletteColor(PAL_IDLE_BG, TFT_BLACK);
        event_sprite.setPaletteColor(PAL_ANY_BG, TFT_RED);
        event_sprite.setPaletteColor(PAL_FF_BG, TFT_YELLOW);
        event_sprite.setPaletteColor(PAL_COMBO_BG, TFT_MAGENTA);
        event_sprite.setPaletteColor(PAL_TEXT_LT, TFT_WHITE);
        event_sprite.setPaletteColor(PAL_TEXT_DK, TFT_BLACK);
        event_sprite.setPaletteColor(PAL_STEP_BG, TFT_DARKGREEN);
        event_sprite.setPaletteColor(PAL_NOMOTION_BG, TFT_NAVY);
        event_sprite.setPaletteColor(PAL_SIGMOTION_BG, TFT_ORANGE);
        event_sprite.setPaletteColor(PAL_ACT_BG, TFT_MAGENTA);     // activity: a continuously-changing state
        event_sprite.setPaletteColor(PAL_WRIST_BG, TFT_DARKCYAN);  // wrist gestures: smart-watch feel
    }

    bool ok{};
#if defined(USING_BUILTIN_IMU)
    // Built-in IMU lives on M5.In_I2C. M5.getBoard() cannot tell IMU generations apart (e.g. Core2
    // v1.1 = MPU6886, v1.3 = BMI270 -- both report the same board id), so probe the bus and register
    // whichever driver matches (mirrors AhrsDemo's begin_builtin()).
    const auto kind = m5::unit::imu::detectBuiltinImu(
        [](uint8_t addr, uint8_t reg, uint8_t& out) { return M5.In_I2C.readRegister(addr, reg, &out, 1, 400000); });
    switch (kind) {
        case m5::unit::imu::ImuKind::BMI270:
            bmi_ptr       = &hw_bmi_storage;
            hw_source_ptr = &hw_bmi_storage;
            ok            = Units.add(hw_bmi_storage, M5.In_I2C);
            break;
        case m5::unit::imu::ImuKind::MPU6886:
        case m5::unit::imu::ImuKind::MPU6050:
        case m5::unit::imu::ImuKind::MPU9250:
            mpu_ptr       = &hw_mpu_storage;
            hw_source_ptr = &hw_mpu_storage;
            ok            = Units.add(hw_mpu_storage, M5.In_I2C);
            break;
        default:
            M5_LOGE("No supported built-in IMU detected (BMI270 or MPU-6886)");
            m5::unit::wiring::failStop();
    }
#else
    // --- I2C bus connection (skill m5-i2c-three-way-branch) ---
    // UnitMiniIMU / UnitDoF6 / UnitDoF9 / UnitDoF10 / UnitMiniIMUPro: NessoN1 -> SoftwareI2C (M5HAL),
    // NanoC6 / NanoH2 -> M5.Ex_I2C, others -> Wire
    // On classic M5Stack Core (board_M5Stack) GROVE Port A *is* the internal I2C bus, and Gray/Fire
    // carry a built-in MPU6886 or BMI270 at 0x68 -- the same address as this library's units. Route
    // the external unit to GROVE Port C on a bus of its own so it never collides with the built-in IMU
    // (same handling as PlotToSerial/AhrsDemo).
    if (M5.getBoard() == m5::board_t::board_M5Stack) {
        ok = m5::unit::imu::wiring::i2cPortC(Units, unit);
    } else {
        ok = m5::unit::wiring::addI2C(Units, unit, 400 * 1000U);
    }
#endif

    if (!ok || !Units.begin()) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

#if defined(USING_BUILTIN_IMU)
    if (bmi_ptr) {
        bmi_ptr->imuConvertParam(convert_param);
    } else if (mpu_ptr) {
        mpu_ptr->imuConvertParam(convert_param);
    }
#else
    hw_source_ptr->imuConvertParam(convert_param);
#endif

    // --- Register HW + SW sources with the controller, then activate the first slot ---
    motion.add(*hw_source_ptr);
    motion.add(sw_ff);

    // Slot 3 (AnyMotion + FreeFall combo) is skipped on chips that cannot run the two together
    // reliably (see the slots[] definition for the reason). Compile-time when we already know the
    // chip; runtime probe result for USING_BUILTIN_IMU.
#if defined(USING_UNIT_MINI_IMU)
    slot_count = SLOT_COUNT_SAFE;  // MPU-6886 -- WoM breaks the SW plugin
#elif defined(USING_BUILTIN_IMU)
    if (mpu_ptr) slot_count = SLOT_COUNT_SAFE;
#endif
    if (slot_count < SLOT_COUNT_FULL) {
        M5_LOGI("Slot table: %u slots (combo slot skipped: chip cannot run HW AnyMotion and SW plugin together)",
                static_cast<unsigned>(slot_count));
    } else {
        M5_LOGI("Slot table: %u slots", static_cast<unsigned>(slot_count));
    }
    activate(current_slot, false);

    render_event(m5::imu::MotionEventStatus{});  // idle

    M5_LOGI("MotionEvents ready");
}

void loop()
{
    static uint32_t prev_ms{0};

    M5.update();
    Units.update();

    // Buttons: check every iteration so BtnA responds immediately (matches AhrsDemo / PlotToSerial;
    // dropping the trailing delay is what makes this cheap).
    if (M5.BtnA.wasClicked()) {
        current_slot = (current_slot + 1) % slot_count;
        activate(current_slot, false);
        render_event(m5::imu::MotionEventStatus{});  // idle after switching
    }
    if (M5.BtnA.wasHold()) {
        // Long press: re-arm the current slot (re-write chip config + reset SW plugin state), and
        // zero the chip's step counter if the slot uses StepCounter. Useful if events get stuck
        // (e.g. a chip register got out of sync) or the config needs to be re-applied after a fault.
        activate(current_slot, true);
        render_event(m5::imu::MotionEventStatus{});
        M5_LOGI("[REARM] slot %s", slots[current_slot].label);
    }

    const uint32_t now = m5::utility::millis();

    // Motion tick: only when a fresh IMU sample lands (AhrsDemo / PlotToSerial convention).
    // updated() flips true for one pass after Units.update() reads a new sample, so this naturally
    // matches the chip's ODR without a hand-tuned tick period.
#if defined(USING_BUILTIN_IMU)
    const bool sample_ready = (bmi_ptr && bmi_ptr->updated()) || (mpu_ptr && mpu_ptr->updated());
#else
    const bool sample_ready = hw_source_ptr->updated();
#endif

    if (sample_ready) {
        const float dt = (prev_ms == 0) ? 0.020f : (now - prev_ms) / 1000.0f;
        prev_ms        = now;

        // Feed the SW plugin (SoftwareFreeFall) the latest physical-unit accel sample
#if defined(USING_BUILTIN_IMU)
        const auto sample = read_accel_sample();
#else
        const auto d = hw_source_ptr->latest();
        m5::imu::imu_physical_data_t sample{};
        sample.accel[0] = d.accel_x() * convert_param.accel_res;
        sample.accel[1] = d.accel_y() * convert_param.accel_res;
        sample.accel[2] = d.accel_z() * convert_param.accel_res;
#endif
        sw_ff.update(dt, sample);
        last_ax = sample.accel[0];
        last_ay = sample.accel[1];
        last_az = sample.accel[2];
        last_g  = std::sqrt(last_ax * last_ax + last_ay * last_ay + last_az * last_az);

        // Poll every registered source and aggregate newly fired events
        const auto fired = motion.update(dt);
        if (m5::imu::any(fired)) {
            last_fired = fired;
            m5::imu::MotionEventStatus status{};
            motion.latest(status);
            log_fired(status);
            render_event(status);
        }
    }

    // Debug strip: refresh at ~5 Hz using last-known values (runs regardless of sample_ready so
    // the strip keeps updating even when the chip is quiet).
    static uint32_t last_debug_ms{0};
    if (has_lcd && debug_h > 0 && (now - last_debug_ms) >= 200) {
        last_debug_ms = now;
        debug_sprite.fillSprite(TFT_BLACK);
        const int32_t line_h = 8 * debug_sprite.getTextSizeY() + 2;
        debug_sprite.setCursor(4, 2);
        debug_sprite.printf("ax:%+.2f ay:%+.2f az:%+.2f g:%.2f", last_ax, last_ay, last_az, last_g);
        debug_sprite.setCursor(4, 2 + line_h);
        const unsigned enb   = static_cast<unsigned>(motion.enabledEvents());
        const unsigned fmask = static_cast<unsigned>(last_fired);
        debug_sprite.printf("enb:0x%02X last:0x%02X", enb, fmask);
        debug_sprite.pushSprite(0, header_h + event_sprite.height());
    }

    // Serial dump: one line per second so a lost fire is still visible in the log
    static uint32_t last_serial_ms{0};
    if ((now - last_serial_ms) >= 1000) {
        last_serial_ms       = now;
        const unsigned enb   = static_cast<unsigned>(motion.enabledEvents());
        const unsigned fmask = static_cast<unsigned>(last_fired);
        M5_LOGI("[DBG] ax=%+.2f ay=%+.2f az=%+.2f g=%.2f enb=0x%02X last=0x%02X slot=%s", last_ax, last_ay, last_az,
                last_g, enb, fmask, slots[current_slot].label);
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
