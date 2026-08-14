/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitBMI270 motion events (AnyMotion via the feature engine).

  Default (no TEST_BUILTIN): external wiring via I2CComponentTestBase (GROVE). Plug a Unit DoF6
  (U228) or any bare BMI270 wiring into the Grove port; build with -D DOF_BMI270_ADDRESS=0x69
  when the board's SW1 is flipped (0x68 is the default). This suite instantiates the plain
  UnitBMI270 (not UnitBMI270AuxBMM150), matching test_dof6 -- motion events do not touch the
  AUX interface, so there is no reason to probe it here.

  With -DTEST_BUILTIN: the board's built-in BMI270 is exercised over M5.In_I2C (e.g. Core2 v1.3,
  CoreS3, AtomS3R). The built-in BMI270 sits at either 0x68 or 0x69 depending on the board, so
  get_instance() probes both addresses via CHIP_ID (mirrors test_bmi270.cpp) and begin() wires
  the unit onto M5.In_I2C.

  Coverage here is register-roundtrip focused (enable/disable state, spec advertisement, and a
  high-threshold stationary sanity check) rather than threshold fine-tuning, since ambient noise
  and the exact chip placement vary per bench.
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_bmi270.hpp>
#include <imu/motion_events.hpp>
#include <chrono>
#include <thread>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::imu;

// GROVE path: the DoF boards' SW1 ships at 0x68 (UnitBMI270's class default is 0x69, matching the
// CoreS3 built-in) -- follow the DoF suites' convention: -D DOF_BMI270_ADDRESS=0x69 when flipped.
#ifndef DOF_BMI270_ADDRESS
#define DOF_BMI270_ADDRESS 0x68
#endif

class TestMotionEventsBMI270 : public I2CComponentTestBase<UnitBMI270> {
protected:
    virtual UnitBMI270* get_instance() override
    {
#if defined(TEST_BUILTIN)
        // The built-in BMI270 sits at a board-dependent address (0x68 or 0x69). Detect it on the
        // internal bus and construct with the result -- no build-time address needed.
        uint8_t addr{0x68};
        m5::unit::bmi270::detectAddress(
            [](uint8_t a, uint8_t reg, uint8_t& out) { return M5.In_I2C.readRegister(a, reg, &out, 1, 400000); }, addr);
        auto ptr = new m5::unit::UnitBMI270(addr);
#else
        // GROVE: the bus is not connected yet at get_instance() time, so no probe -- use the
        // build-time address (see DOF_BMI270_ADDRESS above).
        auto ptr = new m5::unit::UnitBMI270(DOF_BMI270_ADDRESS);
#endif
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }

#if defined(TEST_BUILTIN)
    // Built-in IMU lives on the internal I2C bus, not the external Grove port.
    bool begin() override
    {
        return Units.add(*unit, M5.In_I2C) && Units.begin();
    }
#endif

    void SetUp() override
    {
        I2CComponentTestBase<UnitBMI270>::SetUp();
        if (unit) {
            motion.add(*unit);
        }
    }

    MotionEventController motion;
};

// Register-roundtrip: enable/disable is reflected 1:1 in enabledEvents()
TEST_F(TestMotionEventsBMI270, enable_disable_reflects_in_enabledEvents)
{
    SCOPED_TRACE(ustr);

    AnyMotionConfig cfg;  // defaults: 500mg / 40ms / all axes
    EXPECT_EQ(motion.enabledEvents(), MotionEventKind::None);
    EXPECT_TRUE(motion.enable(MotionEventKind::AnyMotion, &cfg));
    EXPECT_EQ(motion.enabledEvents() & MotionEventKind::AnyMotion, MotionEventKind::AnyMotion);
    EXPECT_TRUE(motion.disable(MotionEventKind::AnyMotion));
    EXPECT_EQ(motion.enabledEvents() & MotionEventKind::AnyMotion, MotionEventKind::None);
}

// motionEventSpec()/has() advertise AnyMotion for this chip
TEST_F(TestMotionEventsBMI270, spec_reports_any_motion)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(any(unit->motionEventSpec() & MotionEventKind::AnyMotion));
    EXPECT_TRUE(motion.has(MotionEventKind::AnyMotion));
}

// disable() leaves no pending fire behind, even shortly after a low-effort enable
TEST_F(TestMotionEventsBMI270, disable_clears_state)
{
    SCOPED_TRACE(ustr);

    AnyMotionConfig cfg;
    EXPECT_TRUE(motion.enable(MotionEventKind::AnyMotion, &cfg));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_TRUE(motion.disable(MotionEventKind::AnyMotion));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Post-disable: a 500ms poll should produce no fires
    for (int i = 0; i < 50; ++i) {
        unit->update();
        motion.update(0.010f);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    MotionEventStatus st{};
    motion.latest(st);
    EXPECT_FALSE(any(st.fired & MotionEventKind::AnyMotion));
}

// Best-effort: a near-max threshold must not fire while the board sits still
TEST_F(TestMotionEventsBMI270, high_threshold_stationary_no_fire)
{
    SCOPED_TRACE(ustr);

    AnyMotionConfig cfg;
    cfg.threshold_mg = 2000;  // 2G, near/above the chip's usable range
    cfg.duration_ms  = 100;
    EXPECT_TRUE(motion.enable(MotionEventKind::AnyMotion, &cfg));

    for (int i = 0; i < 50; ++i) {
        unit->update();
        motion.update(0.010f);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    MotionEventStatus st{};
    motion.latest(st);
    EXPECT_FALSE(any(st.fired & MotionEventKind::AnyMotion))
        << "Stationary board triggered a 2G threshold: implementation or chip issue";
    motion.disable(MotionEventKind::AnyMotion);
}

// --- NoMotion ---

// Register-roundtrip: enable/disable is reflected 1:1 in enabledEvents()
TEST_F(TestMotionEventsBMI270, no_motion_enable_disable_reflects)
{
    SCOPED_TRACE(ustr);

    NoMotionConfig cfg;  // defaults: 70mg / 5000ms / all axes
    EXPECT_TRUE(motion.enable(MotionEventKind::NoMotion, &cfg));
    EXPECT_EQ(motion.enabledEvents() & MotionEventKind::NoMotion, MotionEventKind::NoMotion);
    EXPECT_TRUE(motion.disable(MotionEventKind::NoMotion));
    EXPECT_EQ(motion.enabledEvents() & MotionEventKind::NoMotion, MotionEventKind::None);
}

// Positive fire test: a short still-duration should fire while the board sits still on the bench
TEST_F(TestMotionEventsBMI270, no_motion_short_duration_fires_when_still)
{
    SCOPED_TRACE(ustr);

    NoMotionConfig cfg{70, 100, true, true, true};  // 100ms still -> fire
    EXPECT_TRUE(motion.enable(MotionEventKind::NoMotion, &cfg));

    bool fired_within_500ms = false;
    for (int i = 0; i < 50; ++i) {
        unit->update();
        if (any(motion.update(0.010f) & MotionEventKind::NoMotion)) {
            fired_within_500ms = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(fired_within_500ms)
        << "NoMotion did not fire within 500ms while still; chip noise floor may exceed the threshold";
    motion.disable(MotionEventKind::NoMotion);
}

// Negative test: a duration far longer than the poll window must not fire
TEST_F(TestMotionEventsBMI270, no_motion_long_duration_no_fire_within_1sec)
{
    SCOPED_TRACE(ustr);

    NoMotionConfig cfg{70, 100000, true, true, true};  // 100 sec still (unreachable within this test)
    EXPECT_TRUE(motion.enable(MotionEventKind::NoMotion, &cfg));

    for (int i = 0; i < 100; ++i) {
        unit->update();
        motion.update(0.010f);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    MotionEventStatus st{};
    motion.latest(st);
    EXPECT_FALSE(any(st.fired & MotionEventKind::NoMotion));
    motion.disable(MotionEventKind::NoMotion);
}

// --- SignificantMotion ---

// Register-roundtrip: enable/disable is reflected 1:1 in enabledEvents()
TEST_F(TestMotionEventsBMI270, sig_motion_enable_disable_reflects)
{
    SCOPED_TRACE(ustr);

    SignificantMotionConfig cfg;  // default 5000ms
    EXPECT_TRUE(motion.enable(MotionEventKind::SignificantMotion, &cfg));
    EXPECT_EQ(motion.enabledEvents() & MotionEventKind::SignificantMotion, MotionEventKind::SignificantMotion);
    EXPECT_TRUE(motion.disable(MotionEventKind::SignificantMotion));
}

// A block_size below the chip's 50Hz sample period (20ms) cannot be represented and must be rejected
TEST_F(TestMotionEventsBMI270, sig_motion_duration_out_of_range_rejected)
{
    SCOPED_TRACE(ustr);

    SignificantMotionConfig cfg{10};  // < 20ms minimum -> conversion fails
    EXPECT_FALSE(motion.enable(MotionEventKind::SignificantMotion, &cfg));
    EXPECT_EQ(motion.enabledEvents() & MotionEventKind::SignificantMotion, MotionEventKind::None);
}

// --- StepCounter ---

// Register-roundtrip: enable/disable is reflected 1:1 in enabledEvents() (Counter mode)
TEST_F(TestMotionEventsBMI270, step_counter_mode_enable_disable_reflects)
{
    SCOPED_TRACE(ustr);

    StepCounterConfig cfg{StepCounterConfig::Mode::Counter, 5};
    EXPECT_TRUE(motion.enable(MotionEventKind::StepCounter, &cfg));
    EXPECT_EQ(motion.enabledEvents() & MotionEventKind::StepCounter, MotionEventKind::StepCounter);
    EXPECT_TRUE(motion.disable(MotionEventKind::StepCounter));
}

// Register-roundtrip: enable/disable is reflected 1:1 in enabledEvents() (Detector mode)
TEST_F(TestMotionEventsBMI270, step_detector_mode_enable_disable_reflects)
{
    SCOPED_TRACE(ustr);

    StepCounterConfig cfg{StepCounterConfig::Mode::Detector, 0};
    EXPECT_TRUE(motion.enable(MotionEventKind::StepCounter, &cfg));
    EXPECT_EQ(motion.enabledEvents() & MotionEventKind::StepCounter, MotionEventKind::StepCounter);
    EXPECT_TRUE(motion.disable(MotionEventKind::StepCounter));
}

// A fresh enable must start the chip's cumulative step count at 0
TEST_F(TestMotionEventsBMI270, step_counter_initial_count_is_zero)
{
    SCOPED_TRACE(ustr);

    StepCounterConfig cfg{StepCounterConfig::Mode::Counter, 1};
    EXPECT_TRUE(motion.enable(MotionEventKind::StepCounter, &cfg));

    // Trigger a read cycle so the driver has a chance to sample the count register.
    unit->update();
    motion.update(0.010f);

    MotionEventStatus st{};
    motion.latest(st);
    EXPECT_EQ(st.step_count, 0u) << "Fresh enable did not start step_count at 0";
    motion.disable(MotionEventKind::StepCounter);
}
