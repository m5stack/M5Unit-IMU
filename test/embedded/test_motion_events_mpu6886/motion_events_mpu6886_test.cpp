/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitMPU6886 motion events (AnyMotion via Wake-on-Motion / WoM).

  Default (no TEST_BUILTIN): external wiring via I2CComponentTestBase (GROVE). Plug a Unit Mini
  IMU (U095, MPU6886) into the Grove port.

  With -DTEST_BUILTIN: the board's built-in MPU6886 (0x68) is exercised over M5.In_I2C instead of
  the external Grove port (e.g. Core2 v1.1, which carries an MPU6886 -- v1.3 switched to a BMI270,
  see test_motion_events_bmi270).

  Coverage here is register-roundtrip focused (enable/disable state, spec advertisement, a
  high-threshold stationary sanity check) plus a check unique to this chip: enableEvent() rewrites
  PWR_MGMT_1 to enter WoM's cycle mode, and disableEvent() must restore exactly what was there
  before -- see UnitMPU6886::PreWomSnapshot in unit_mpu6886.hpp/.cpp.
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_mpu6886.hpp>
#include <imu/motion_events.hpp>
#include <chrono>
#include <thread>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::imu;

// PWR_MGMT_1 (0x6B): bit7 RESET / bit6 SLEEP / bit5 CYCLE / CLKSEL[2:0] -- see unit_mpu6xxx.hpp /
// unit_mpu6886.cpp. readRegister8()'s Reg template parameter requires an unsigned integral type,
// so this needs a properly-typed constant rather than a bare (signed int) literal.
constexpr uint8_t REG_PWR_MGMT_1{0x6B};

class TestMotionEventsMPU6886 : public I2CComponentTestBase<UnitMPU6886> {
protected:
    virtual UnitMPU6886* get_instance() override
    {
        auto ptr = new m5::unit::UnitMPU6886();
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }

#if defined(TEST_BUILTIN)
    // With -DTEST_BUILTIN the board's built-in MPU6886 (0x68) is exercised over the internal I2C
    // bus instead of the external Grove port.
    bool begin() override
    {
        return Units.add(*unit, M5.In_I2C) && Units.begin();
    }
#endif

    void SetUp() override
    {
        I2CComponentTestBase<UnitMPU6886>::SetUp();
        if (unit) {
            motion.add(*unit);
        }
    }

    MotionEventController motion;
};

// Register-roundtrip: enable/disable is reflected 1:1 in enabledEvents()
TEST_F(TestMotionEventsMPU6886, enable_disable_reflects_in_enabledEvents)
{
    SCOPED_TRACE(ustr);

    AnyMotionConfig cfg;  // defaults: 500mg / 40ms(ignored by WoM) / all axes
    EXPECT_EQ(motion.enabledEvents(), MotionEventKind::None);
    EXPECT_TRUE(motion.enable(MotionEventKind::AnyMotion, &cfg));
    EXPECT_EQ(motion.enabledEvents() & MotionEventKind::AnyMotion, MotionEventKind::AnyMotion);
    EXPECT_TRUE(motion.disable(MotionEventKind::AnyMotion));
    EXPECT_EQ(motion.enabledEvents() & MotionEventKind::AnyMotion, MotionEventKind::None);
}

// motionEventSpec()/has() advertise AnyMotion for this chip
TEST_F(TestMotionEventsMPU6886, spec_reports_any_motion)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(any(unit->motionEventSpec() & MotionEventKind::AnyMotion));
    EXPECT_TRUE(motion.has(MotionEventKind::AnyMotion));
}

// disable() leaves no pending fire behind, even shortly after a low-effort enable
TEST_F(TestMotionEventsMPU6886, disable_clears_state)
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
TEST_F(TestMotionEventsMPU6886, high_threshold_stationary_no_fire)
{
    SCOPED_TRACE(ustr);

    AnyMotionConfig cfg;
    cfg.threshold_mg = 2000;  // 2G-equivalent step; WoM ignores duration_ms
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

// Pre-state restore: enableEvent() rewrites PWR_MGMT_1 (bit5 CYCLE) to enter WoM; disableEvent()
// must restore exactly what was there beforehand, not just clear CYCLE
TEST_F(TestMotionEventsMPU6886, disable_restores_pre_enable_pwr_mgmt_1)
{
    SCOPED_TRACE(ustr);

    uint8_t pwr_before{};
    EXPECT_TRUE(unit->readRegister8(REG_PWR_MGMT_1, pwr_before, 0));

    AnyMotionConfig cfg;
    cfg.threshold_mg   = 500;
    const bool enabled = motion.enable(MotionEventKind::AnyMotion, &cfg);
    EXPECT_TRUE(enabled);
    if (!enabled) {
        return;
    }

    uint8_t pwr_during{};
    EXPECT_TRUE(unit->readRegister8(REG_PWR_MGMT_1, pwr_during, 0));
    EXPECT_TRUE(pwr_during & 0x20) << "CYCLE bit not set during WoM";

    EXPECT_TRUE(motion.disable(MotionEventKind::AnyMotion));

    uint8_t pwr_after{};
    EXPECT_TRUE(unit->readRegister8(REG_PWR_MGMT_1, pwr_after, 0));
    EXPECT_EQ(pwr_after, pwr_before) << "PWR_MGMT_1 not restored after disable";
}
