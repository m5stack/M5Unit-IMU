/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitSPA06

  The SPA06-003 barometer has no standalone GROVE product; it ships on Unit DoF10 at 0x76. Plug a
  DoF10 into the Grove port -- this suite adds ONLY the standalone UnitSPA06 driver at 0x76 (the
  BMI270/BMM350 on the same bus are simply left untouched), so the barometer driver is exercised
  in isolation. The full 3-child composite path is test_dof10.
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_spa06.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;

class TestSPA06 : public I2CComponentTestBase<UnitSPA06> {
protected:
    virtual UnitSPA06* get_instance() override
    {
        auto ptr = new m5::unit::UnitSPA06();
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }
};

TEST_F(TestSPA06, BeginStartsPeriodic)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->inPeriodic());
}

TEST_F(TestSPA06, PeriodicMeasurement)
{
    SCOPED_TRACE(ustr);

    // Default rate is 16 samples/s; 200ms is enough for a couple of samples
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int updated{};
    for (int i = 0; i < 50; ++i) {
        unit->update();
        if (unit->updated()) {
            ++updated;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_GT(updated, 0);
    EXPECT_FALSE(unit->empty());

    // Compensated values in plausible ambient ranges
    const float hpa = unit->pressure();
    EXPECT_TRUE(std::isfinite(hpa));
    EXPECT_GT(hpa, 300.0f);  // high-altitude floor
    EXPECT_LT(hpa, 1100.0f);

    const float celsius = unit->temperature();
    EXPECT_TRUE(std::isfinite(celsius));
    EXPECT_GT(celsius, -20.0f);
    EXPECT_LT(celsius, 60.0f);

    const float alt = unit->altitude();
    EXPECT_TRUE(std::isfinite(alt));
    EXPECT_GT(alt, -500.0f);  // below-sea-level margin
    EXPECT_LT(alt, 9000.0f);
}

TEST_F(TestSPA06, RelativeAltitudeTare)
{
    SCOPED_TRACE(ustr);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    for (int i = 0; i < 20; ++i) {
        unit->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    unit->setReference();
    // Right after taring, the relative altitude of a static unit is ~0 m
    const float rel = unit->relativeAltitude();
    EXPECT_TRUE(std::isfinite(rel));
    EXPECT_NEAR(rel, 0.0f, 2.0f);  // sensor noise floor is well under 2 m
}

TEST_F(TestSPA06, StopStartPeriodic)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    for (int i = 0; i < 10; ++i) {
        unit->update();
        EXPECT_FALSE(unit->updated());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    bool updated{};
    for (int i = 0; i < 50; ++i) {
        unit->update();
        if (unit->updated()) {
            updated = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(updated);
}

TEST_F(TestSPA06, ConfigRoundTrip)
{
    SCOPED_TRACE(ustr);

    auto cfg                     = unit->config();
    cfg.start_periodic           = false;
    cfg.pressure_oversampling    = 32;
    cfg.temperature_oversampling = 2;
    cfg.rate                     = 2;
    unit->config(cfg);

    const auto cfg2 = unit->config();
    EXPECT_FALSE(cfg2.start_periodic);
    EXPECT_EQ(cfg2.pressure_oversampling, cfg.pressure_oversampling);
    EXPECT_EQ(cfg2.temperature_oversampling, cfg.temperature_oversampling);
    EXPECT_EQ(cfg2.rate, cfg.rate);
}

TEST_F(TestSPA06, SoftReset)
{
    SCOPED_TRACE(ustr);

    // Running: rejected
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_FALSE(unit->softReset());

    // Stopped: accepted; startPeriodicMeasurement() reconfigures and resumes
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->softReset());
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());

    // Measurement actually flows again
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    bool updated{};
    for (int i = 0; i < 50; ++i) {
        unit->update();
        if (unit->updated()) {
            updated = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(updated);
}

TEST_F(TestSPA06, StartPeriodicWithArgs)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());

    // Non-default settings (32x/2x oversampling, 4 samples/s) are written and synced to config()
    EXPECT_TRUE(unit->startPeriodicMeasurement(32, 2, 2));
    EXPECT_TRUE(unit->inPeriodic());

    const auto cfg = unit->config();
    EXPECT_EQ(cfg.pressure_oversampling, 32);
    EXPECT_EQ(cfg.temperature_oversampling, 2);
    EXPECT_EQ(cfg.rate, 2);

    // Measurement actually flows with the new settings
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    bool updated{};
    for (int i = 0; i < 50; ++i) {
        unit->update();
        if (unit->updated()) {
            updated = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(updated);

    // The no-argument overload resumes with the settings synced by the call above
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_EQ(unit->config().pressure_oversampling, 32);
}
