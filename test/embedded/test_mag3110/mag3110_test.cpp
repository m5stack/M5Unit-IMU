/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitMAG3110

  With -DTEST_BUILTIN the MAG3110 on an original (2017) classic M5Stack Gray is exercised over
  M5.In_I2C. It sits directly on the internal bus at 0x0E (no aux bypass, unlike the AK8963). Later
  Gray revisions (MPU9250, then MPU6886 + BMM150) have no MAG3110, so this env only passes on the
  original 2017 hardware.
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_mag3110.hpp>
#include <imu/mag_detect.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;

class TestMAG3110 : public I2CComponentTestBase<UnitMAG3110> {
protected:
    virtual UnitMAG3110* get_instance() override
    {
        auto ptr = new m5::unit::UnitMAG3110();
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }

#if defined(TEST_BUILTIN)
    bool begin() override
    {
        return Units.add(*unit, M5.In_I2C) && Units.begin();
    }
#endif
};

#if defined(TEST_BUILTIN)
TEST_F(TestMAG3110, DetectReportsMag3110)
{
    SCOPED_TRACE(ustr);
    const auto kind = m5::unit::imu::detectBuiltinMag(
        [](uint8_t a, uint8_t r, uint8_t& o) { return M5.In_I2C.readRegister(a, r, &o, 1, 400000); });
    EXPECT_EQ(kind, m5::unit::imu::MagKind::MAG3110);
}
#endif

TEST_F(TestMAG3110, BeginStartsPeriodic)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->inPeriodic());
}

TEST_F(TestMAG3110, ImuSpecIsMag)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag));
    EXPECT_FALSE(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Accel));
}

TEST_F(TestMAG3110, PeriodicMeasurement)
{
    SCOPED_TRACE(ustr);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

    m5::imu::imu_convert_param_t param{};
    unit->imuConvertParam(param);

    m5::imu::imu_raw_data_t raw{};
    int checked{};
    while (m5::imu::any(unit->readImuRaw(raw))) {
        const float mx   = raw.mag[0] * param.mag_res;
        const float my   = raw.mag[1] * param.mag_res;
        const float mz   = raw.mag[2] * param.mag_res;
        const float norm = std::sqrt(mx * mx + my * my + mz * mz);
        EXPECT_TRUE(std::isfinite(norm));
        EXPECT_GT(norm, 10.0f);
        EXPECT_LT(norm, 200.0f);  // built-in mag: allow mild hard-iron (cf. AK8963/BMM150 built-in tests)
        ++checked;
    }
    EXPECT_GT(checked, 0);
}

TEST_F(TestMAG3110, SleepWakeup)
{
    SCOPED_TRACE(ustr);

    // Entering STANDBY while running is allowed; it stops measurement itself
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_TRUE(unit->sleep());
    EXPECT_FALSE(unit->inPeriodic());

    // Nothing is sampled while in STANDBY
    for (int i = 0; i < 10; ++i) {
        unit->update();
        EXPECT_FALSE(unit->updated());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // wakeup() is a no-op on this chip (STANDBY is the configurable state)
    EXPECT_TRUE(unit->wakeup());
    EXPECT_FALSE(unit->inPeriodic());

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

TEST_F(TestMAG3110, SoftReset)
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

TEST_F(TestMAG3110, WriteDataRate)
{
    SCOPED_TRACE(ustr);

    mag3110::DataRate org_rate{};
    mag3110::OverSampling org_os{};
    EXPECT_TRUE(unit->readDataRate(org_rate, org_os));
    // begin() seeds the live state from config_t, and REG_CTRL_REG1 has to agree with it
    EXPECT_EQ(org_rate, unit->config().rate);
    EXPECT_EQ(org_os, unit->config().over_sampling);

    // Changing the rate changes the measurement interval, so it is refused while running
    EXPECT_FALSE(unit->writeDataRate(mag3110::DataRate::Rate10Hz, mag3110::OverSampling::X64));
    mag3110::DataRate rate{};
    mag3110::OverSampling os{};
    EXPECT_TRUE(unit->readDataRate(rate, os));
    EXPECT_EQ(rate, org_rate);
    EXPECT_EQ(os, org_os);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());

    // Every DataRate/OverSampling pair is valid on this chip (datasheet Table 32 lists all 32), and
    // each has to survive the register round trip -- REG_CTRL_REG1 only accepts these fields while
    // AC is clear, so a pair that reads back wrong means the standby handshake did not happen
    for (uint8_t r = 0; r < 8; ++r) {
        for (uint8_t o = 0; o < 4; ++o) {
            const auto want_rate = static_cast<mag3110::DataRate>(r);
            const auto want_os   = static_cast<mag3110::OverSampling>(o);
            EXPECT_TRUE(unit->writeDataRate(want_rate, want_os)) << "DR " << (int)r << " OS " << (int)o;
            EXPECT_TRUE(unit->readDataRate(rate, os)) << "DR " << (int)r << " OS " << (int)o;
            EXPECT_EQ(rate, want_rate);
            EXPECT_EQ(os, want_os);
        }
    }

    // The slowest setting really does stretch the interval; without that update() would keep
    // hammering the bus at the 13ms of the fastest one
    EXPECT_TRUE(unit->writeDataRate(mag3110::DataRate::Rate0_63Hz, mag3110::OverSampling::X128));
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_EQ(unit->interval(), mag3110::interval_ms(mag3110::DataRate::Rate0_63Hz, mag3110::OverSampling::X128));
    EXPECT_TRUE(unit->stopPeriodicMeasurement());

    // A rate given to startPeriodicMeasurement() applies without a separate write, and measurement
    // actually flows at it
    EXPECT_TRUE(unit->startPeriodicMeasurement(mag3110::DataRate::Rate20Hz, mag3110::OverSampling::X16));
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_TRUE(unit->readDataRate(rate, os));
    EXPECT_EQ(rate, mag3110::DataRate::Rate20Hz);
    EXPECT_EQ(os, mag3110::OverSampling::X16);
    EXPECT_EQ(unit->interval(), mag3110::interval_ms(mag3110::DataRate::Rate20Hz, mag3110::OverSampling::X16));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    bool updated{};
    for (int i = 0; i < 50 && !updated; ++i) {
        unit->update();
        updated = unit->updated();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(updated);

    // Restore what begin() had set up for the tests that follow
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->writeDataRate(org_rate, org_os));
    EXPECT_TRUE(unit->startPeriodicMeasurement());
}
