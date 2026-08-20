/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitDoF6 (U228), which is a plain alias of UnitBMI270

  Plug a Unit DoF6 into the Grove port. Build with -D DOF_BMI270_ADDRESS=0x69 when the board's SW1
  is flipped (0x68 is the default).

  Why this exists separately from test_bmi270: that suite builds UnitBMI270AuxBMM150, so its probe
  switches the BMI270's AUX interface on -- which is right for a Unit Mini IMU-Pro (U171), whose
  BMM150 really is wired there, but is not how the library drives a DoF6. M5UnitUnifiedIMU.hpp
  aliases UnitDoF6 to the plain UnitBMI270 because a DoF6 has no magnetometer to reach, so a DoF6
  belongs on that class, and that is what this suite instantiates. (Whether enabling AUX on a DoF6 is
  actively harmful is untested: it is documented for DoF9/DoF10, whose AUX pins share the main bus,
  and the DoF6 is the same board family -- so not enabling it is simply the matching choice.)

  Coverage-wise the accel/gyro API is already exercised on the built-in boards through the derived
  class; what only runs here is UnitBMI270 itself over external wiring -- its own imuSpec() (which
  must never claim Mag) and its own 12-byte burst read.
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <M5UnitUnifiedIMU.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::bmi270;

#ifndef DOF_BMI270_ADDRESS
#define DOF_BMI270_ADDRESS 0x68
#endif

class TestDoF6 : public I2CComponentTestBase<UnitDoF6> {
protected:
    virtual UnitDoF6* get_instance() override
    {
        auto ptr = new m5::unit::UnitDoF6(DOF_BMI270_ADDRESS);
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }
};

TEST_F(TestDoF6, BeginStartsPeriodic)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->inPeriodic());
}

TEST_F(TestDoF6, ImuSpecNeverClaimsMag)
{
    SCOPED_TRACE(ustr);

    // UnitBMI270's own imuSpec(), not the derived class's: the AUX interface is left alone here, so
    // Mag must not appear no matter what is soldered to it
    const auto spec = unit->imuSpec();
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Accel));
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Gyro));
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Temp));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Mag));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Baro));
}

TEST_F(TestDoF6, PeriodicMeasurement)
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
        const float ax   = raw.accel[0] * param.accel_res;
        const float ay   = raw.accel[1] * param.accel_res;
        const float az   = raw.accel[2] * param.accel_res;
        const float norm = std::sqrt(ax * ax + ay * ay + az * az);
        EXPECT_TRUE(std::isfinite(norm));
        // Gravity, device at rest -- this is what proves the 12-byte burst landed in the right slots
        EXPECT_GT(norm, 0.5f);
        EXPECT_LT(norm, 1.5f);
        // The mag slots are untouched by the base class, so they stay at their initialised zero
        EXPECT_EQ(raw.mag[0], 0);
        EXPECT_EQ(raw.mag[1], 0);
        EXPECT_EQ(raw.mag[2], 0);
        ++checked;
    }
    EXPECT_GT(checked, 0);
}

TEST_F(TestDoF6, Temperature)
{
    SCOPED_TRACE(ustr);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (int i = 0; i < 20; ++i) {
        unit->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    m5::imu::imu_convert_param_t param{};
    unit->imuConvertParam(param);
    m5::imu::imu_raw_data_t raw{};
    int checked{};
    while (m5::imu::any(unit->readImuRaw(raw))) {
        const float celsius = raw.temp * param.temp_res + param.temp_offset;
        EXPECT_TRUE(std::isfinite(celsius));
        EXPECT_GT(celsius, 0.0f);
        EXPECT_LT(celsius, 60.0f);
        if (!checked) {
            M5_LOGI("temperature: %.2fC", celsius);
        }
        ++checked;
    }
    EXPECT_GT(checked, 0);
}

TEST_F(TestDoF6, WriteRangeAndOutputDataRate)
{
    SCOPED_TRACE(ustr);

    // Range and ODR are refused while running, then round-trip through the registers once stopped
    EXPECT_FALSE(unit->writeAccelRange(AccelRange::G4));
    EXPECT_FALSE(unit->writeGyroODR(GyroODR::Rate50Hz));

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->writeAccelRange(AccelRange::G4));
    EXPECT_TRUE(unit->writeGyroRange(GyroRange::DPS500));
    EXPECT_TRUE(unit->writeAccelODR(AccelODR::Rate50Hz));
    EXPECT_TRUE(unit->writeGyroODR(GyroODR::Rate50Hz));

    AccelRange arange{};
    GyroRange grange{};
    AccelODR aodr{};
    GyroODR godr{};
    EXPECT_TRUE(unit->readAccelRange(arange));
    EXPECT_TRUE(unit->readGyroRange(grange));
    EXPECT_TRUE(unit->readAccelODR(aodr));
    EXPECT_TRUE(unit->readGyroODR(godr));
    EXPECT_EQ(arange, AccelRange::G4);
    EXPECT_EQ(grange, GyroRange::DPS500);
    EXPECT_EQ(aodr, AccelODR::Rate50Hz);
    EXPECT_EQ(godr, GyroODR::Rate50Hz);

    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_EQ(unit->interval(), odr_interval_ms(GyroODR::Rate50Hz));
}

TEST_F(TestDoF6, SleepWakeup)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_TRUE(unit->sleep());
    EXPECT_FALSE(unit->inPeriodic());
    for (int i = 0; i < 10; ++i) {
        unit->update();
        EXPECT_FALSE(unit->updated());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(unit->wakeup());
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bool updated{};
    for (int i = 0; i < 50 && !updated; ++i) {
        unit->update();
        updated = unit->updated();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(updated);
}

TEST_F(TestDoF6, SoftReset)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_FALSE(unit->softReset());

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->softReset());
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bool updated{};
    for (int i = 0; i < 50 && !updated; ++i) {
        unit->update();
        updated = unit->updated();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(updated);
}
