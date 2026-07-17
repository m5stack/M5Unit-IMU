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
