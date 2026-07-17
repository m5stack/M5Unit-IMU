/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitDoF10 (composite: BMI270 parent + BMM350 + SPA06 framework children)

  Plug a Unit DoF10 into the Grove port. On top of what test_dof9 verifies (composite wiring,
  shared adapter, SW1 address), this exercises the THREE-child begin sequence including the SPA06
  barometer at 0x76. Build with -D DOF_BMI270_ADDRESS=0x69 when the board's SW1 is flipped
  (0x68 is the default).
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_dof10.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;

#ifndef DOF_BMI270_ADDRESS
#define DOF_BMI270_ADDRESS 0x68
#endif

class TestDoF10 : public I2CComponentTestBase<UnitDoF10> {
protected:
    virtual UnitDoF10* get_instance() override
    {
        auto ptr = new m5::unit::UnitDoF10(DOF_BMI270_ADDRESS);
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }
};

TEST_F(TestDoF10, AllThreeChildrenBegunByFramework)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->imu().inPeriodic());
    EXPECT_TRUE(unit->mag().inPeriodic());
    EXPECT_TRUE(unit->baro().inPeriodic());
}

TEST_F(TestDoF10, ImuGravityThroughSharedAdapter)
{
    SCOPED_TRACE(ustr);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int updated{};
    for (int i = 0; i < 50; ++i) {
        unit->imu().update();
        if (unit->imu().updated()) {
            ++updated;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_GT(updated, 0);

    m5::imu::imu_convert_param_t param{};
    unit->imu().imuConvertParam(param);
    m5::imu::imu_raw_data_t raw{};
    int checked{};
    while (m5::imu::any(unit->imu().readImuRaw(raw))) {
        const float ax   = raw.accel[0] * param.accel_res;
        const float ay   = raw.accel[1] * param.accel_res;
        const float az   = raw.accel[2] * param.accel_res;
        const float norm = std::sqrt(ax * ax + ay * ay + az * az);
        EXPECT_TRUE(std::isfinite(norm));
        EXPECT_GT(norm, 0.5f);  // gravity, device at rest
        EXPECT_LT(norm, 1.5f);
        ++checked;
    }
    EXPECT_GT(checked, 0);
}

TEST_F(TestDoF10, BaroThroughSharedAdapter)
{
    SCOPED_TRACE(ustr);

    // Default barometer rate is 16 samples/s
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int updated{};
    for (int i = 0; i < 50; ++i) {
        unit->baro().update();
        if (unit->baro().updated()) {
            ++updated;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_GT(updated, 0);

    const float hpa = unit->baro().pressure();
    EXPECT_TRUE(std::isfinite(hpa));
    EXPECT_GT(hpa, 300.0f);
    EXPECT_LT(hpa, 1100.0f);

    const float celsius = unit->baro().temperature();
    EXPECT_TRUE(std::isfinite(celsius));
    EXPECT_GT(celsius, -20.0f);
    EXPECT_LT(celsius, 60.0f);
}

TEST_F(TestDoF10, MagThroughSharedAdapter)
{
    SCOPED_TRACE(ustr);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int updated{};
    for (int i = 0; i < 50; ++i) {
        unit->mag().update();
        if (unit->mag().updated()) {
            ++updated;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_GT(updated, 0);

    m5::imu::imu_convert_param_t param{};
    unit->mag().imuConvertParam(param);
    m5::imu::imu_raw_data_t raw{};
    int checked{};
    while (m5::imu::any(unit->mag().readImuRaw(raw))) {
        const float mx   = raw.mag[0] * param.mag_res;
        const float my   = raw.mag[1] * param.mag_res;
        const float mz   = raw.mag[2] * param.mag_res;
        const float norm = std::sqrt(mx * mx + my * my + mz * mz);
        EXPECT_TRUE(std::isfinite(norm));
        EXPECT_GT(norm, 5.0f);
        EXPECT_LT(norm, 1500.0f);
        ++checked;
    }
    EXPECT_GT(checked, 0);
}
