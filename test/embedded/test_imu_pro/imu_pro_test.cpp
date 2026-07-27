/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitMiniIMUPro (composite: BMI270 + BMP280 framework children)

  Plug a Unit Mini IMU-Pro (U171) into the Grove port. Unlike test_dof10, only the BMI270 and the
  BMP280 (0x76, driver from M5Unit-ENV) sit on the main bus: the board's BMM150 is wired to the
  BMI270's AUX interface, so the magnetometer arrives through imu(), not through a child of its
  own. The BMI270 address is fixed at 0x68 (no SW1 on this board).
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_mini_imu_pro.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;

class TestMiniIMUPro : public I2CComponentTestBase<UnitMiniIMUPro> {
protected:
    virtual UnitMiniIMUPro* get_instance() override
    {
        auto ptr = new m5::unit::UnitMiniIMUPro();
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }
};

TEST_F(TestMiniIMUPro, ChildrenBegunByFramework)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->imu().inPeriodic());
    EXPECT_TRUE(unit->baro().inPeriodic());
}

TEST_F(TestMiniIMUPro, ImuGravityThroughSharedAdapter)
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

TEST_F(TestMiniIMUPro, BaroThroughSharedAdapter)
{
    SCOPED_TRACE(ustr);

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

    const float hpa = unit->baro().pressure() / 100.0f;  // pressure() is Pa; the range below is hPa
    EXPECT_TRUE(std::isfinite(hpa));
    EXPECT_GT(hpa, 300.0f);
    EXPECT_LT(hpa, 1100.0f);

    const float celsius = unit->baro().temperature();
    EXPECT_TRUE(std::isfinite(celsius));
    EXPECT_GT(celsius, -20.0f);
    EXPECT_LT(celsius, 60.0f);
}

TEST_F(TestMiniIMUPro, MagThroughAuxBridge)
{
    SCOPED_TRACE(ustr);

    // The BMM150 is an AUX target of the BMI270, so its samples ride along with the accel/gyro
    // burst rather than arriving from a magnetometer child of their own
    const bool has_mag = m5::imu::any(unit->imu().imuSpec() & m5::imu::Capability::Mag);
    EXPECT_TRUE(has_mag) << "AUX BMM150 was not detected; check the BMI270 AUX wiring";
    if (!has_mag) {
        return;  // Nothing further to assert without a magnetometer
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int updated{};
    for (int i = 0; i < 50; ++i) {
        unit->imu().update();
        if (unit->imu().updated()) {
            ++updated;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_GT(updated, 0);

    m5::imu::imu_convert_param_t param{};
    unit->imu().imuConvertParam(param);
    m5::imu::imu_raw_data_t raw{};
    int checked{};
    while (m5::imu::any(unit->imu().readImuRaw(raw))) {
        const float mx   = raw.mag[0] * param.mag_res;
        const float my   = raw.mag[1] * param.mag_res;
        const float mz   = raw.mag[2] * param.mag_res;
        const float norm = std::sqrt(mx * mx + my * my + mz * mz);
        EXPECT_TRUE(std::isfinite(norm));
        EXPECT_GT(norm, 5.0f);     // geomagnetic floor
        EXPECT_LT(norm, 1500.0f);  // generous hard-iron/desk-magnet ceiling
        ++checked;
    }
    EXPECT_GT(checked, 0);
}

TEST_F(TestMiniIMUPro, ChildCapabilities)
{
    SCOPED_TRACE(ustr);
    const auto spec = unit->imu().imuSpec();
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Accel));
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Gyro));
    // The board's BMM150 hangs off the BMI270's AUX interface, so the accel/gyro unit is the one
    // that advertises Mag -- there is no separate magnetometer child on the main bus
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Mag));
}
