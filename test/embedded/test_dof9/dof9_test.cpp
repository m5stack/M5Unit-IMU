/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitDoF9 (composite: BMI270 parent + BMM350 framework child)

  Plug a Unit DoF9 (or DoF10 -- its extra SPA06 is simply left untouched) into the Grove port.
  What this suite verifies beyond the standalone test_bmi270/test_bmm350 is the ENV3-style
  composite wiring itself: ensure_adapter() bus sharing, the framework beginning the child after
  the parent, and the SW1-selectable BMI270 address. Build with -D DOF_BMI270_ADDRESS=0x69 when
  the board's SW1 is flipped (0x68 is the default).
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_dof9.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;

#ifndef DOF_BMI270_ADDRESS
#define DOF_BMI270_ADDRESS 0x68
#endif

class TestDoF9 : public I2CComponentTestBase<UnitDoF9> {
protected:
    virtual UnitDoF9* get_instance() override
    {
        auto ptr = new m5::unit::UnitDoF9(DOF_BMI270_ADDRESS);
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }
};

TEST_F(TestDoF9, ChildrenBegunByFramework)
{
    SCOPED_TRACE(ustr);
    // Units.begin() begins the parent, then each framework child on the shared bus
    EXPECT_TRUE(unit->imu().inPeriodic());
    EXPECT_TRUE(unit->mag().inPeriodic());
}

TEST_F(TestDoF9, ImuGravityThroughSharedAdapter)
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

TEST_F(TestDoF9, MagThroughSharedAdapter)
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

    // Same two-path check as test_bmm350: the adapter's 16 LSB/uT encoding has to scale back to the
    // compensated uT the driver computed. Draining the buffer leaves the newest sample, which is
    // the one latest() returns, so the two numbers describe one measurement and any gap is a
    // scaling bug rather than noise -- what matters here is that going through the composite's
    // shared adapter did not change it
    m5::imu::imu_convert_param_t param{};
    unit->mag().imuConvertParam(param);
    m5::imu::imu_raw_data_t raw{}, last{};
    int checked{};
    while (m5::imu::any(unit->mag().readImuRaw(raw))) {
        last = raw;
        ++checked;
    }
    EXPECT_GT(checked, 0);

    const auto d          = unit->mag().latest();
    const float comp_norm = std::sqrt(d.x_ut() * d.x_ut() + d.y_ut() * d.y_ut() + d.z_ut() * d.z_ut());
    const float mx        = last.mag[0] * param.mag_res;
    const float my        = last.mag[1] * param.mag_res;
    const float mz        = last.mag[2] * param.mag_res;
    const float norm      = std::sqrt(mx * mx + my * my + mz * mz);
    M5_LOGI("mag: comp=%.1fuT readImuRaw*mag_res=%.1fuT (raw=[%ld,%ld,%ld] mag_res=%.5f)", comp_norm, norm,
            (long)last.mag[0], (long)last.mag[1], (long)last.mag[2], param.mag_res);
    EXPECT_TRUE(std::isfinite(norm));
    EXPECT_NEAR(norm, comp_norm, comp_norm * 0.01f + 0.5f);
    // Sanity only; the agreement above is what verifies the scale
    EXPECT_GT(comp_norm, 1.0f);
    EXPECT_LT(comp_norm, 2000.0f);
}

TEST_F(TestDoF9, ChildCapabilities)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(m5::imu::any(unit->imu().imuSpec() & m5::imu::Capability::Accel));
    EXPECT_TRUE(m5::imu::any(unit->imu().imuSpec() & m5::imu::Capability::Gyro));
    // The DoF board's BMM350 sits on the MAIN bus; the BMI270 has no AUX magnetometer here
    EXPECT_FALSE(m5::imu::any(unit->imu().imuSpec() & m5::imu::Capability::Mag));
    EXPECT_TRUE(m5::imu::any(unit->mag().imuSpec() & m5::imu::Capability::Mag));
}
