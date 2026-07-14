/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitMPU6050
  Built-in on the original (2017) M5Stack Gray (MPU6050 + MAG3110); with -DTEST_BUILTIN the
  board's built-in MPU6050 (0x68, WHO_AM_I 0x68) is exercised over the internal I2C bus.
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_mpu6050.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::mpu6886;

class TestMPU6050 : public I2CComponentTestBase<UnitMPU6050> {
protected:
    virtual UnitMPU6050* get_instance() override
    {
        auto ptr = new m5::unit::UnitMPU6050();
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

TEST_F(TestMPU6050, BeginStartsPeriodic)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->inPeriodic());
}

TEST_F(TestMPU6050, PeriodicMeasurement)
{
    SCOPED_TRACE(ustr);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int updated{};
    for (int i = 0; i < 50; ++i) {
        unit->update();
        if (unit->updated()) {
            ++updated;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_GT(updated, 0);
    EXPECT_FALSE(unit->empty());
    const auto d   = unit->latest();
    const float ax = d.accel_x() / 4096.0f;
    const float ay = d.accel_y() / 4096.0f;
    const float az = d.accel_z() / 4096.0f;
    const float g  = std::sqrt(ax * ax + ay * ay + az * az);
    EXPECT_NEAR(g, 1.0f, 0.3f);
}

TEST_F(TestMPU6050, WriteRange)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->writeAccelRange(AccelRange::G4));
    EXPECT_TRUE(unit->writeGyroRange(GyroRange::DPS500));
    m5::imu::imu_convert_param_t p{};
    unit->imuConvertParam(p);
    EXPECT_FLOAT_EQ(p.accel_res, 4.0f / 32768);
    EXPECT_FLOAT_EQ(p.gyro_res, 500.0f / 32768);
}

TEST_F(TestMPU6050, ImuSpec)
{
    SCOPED_TRACE(ustr);

    const auto spec = unit->imuSpec();
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Accel));
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Gyro));
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Temp));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Mag));
}

TEST_F(TestMPU6050, Temperature)
{
    SCOPED_TRACE(ustr);

    m5::imu::imu_convert_param_t param{};
    unit->imuConvertParam(param);
    // MPU6050 temperature scale: 1/340 degC/LSB, +36.53 degC (per the datasheet).
    EXPECT_FLOAT_EQ(param.temp_res, 1.0f / 340.0f);
    EXPECT_FLOAT_EQ(param.temp_offset, 36.53f);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (int i = 0; i < 10; ++i) {
        unit->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    m5::imu::imu_raw_data_t raw{};
    int checked{};
    while (m5::imu::any(unit->readImuRaw(raw))) {
        const float celsius = raw.temp * param.temp_res + param.temp_offset;
        EXPECT_TRUE(std::isfinite(celsius));
        EXPECT_GT(celsius, 0.0f);
        EXPECT_LT(celsius, 100.0f);
        ++checked;
    }
    EXPECT_GT(checked, 0);
}
