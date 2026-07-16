/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitSH200Q

  With -DTEST_BUILTIN the SH200Q on the earliest (2019.3) M5StickC -- or on a 2019.7 M5Stack Fire
  -- is exercised over M5.In_I2C at 0x6C. Other revisions of both boards carry different chips, so
  the SH200Q envs only pass on those original lots. The gyro check is deliberately loose: the
  SH200Q gyro is known to have large per-axis zero offsets.
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_sh200q.hpp>
#include <imu/imu_detect.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;

class TestSH200Q : public I2CComponentTestBase<UnitSH200Q> {
protected:
    virtual UnitSH200Q* get_instance() override
    {
        auto ptr = new m5::unit::UnitSH200Q();
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
TEST_F(TestSH200Q, DetectReportsSh200q)
{
    SCOPED_TRACE(ustr);
    const auto kind = m5::unit::imu::detectBuiltinImu(
        [](uint8_t a, uint8_t r, uint8_t& o) { return M5.In_I2C.readRegister(a, r, &o, 1, 400000); });
    EXPECT_EQ(kind, m5::unit::imu::ImuKind::SH200Q);
}
#endif

TEST_F(TestSH200Q, BeginStartsPeriodic)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->inPeriodic());
}

TEST_F(TestSH200Q, ImuSpecIsAccelGyroTemp)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Accel));
    EXPECT_TRUE(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Gyro));
    EXPECT_TRUE(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Temp));
    EXPECT_FALSE(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag));
}

TEST_F(TestSH200Q, RangeReadback)
{
    SCOPED_TRACE(ustr);
    // begin() wrote the defaults (8G / 2000dps); read them back from the device
    sh200q::AccelRange ar{};
    sh200q::GyroRange gr{};
    EXPECT_TRUE(unit->readAccelRange(ar));
    EXPECT_TRUE(unit->readGyroRange(gr));
    EXPECT_EQ(ar, sh200q::AccelRange::G8);
    EXPECT_EQ(gr, sh200q::GyroRange::DPS2000);
}

TEST_F(TestSH200Q, PeriodicMeasurement)
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
        EXPECT_GT(norm, 0.5f);  // gravity, device at rest
        EXPECT_LT(norm, 1.5f);

        // Loose sanity only: the SH200Q gyro has notoriously large zero offsets
        const float gx = raw.gyro[0] * param.gyro_res;
        const float gy = raw.gyro[1] * param.gyro_res;
        const float gz = raw.gyro[2] * param.gyro_res;
        EXPECT_TRUE(std::isfinite(gx));
        EXPECT_TRUE(std::isfinite(gy));
        EXPECT_TRUE(std::isfinite(gz));
        EXPECT_LT(std::fabs(gx), 200.0f);
        EXPECT_LT(std::fabs(gy), 200.0f);
        EXPECT_LT(std::fabs(gz), 200.0f);

        const float temp = raw.temp * param.temp_res + param.temp_offset;
        EXPECT_GT(temp, -20.0f);
        EXPECT_LT(temp, 80.0f);
        ++checked;
    }
    EXPECT_GT(checked, 0);
}
