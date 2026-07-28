/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitBMM350

  U231 (GROVE) を想定した wiring (I2CComponentTestBase の既定 begin()) で実行する。
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_bmm350.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::bmm350;

class TestBMM350 : public I2CComponentTestBase<UnitBMM350> {
protected:
    virtual UnitBMM350* get_instance() override
    {
        auto ptr = new m5::unit::UnitBMM350();
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }
};

TEST_F(TestBMM350, BeginStartsPeriodic)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->inPeriodic());
}

TEST_F(TestBMM350, PeriodicMeasurement)
{
    SCOPED_TRACE(ustr);

    // 100Hz 設定なので 200ms あれば複数サンプル溜まる
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

    // Two paths reach the same field: latest()'s compensated uT, and readImuRaw()'s library-wide
    // 16 LSB/uT encoding scaled back by mag_res. readImuRaw() rounds x_ut()*16 and hands out the
    // buffer oldest-first, so draining it leaves the newest sample -- the one latest() returns --
    // and the two can be compared directly. A disagreement is then a scaling bug, not sensor noise
    m5::imu::imu_convert_param_t param{};
    unit->imuConvertParam(param);
    m5::imu::imu_raw_data_t raw{}, last{};
    bool got_raw{false};
    while (m5::imu::any(unit->readImuRaw(raw))) {
        last    = raw;
        got_raw = true;
    }
    EXPECT_TRUE(got_raw);

    const auto d          = unit->latest();
    const float comp_norm = std::sqrt(d.x_ut() * d.x_ut() + d.y_ut() * d.y_ut() + d.z_ut() * d.z_ut());
    const float rx        = last.mag[0] * param.mag_res;
    const float ry        = last.mag[1] * param.mag_res;
    const float rz        = last.mag[2] * param.mag_res;
    const float raw_norm  = std::sqrt(rx * rx + ry * ry + rz * rz);
    // The bounds below are deliberately loose, so the measured value is what tells a reader
    // whether the scale is sane
    M5_LOGI("mag: comp=%.1fuT readImuRaw*mag_res=%.1fuT (raw=[%ld,%ld,%ld] mag_res=%.5f)", comp_norm, raw_norm,
            (long)last.mag[0], (long)last.mag[1], (long)last.mag[2], param.mag_res);
    // Rounding to 1/16uT per axis is the only difference allowed; a wrong scale factor would show
    // up as a ratio, which no tolerance this tight can absorb
    EXPECT_NEAR(raw_norm, comp_norm, comp_norm * 0.01f + 0.5f);
    // Sanity only: hard iron and desk magnets move the absolute value by a lot (over 1000uT has
    // been measured on a steel desk), so this catches an order-of-magnitude error and nothing finer
    EXPECT_GT(comp_norm, 1.0f);
    EXPECT_LT(comp_norm, 2000.0f);
}

TEST_F(TestBMM350, ReadImuRawPopSemantics)
{
    SCOPED_TRACE(ustr);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (int i = 0; i < 10; ++i) {
        unit->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    m5::imu::imu_raw_data_t raw{};
    int popped{};
    while (m5::imu::any(unit->readImuRaw(raw))) {
        ++popped;
    }
    EXPECT_GT(popped, 0);
    // 全消費後は None、latest() は引き続き有効
    EXPECT_FALSE(m5::imu::any(unit->readImuRaw(raw)));
    EXPECT_FALSE(unit->empty());
}

TEST_F(TestBMM350, ReadSettings)
{
    SCOPED_TRACE(ustr);

    // begin() 直後は config_t のデフォルト (DataRate::Rate100Hz / Averaging::X4, clamp_averaging()
    // でも Rate100Hz は X4 まで許容するのでクランプされない) のはず
    DataRate rate{};
    Averaging avg{};
    EXPECT_TRUE(unit->readDataRate(rate));
    EXPECT_EQ(rate, DataRate::Rate100Hz);
    EXPECT_TRUE(unit->readAveraging(avg));
    EXPECT_EQ(avg, Averaging::X4);
}

TEST_F(TestBMM350, ImuSpec)
{
    SCOPED_TRACE(ustr);

    const auto spec = unit->imuSpec();
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Mag));
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Temp));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Accel));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Gyro));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Baro));
}

TEST_F(TestBMM350, Temperature)
{
    SCOPED_TRACE(ustr);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (int i = 0; i < 10; ++i) {
        unit->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
        ++checked;
    }
    EXPECT_GT(checked, 0);
}

TEST_F(TestBMM350, StopStartPeriodic)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    // No updates should occur while stopped
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

TEST_F(TestBMM350, ConfigRoundTrip)
{
    SCOPED_TRACE(ustr);

    auto cfg           = unit->config();
    cfg.start_periodic = false;
    cfg.rate           = DataRate::Rate50Hz;
    cfg.avg            = Averaging::X2;
    unit->config(cfg);

    const auto cfg2 = unit->config();
    EXPECT_FALSE(cfg2.start_periodic);
    EXPECT_EQ(cfg2.rate, cfg.rate);
    EXPECT_EQ(cfg2.avg, cfg.avg);
}

TEST_F(TestBMM350, SleepWakeup)
{
    SCOPED_TRACE(ustr);

    // Suspending while running is allowed; it stops measurement itself
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_TRUE(unit->sleep());
    EXPECT_FALSE(unit->inPeriodic());

    // Nothing is sampled while suspended
    for (int i = 0; i < 10; ++i) {
        unit->update();
        EXPECT_FALSE(unit->updated());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Waking verifies the power mode actually returned to normal; measurement stays stopped
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

TEST_F(TestBMM350, SoftReset)
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

TEST_F(TestBMM350, StartPeriodicWithArgs)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());

    // Non-default settings are written to the chip and synced to config()
    // (Rate50Hz accepts up to X8, so X2 is not clamped)
    EXPECT_TRUE(unit->startPeriodicMeasurement(DataRate::Rate50Hz, Averaging::X2));
    EXPECT_TRUE(unit->inPeriodic());

    DataRate rate{};
    Averaging avg{};
    EXPECT_TRUE(unit->readDataRate(rate));
    EXPECT_EQ(rate, DataRate::Rate50Hz);
    EXPECT_TRUE(unit->readAveraging(avg));
    EXPECT_EQ(avg, Averaging::X2);

    const auto cfg = unit->config();
    EXPECT_EQ(cfg.rate, DataRate::Rate50Hz);
    EXPECT_EQ(cfg.avg, Averaging::X2);

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
    EXPECT_EQ(unit->config().rate, DataRate::Rate50Hz);
}
