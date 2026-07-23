/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitMPU6886
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_mpu6886.hpp>
#if defined(TEST_PORT_C)
#include <wiring/m5_unit_imu_wiring.hpp>
#endif
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::mpu6886;

class TestMPU6886 : public I2CComponentTestBase<UnitMPU6886> {
protected:
    virtual UnitMPU6886* get_instance() override
    {
        auto ptr = new m5::unit::UnitMPU6886();
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }

#if defined(TEST_BUILTIN)
    // With -DTEST_BUILTIN the board's built-in MPU6886 (0x68) is exercised over the internal
    // I2C bus instead of the external Grove port.
    bool begin() override
    {
        return Units.add(*unit, M5.In_I2C) && Units.begin();
    }
#elif defined(TEST_PORT_C)
    // With -DTEST_PORT_C the external unit is routed to GROVE Port C on a classic Core (needs the
    // M5GO/Fire base), exercising this library's i2cPortC() lgfx-port-1 rebind path -- the only way
    // an external 0x68 unit can coexist with the internal bus there (Port A IS the internal bus).
    bool begin() override
    {
        return m5::unit::imu::wiring::i2cPortC(Units, *unit) && Units.begin();
    }
#endif
};

TEST_F(TestMPU6886, BeginStartsPeriodic)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->inPeriodic());
}

TEST_F(TestMPU6886, PeriodicMeasurement)
{
    SCOPED_TRACE(ustr);

    // 250Hz 設定なので 100ms あれば複数サンプル溜まる
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
    const auto d = unit->latest();
    // 静置想定: accel の合成が概ね 1G (±8G レンジで 4096 LSB/G)
    const float ax = d.accel_x() / 4096.0f;
    const float ay = d.accel_y() / 4096.0f;
    const float az = d.accel_z() / 4096.0f;
    const float g  = std::sqrt(ax * ax + ay * ay + az * az);
    EXPECT_NEAR(g, 1.0f, 0.3f);
}

TEST_F(TestMPU6886, ReadImuRawPopSemantics)
{
    SCOPED_TRACE(ustr);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    for (int i = 0; i < 10; ++i) {
        unit->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
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

TEST_F(TestMPU6886, WriteRange)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->writeAccelRange(AccelRange::G4));
    EXPECT_TRUE(unit->writeGyroRange(GyroRange::DPS500));
    m5::imu::imu_convert_param_t p{};
    unit->imuConvertParam(p);
    EXPECT_FLOAT_EQ(p.accel_res, 4.0f / 32768);
    EXPECT_FLOAT_EQ(p.gyro_res, 500.0f / 32768);

    AccelRange ar{};
    GyroRange gr{};
    EXPECT_TRUE(unit->readAccelRange(ar));
    EXPECT_EQ(ar, AccelRange::G4);
    EXPECT_TRUE(unit->readGyroRange(gr));
    EXPECT_EQ(gr, GyroRange::DPS500);
}

TEST_F(TestMPU6886, ImuSpec)
{
    SCOPED_TRACE(ustr);

    const auto spec = unit->imuSpec();
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Accel));
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Gyro));
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Temp));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Mag));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Baro));
}

TEST_F(TestMPU6886, Temperature)
{
    SCOPED_TRACE(ustr);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (int i = 0; i < 10; ++i) {
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
        EXPECT_LT(celsius, 100.0f);  // wide sanity ceiling: die self-heating in an enclosed host
        ++checked;
    }
    EXPECT_GT(checked, 0);
}

TEST_F(TestMPU6886, StopStartPeriodic)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    // No updates should occur while stopped
    for (int i = 0; i < 10; ++i) {
        unit->update();
        EXPECT_FALSE(unit->updated());
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bool updated{};
    for (int i = 0; i < 50; ++i) {
        unit->update();
        if (unit->updated()) {
            updated = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_TRUE(updated);
}

TEST_F(TestMPU6886, SleepWakeup)
{
    SCOPED_TRACE(ustr);

    // Unlike softReset(), sleeping while running is allowed; it stops measurement itself
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_TRUE(unit->sleep());
    EXPECT_FALSE(unit->inPeriodic());

    // Nothing is sampled while asleep
    for (int i = 0; i < 10; ++i) {
        unit->update();
        EXPECT_FALSE(unit->updated());
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Waking only powers the chip back up; measurement deliberately stays stopped
    EXPECT_TRUE(unit->wakeup());
    EXPECT_FALSE(unit->inPeriodic());
    for (int i = 0; i < 10; ++i) {
        unit->update();
        EXPECT_FALSE(unit->updated());
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Sampling resumes only once the caller restarts it explicitly
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bool updated{};
    for (int i = 0; i < 50; ++i) {
        unit->update();
        if (unit->updated()) {
            updated = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_TRUE(updated);
}

TEST_F(TestMPU6886, SleepWhileStoppedAndRedundantWakeup)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->sleep());   // already stopped: still sleeps
    EXPECT_TRUE(unit->wakeup());
    EXPECT_TRUE(unit->wakeup());  // waking an awake chip is harmless

    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bool updated{};
    for (int i = 0; i < 50; ++i) {
        unit->update();
        if (unit->updated()) {
            updated = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_TRUE(updated);
}

TEST_F(TestMPU6886, ConfigRoundTrip)
{
    SCOPED_TRACE(ustr);

    auto cfg            = unit->config();
    cfg.start_periodic  = false;
    cfg.accel_range     = AccelRange::G4;
    cfg.gyro_range      = GyroRange::DPS500;
    cfg.sample_rate_div = 7;
    unit->config(cfg);

    const auto cfg2 = unit->config();
    EXPECT_FALSE(cfg2.start_periodic);
    EXPECT_EQ(cfg2.accel_range, cfg.accel_range);
    EXPECT_EQ(cfg2.gyro_range, cfg.gyro_range);
    EXPECT_EQ(cfg2.dlpf, cfg.dlpf);
    EXPECT_EQ(cfg2.sample_rate_div, cfg.sample_rate_div);
}

TEST_F(TestMPU6886, SoftReset)
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

TEST_F(TestMPU6886, StartPeriodicWithArgs)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());

    // Non-default settings are written to the chip and synced to config()
    EXPECT_TRUE(unit->startPeriodicMeasurement(AccelRange::G4, GyroRange::DPS500, Dlpf::BW92, 7));
    EXPECT_TRUE(unit->inPeriodic());

    AccelRange ar{};
    GyroRange gr{};
    EXPECT_TRUE(unit->readAccelRange(ar));
    EXPECT_EQ(ar, AccelRange::G4);
    EXPECT_TRUE(unit->readGyroRange(gr));
    EXPECT_EQ(gr, GyroRange::DPS500);

    const auto cfg = unit->config();
    EXPECT_EQ(cfg.accel_range, AccelRange::G4);
    EXPECT_EQ(cfg.gyro_range, GyroRange::DPS500);
    EXPECT_EQ(cfg.dlpf, Dlpf::BW92);
    EXPECT_EQ(cfg.sample_rate_div, 7);

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
    EXPECT_EQ(unit->config().sample_rate_div, 7);
}
