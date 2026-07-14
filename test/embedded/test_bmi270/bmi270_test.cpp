/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitBMI270

  Default (no TEST_BUILTIN): external wiring via I2CComponentTestBase (GROVE / standard I2C),
  e.g. a UnitDoF6/DoF9 plugged into a host's Grove port.

  With -DTEST_BUILTIN: the board's built-in BMI270 is exercised over M5.In_I2C. The built-in
  BMI270 sits at either 0x68 or 0x69 depending on the board (CoreS3/CoreS3SE: 0x69;
  AtomS3R/StickS3/PaperS3: 0x68), so get_instance() probes both addresses via CHIP_ID (mirrors
  M5Unified IMU_Class::begin()) and begin() wires the unit onto M5.In_I2C.
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_bmi270.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::bmi270;

class TestBMI270 : public I2CComponentTestBase<UnitBMI270> {
protected:
    virtual UnitBMI270* get_instance() override
    {
#if defined(TEST_BUILTIN)
        // The built-in BMI270 sits at a board-dependent address (0x68 or 0x69). Detect it on the
        // internal bus and construct with the result -- no build-time address, and a Capsule
        // (indistinguishable from a bare StampS3 via M5.getBoard()) is handled like any other board.
        uint8_t addr{0x68};
        m5::unit::UnitBMI270::detectAddress(
            [](uint8_t a, uint8_t reg, uint8_t& out) {
                return M5.In_I2C.readRegister(a, reg, &out, 1, 400000);
            },
            addr);
        auto ptr = new m5::unit::UnitBMI270(addr);
#else
        auto ptr = new m5::unit::UnitBMI270();
#endif
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }

#if defined(TEST_BUILTIN)
    // Built-in IMU lives on the internal I2C bus, not the external Grove port.
    bool begin() override
    {
        return Units.add(*unit, M5.In_I2C) && Units.begin();
    }
#endif
};

TEST_F(TestBMI270, BeginStartsPeriodic)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->inPeriodic());
}

TEST_F(TestBMI270, PeriodicMeasurement)
{
    SCOPED_TRACE(ustr);

    // 100Hz 設定なので 100ms あれば複数サンプル溜まる
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

TEST_F(TestBMI270, ReadImuRawPopSemantics)
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

TEST_F(TestBMI270, WriteRange)
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

TEST_F(TestBMI270, AuxMagWhenPresent)
{
    SCOPED_TRACE(ustr);

    // Whether an on-chip BMM150 is wired to the AUX interface depends on the board
    // (CoreS3/AtomS3R: yes; other BMI270 wiring: no). Either way, begin() must have
    // succeeded (fixture requirement) and readImuRaw()'s mag output must agree with
    // what imuSpec() advertises.
    const bool has_mag = m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (int i = 0; i < 10; ++i) {
        unit->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    m5::imu::imu_raw_data_t raw{};
    bool saw_nonzero_mag{false};
    int popped{};
    while (m5::imu::any(unit->readImuRaw(raw))) {
        ++popped;
        if (raw.mag[0] != 0 || raw.mag[1] != 0 || raw.mag[2] != 0) {
            saw_nonzero_mag = true;
        }
    }
    EXPECT_GT(popped, 0);

    if (has_mag) {
        // AUX BMM150 detected: expect a real magnetometer reading in at least one of the
        // popped samples (BMM150 updates at 30Hz; 100ms should yield at least one)
        EXPECT_TRUE(saw_nonzero_mag);
    } else {
        // No AUX BMM150 wired: readImuRaw() never touches out.mag, so it stays at its
        // zero-initialized default across every popped sample
        EXPECT_FALSE(saw_nonzero_mag);
    }
}

TEST_F(TestBMI270, ImuSpec)
{
    SCOPED_TRACE(ustr);

    // Accel/Gyro/Temp are always advertised; Mag depends on whether an AUX BMM150 was
    // detected in begin() (board-dependent, see AuxMagWhenPresent above).
    const auto spec = unit->imuSpec();
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Accel));
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Gyro));
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Temp));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Baro));
}

TEST_F(TestBMI270, Temperature)
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
        EXPECT_LT(celsius, 60.0f);
        ++checked;
    }
    EXPECT_GT(checked, 0);
}

TEST_F(TestBMI270, StopStartPeriodic)
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

TEST_F(TestBMI270, ConfigRoundTrip)
{
    SCOPED_TRACE(ustr);

    auto cfg           = unit->config();
    cfg.start_periodic = false;
    cfg.accel_range    = AccelRange::G4;
    cfg.gyro_range     = GyroRange::DPS500;
    unit->config(cfg);

    const auto cfg2 = unit->config();
    EXPECT_FALSE(cfg2.start_periodic);
    EXPECT_EQ(cfg2.accel_range, cfg.accel_range);
    EXPECT_EQ(cfg2.gyro_range, cfg.gyro_range);
    EXPECT_EQ(cfg2.accel_odr, cfg.accel_odr);
    EXPECT_EQ(cfg2.gyro_odr, cfg.gyro_odr);
    EXPECT_EQ(cfg2.enable_aux_bmm150, cfg.enable_aux_bmm150);
}
