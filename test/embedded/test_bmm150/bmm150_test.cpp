/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitBMM150

  There is no standalone BMM150 Grove unit, so by default this test targets an external BMM150
  reachable over the standard I2C wiring (I2CComponentTestBase's default begin()).

  With -DTEST_BUILTIN the discrete built-in BMM150 (0x10) found on classic M5Stack Core/Fire
  (board_M5Stack) is exercised over M5.In_I2C. Note CoreS3/AtomS3R expose their magnetometer via
  the BMI270 AUX interface instead, which is covered by test_bmi270's AuxMagWhenPresent.
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_bmm150.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::bmm150;

class TestBMM150 : public I2CComponentTestBase<UnitBMM150> {
protected:
    virtual UnitBMM150* get_instance() override
    {
        auto ptr = new m5::unit::UnitBMM150();
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }

#if defined(TEST_BUILTIN)
    // With -DTEST_BUILTIN the board's discrete built-in BMM150 (0x10, e.g. classic Core/Fire) is
    // exercised over the internal I2C bus instead of the external Grove port.
    bool begin() override
    {
        return Units.add(*unit, M5.In_I2C) && Units.begin();
    }
#endif
};

TEST_F(TestBMM150, BeginStartsPeriodic)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->inPeriodic());
}

TEST_F(TestBMM150, PeriodicMeasurement)
{
    SCOPED_TRACE(ustr);

    // 30Hz 設定なので 200ms あれば複数サンプル溜まる
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

    // Reject the overflow sentinel: comp_x()/comp_y()/comp_z() return bmm150::OVERFLOW_OUTPUT (-32768)
    // for a not-ready or invalid read, which would otherwise leak -2048uT per axis into the norm (the
    // source of the earlier flaky ~2049uT reading). Poll for a fully valid sample and assert on that.

    float norm     = 0.0f;
    bool got_valid = false;
    for (int i = 0; i < 50 && !got_valid; ++i) {
        const auto d = unit->latest();
        if (d.comp_x() != OVERFLOW_OUTPUT && d.comp_y() != OVERFLOW_OUTPUT && d.comp_z() != OVERFLOW_OUTPUT) {
            // comp_x/y/z() are integer uT (1 LSB/uT), not 16 LSB/uT: rotating a board through every
            // orientation traces a sphere whose radius is the local geomagnetic field, and that radius
            // measured ~36 in these very units. adj_x/y/z() rescales them to the library-wide 16 LSB/uT.
            const float mx = d.comp_x();
            const float my = d.comp_y();
            const float mz = d.comp_z();
            norm           = std::sqrt(mx * mx + my * my + mz * mz);
            got_valid      = true;
            break;
        }
        unit->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(got_valid);
    EXPECT_TRUE(std::isfinite(norm));

    // Only a sanity range here -- the real scaling guard is the comp vs readImuRaw comparison below.
    // A magnitude band cannot do that job: hard iron cancels much of the geomagnetic field at some
    // orientations (an AtomS3R measured under 15uT), while a base plate carrying a magnet pushed a
    // Gray past 1200uT, so any floor tight enough to catch a 16x error would be flaky.
    EXPECT_GT(norm, 1.0f);
#if defined(TEST_BUILTIN)
    EXPECT_LT(norm, 2000.0f);
#else
    EXPECT_LT(norm, 500.0f);
#endif

    // readImuRaw() has to expose that same field in the library-wide 16 LSB/uT scale (mag_res = 1/16),
    // so multiplying it back out must land in the same physical band. This is what catches a regression
    // in out.mag itself: storing the compensated uT unscaled would divide this norm by 16.
    m5::imu::imu_convert_param_t cp{};
    unit->imuConvertParam(cp);
    m5::imu::imu_raw_data_t raw{};
    bool got_raw{false};
    for (int i = 0; i < 50 && !got_raw; ++i) {
        while (m5::imu::any(unit->readImuRaw(raw))) {
            if (raw.mag[0] != OVERFLOW_OUTPUT && raw.mag[1] != OVERFLOW_OUTPUT && raw.mag[2] != OVERFLOW_OUTPUT) {
                got_raw = true;
                break;
            }
        }
        if (!got_raw) {
            unit->update();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    EXPECT_TRUE(got_raw);
    if (got_raw) {
        const float rx       = raw.mag[0] * cp.mag_res;
        const float ry       = raw.mag[1] * cp.mag_res;
        const float rz       = raw.mag[2] * cp.mag_res;
        const float raw_norm = std::sqrt(rx * rx + ry * ry + rz * rz);
        // Log the measured field: the bounds above are deliberately loose (hard iron varies per host
        // and per desk), so the actual number is what tells a reader whether the scale is sane
        M5_LOGI("mag: comp=%.1fuT readImuRaw*mag_res=%.1fuT", norm, raw_norm);
        // Both are uT by construction (adj_* scales comp by 16, mag_res scales it back), so they have
        // to agree to within sensor noise -- a mismatch means one of those two factors is wrong
        EXPECT_NEAR(raw_norm, norm, norm * 0.1f + 3.0f);
        EXPECT_GT(raw_norm, 15.0f);
    }
}

#if defined(TEST_BUILTIN)
// Built-in regression: the factory trim is only read from a real chip in this path (the native oracle
// test injects trim directly), so verify read_trim() over the internal bus returns the non-zero
// radius-hall coefficients the X/Y/Z compensation divides by. A zero dig_xyz1/dig_z1 (failed trim
// read) is the classic cause of wildly inflated output. The full trim+compensation chain was
// cross-checked against the BMM150 oracle on Gray (comp x=-1158 y=-973 z=359 for a captured sample).
TEST_F(TestBMM150, BuiltInTrimReadNonZero)
{
    SCOPED_TRACE(ustr);

    // The 0x68 burst (parsed as in UnitBMM150::read_trim) carries dig_z1 and dig_xyz1; read it directly
    // over the internal bus (trim is OTP/stable, so this matches what begin() loaded).
    uint8_t b2[10]{};
    ASSERT_TRUE(M5.In_I2C.readRegister(0x10, 0x68, b2, sizeof(b2), 400000));
    const unsigned dig_z1   = static_cast<uint16_t>((b2[3] << 8) | b2[2]);
    const unsigned dig_xyz1 = static_cast<uint16_t>(((b2[5] << 8) | b2[4]) & 0x3FFF);

    EXPECT_GT(dig_xyz1, 0u);
    EXPECT_GT(dig_z1, 0u);
}
#endif

TEST_F(TestBMM150, ReadImuRawPopSemantics)
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

TEST_F(TestBMM150, ReadSettings)
{
    SCOPED_TRACE(ustr);

    // begin() 直後は config_t のデフォルト (Preset::Regular / DataRate::Rate30Hz) のはず
    DataRate rate{};
    Preset preset{};
    EXPECT_TRUE(unit->readDataRate(rate));
    EXPECT_EQ(rate, DataRate::Rate30Hz);
    EXPECT_TRUE(unit->readPreset(preset));
    EXPECT_EQ(preset, Preset::Regular);
}

TEST_F(TestBMM150, ImuSpec)
{
    SCOPED_TRACE(ustr);

    // Magnetometer-only chip: no Accel/Gyro/Temp output
    const auto spec = unit->imuSpec();
    EXPECT_TRUE(m5::imu::any(spec & m5::imu::Capability::Mag));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Accel));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Gyro));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Temp));
    EXPECT_FALSE(m5::imu::any(spec & m5::imu::Capability::Baro));
}

TEST_F(TestBMM150, StopStartPeriodic)
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

TEST_F(TestBMM150, ConfigRoundTrip)
{
    SCOPED_TRACE(ustr);

    auto cfg           = unit->config();
    cfg.start_periodic = false;
    cfg.preset         = Preset::EnhancedRegular;
    cfg.rate           = DataRate::Rate10Hz;
    unit->config(cfg);

    const auto cfg2 = unit->config();
    EXPECT_FALSE(cfg2.start_periodic);
    EXPECT_EQ(cfg2.preset, cfg.preset);
    EXPECT_EQ(cfg2.rate, cfg.rate);
}

TEST_F(TestBMM150, SleepWakeup)
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

    // Waking only makes the registers writable again; measurement stays stopped
    EXPECT_TRUE(unit->wakeup());
    EXPECT_FALSE(unit->inPeriodic());

    // Suspend cleared the register map, so this is also what restores preset/data rate
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

TEST_F(TestBMM150, SoftReset)
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

TEST_F(TestBMM150, StartPeriodicWithArgs)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());

    // Non-default settings are written to the chip and synced to config()
    EXPECT_TRUE(unit->startPeriodicMeasurement(DataRate::Rate10Hz, Preset::HighAccuracy));
    EXPECT_TRUE(unit->inPeriodic());

    DataRate rate{};
    Preset preset{};
    EXPECT_TRUE(unit->readDataRate(rate));
    EXPECT_EQ(rate, DataRate::Rate10Hz);
    EXPECT_TRUE(unit->readPreset(preset));
    EXPECT_EQ(preset, Preset::HighAccuracy);

    const auto cfg = unit->config();
    EXPECT_EQ(cfg.rate, DataRate::Rate10Hz);
    EXPECT_EQ(cfg.preset, Preset::HighAccuracy);

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
    EXPECT_EQ(unit->config().rate, DataRate::Rate10Hz);
}
