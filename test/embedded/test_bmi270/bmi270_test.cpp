/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitBMI270

  Default (no TEST_BUILTIN): external wiring via I2CComponentTestBase (GROVE / standard I2C),
  e.g. a UnitDoF6 plugged into a host's Grove port.

  Attach only a UnitDoF6 or a Unit Mini IMU-Pro (U171) on that path -- NOT a DoF9/DoF10. This
  suite builds a bare UnitBMI270, so enable_aux_bmm150 keeps its default and the AUX probe runs.
  On DoF9/DoF10 the BMI270's AUX pins share the main bus, so enabling AUX makes the BMI270 a
  second I2C master and the bus collides; recovery needs the unit to be power-cycled. The DoF9 /
  DoF10 composites disable the probe themselves (see UnitDoF9's constructor), so test_dof9 /
  test_dof10 are the safe way to exercise those boards.

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

// GROVE path: the DoF boards' SW1 ships at 0x68 (UnitBMI270's class default is 0x69, matching the
// CoreS3 built-in) -- follow the DoF suites' convention: -D DOF_BMI270_ADDRESS=0x69 when flipped.
#ifndef DOF_BMI270_ADDRESS
#define DOF_BMI270_ADDRESS 0x68
#endif

class TestBMI270 : public I2CComponentTestBase<UnitBMI270> {
protected:
    virtual UnitBMI270* get_instance() override
    {
#if defined(TEST_BUILTIN)
        // The built-in BMI270 sits at a board-dependent address (0x68 or 0x69). Detect it on the
        // internal bus and construct with the result -- no build-time address, and a Capsule
        // (indistinguishable from a bare StampS3 via M5.getBoard()) is handled like any other board.
        uint8_t addr{0x68};
        m5::unit::bmi270::detectAddress(
            [](uint8_t a, uint8_t reg, uint8_t& out) { return M5.In_I2C.readRegister(a, reg, &out, 1, 400000); }, addr);
        auto ptr = new m5::unit::UnitBMI270(addr);
#else
        // GROVE: the bus is not connected yet at get_instance() time, so no probe -- use the
        // build-time address (see DOF_BMI270_ADDRESS above).
        auto ptr = new m5::unit::UnitBMI270(DOF_BMI270_ADDRESS);
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

    // Range decides accel_res/gyro_res, and samples already buffered were captured at the old
    // full-scale -- converting those with the new factor would silently misreport them. So the
    // range is a stop-first setting, refused while measuring
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_FALSE(unit->writeAccelRange(AccelRange::G4));
    EXPECT_FALSE(unit->writeGyroRange(GyroRange::DPS500));

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
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

    m5::imu::imu_convert_param_t cp{};
    unit->imuConvertParam(cp);
    m5::imu::imu_raw_data_t raw{};
    bool saw_nonzero_mag{false};
    float mag_norm{};
    int popped{};
    while (m5::imu::any(unit->readImuRaw(raw))) {
        ++popped;
        if (raw.mag[0] != 0 || raw.mag[1] != 0 || raw.mag[2] != 0) {
            saw_nonzero_mag = true;
            if (raw.mag[0] != bmm150::OVERFLOW_OUTPUT && raw.mag[1] != bmm150::OVERFLOW_OUTPUT &&
                raw.mag[2] != bmm150::OVERFLOW_OUTPUT) {
                const float mx = raw.mag[0] * cp.mag_res;
                const float my = raw.mag[1] * cp.mag_res;
                const float mz = raw.mag[2] * cp.mag_res;
                mag_norm       = std::sqrt(mx * mx + my * my + mz * mz);
            }
        }
    }
    EXPECT_GT(popped, 0);

    if (has_mag) {
        // AUX BMM150 detected: expect a real magnetometer reading in at least one of the
        // popped samples (BMM150 updates at 30Hz; 100ms should yield at least one)
        EXPECT_TRUE(saw_nonzero_mag);
        // The scale guard is structural rather than a magnitude band: out.mag holds the compensated
        // integer uT rescaled by 16 (adj_x/y/z) and mag_res scales it back, so the two readings have
        // to agree. Losing either factor moves them 16x apart however weak the local field is -- and
        // it can be weak: hard iron cancels much of the geomagnetic field at some orientations (an
        // AtomS3R measured under 15uT), so a fixed floor would only be a flaky test.
        const auto d = unit->latest().mag;
        if (d.comp_x() != bmm150::OVERFLOW_OUTPUT && d.comp_y() != bmm150::OVERFLOW_OUTPUT &&
            d.comp_z() != bmm150::OVERFLOW_OUTPUT) {
            const float cx        = d.comp_x();
            const float cy        = d.comp_y();
            const float cz        = d.comp_z();
            const float comp_norm = std::sqrt(cx * cx + cy * cy + cz * cz);
            M5_LOGI("AUX mag: comp=%.1fuT readImuRaw*mag_res=%.1fuT (raw=[%ld,%ld,%ld] mag_res=%.5f)", comp_norm,
                    mag_norm, (long)raw.mag[0], (long)raw.mag[1], (long)raw.mag[2], cp.mag_res);
            EXPECT_NEAR(mag_norm, comp_norm, comp_norm * 0.1f + 3.0f);
        }
        // Only a sanity range on top: non-zero, and clear of a base plate carrying a magnet
        // (over 1200uT measured on a Gray)
        EXPECT_GT(mag_norm, 1.0f);
        EXPECT_LT(mag_norm, 2000.0f);
    } else {
        // No AUX BMM150 wired: readImuRaw() never touches out.mag, so it stays at its
        // zero-initialized default across every popped sample
        EXPECT_FALSE(saw_nonzero_mag);
    }
}

TEST_F(TestBMI270, AuxMagPresetRejectsWhatTheFixedRateCannotComplete)
{
    SCOPED_TRACE(ustr);

    const bool has_mag = m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag);
    const auto original = unit->config();

    EXPECT_TRUE(unit->stopPeriodicMeasurement());

    // The bridge polls the AUX BMM150 at a fixed 30Hz (33.3ms), but HighAccuracy takes 49.3ms, so the
    // measurement would never complete and the data registers would stop updating. That has to fail
    // the start outright -- degrading to 6-axis would hide a configuration error behind the same
    // path a board with no magnetometer takes
    auto cfg           = original;
    cfg.aux_mag_preset = bmm150::Preset::HighAccuracy;
    unit->config(cfg);
    EXPECT_FALSE(unit->startPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    // One that fits is accepted, and the magnetometer still reports if the board has one
    cfg.aux_mag_preset = bmm150::Preset::EnhancedRegular;  // 16.7ms < 33.3ms
    unit->config(cfg);
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_EQ(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag), has_mag);

    // Restore what begin() had set up for the tests that follow
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    unit->config(original);
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_EQ(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag), has_mag);
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

TEST_F(TestBMI270, SleepWakeup)
{
    SCOPED_TRACE(ustr);

    // Remember whether this board has an AUX BMM150, so the post-wake capability can be
    // compared against the pre-sleep one
    const bool had_mag = m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag);

    // Unlike softReset(), suspending while running is allowed; it stops measurement itself
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_TRUE(unit->sleep());
    EXPECT_FALSE(unit->inPeriodic());

    // Nothing is sampled while suspended
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

    // Restarting re-probes the AUX BMM150, so the capability set must come back unchanged
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_EQ(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag), had_mag);

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

    // ...and the magnetometer must actually produce data again after its suspend
    if (had_mag) {
        m5::imu::imu_raw_data_t raw{};
        bool saw_nonzero_mag{false};
        while (m5::imu::any(unit->readImuRaw(raw))) {
            if (raw.mag[0] != 0 || raw.mag[1] != 0 || raw.mag[2] != 0) {
                saw_nonzero_mag = true;
            }
        }
        EXPECT_TRUE(saw_nonzero_mag);
    }
}

TEST_F(TestBMI270, SleepWhileStoppedAndRedundantWakeup)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->sleep());   // already stopped: still suspends
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

TEST_F(TestBMI270, SoftReset)
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

TEST_F(TestBMI270, StartPeriodicWithArgs)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());

    // Non-default settings are written to the chip and synced to config()
    EXPECT_TRUE(
        unit->startPeriodicMeasurement(AccelRange::G4, GyroRange::DPS500, AccelODR::Rate50Hz, GyroODR::Rate50Hz));
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
    EXPECT_EQ(cfg.accel_odr, AccelODR::Rate50Hz);
    EXPECT_EQ(cfg.gyro_odr, GyroODR::Rate50Hz);

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
    EXPECT_EQ(unit->config().gyro_odr, GyroODR::Rate50Hz);
}
