/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitBMI270

  Default (no TEST_BUILTIN): external wiring via I2CComponentTestBase (GROVE / standard I2C).

  Attach ONLY a Unit Mini IMU-Pro (U171) on that path -- no DoF board of any kind. This suite builds
  a UnitBMI270AuxBMM150, so the AUX probe runs, and the U171 is the one M5 unit whose BMM150 really
  sits on the AUX interface. On the DoF boards the BMI270's AUX pins share the main I2C bus, so
  enabling AUX turns the BMI270 into a second bus master and the bus collides.

  That includes the DoF6, which is the same board family with the magnetometer left unpopulated. It
  usually survives a run here because the probe finds nothing and switches the interface straight
  back off (see UnitBMI270AuxBMM150::setup_aux_bmm150), so the BMI270 is only a second master for a
  few milliseconds -- but the window is real, and losing that race has been observed: the unit then
  answered every read with 0x20 instead of CHIP_ID 0x24 until it was physically unplugged, re-flashing
  the host being no help since the unit keeps its power. Passing is luck, not safety.

  Use test_dof6 for a DoF6 and test_dof9 / test_dof10 for those boards -- all three instantiate the
  plain UnitBMI270, which never touches the AUX interface.

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
#include <unit/unit_bmi270_aux_bmm150.hpp>
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

class TestBMI270 : public I2CComponentTestBase<UnitBMI270AuxBMM150> {
protected:
    virtual UnitBMI270AuxBMM150* get_instance() override
    {
#if defined(TEST_BUILTIN)
        // The built-in BMI270 sits at a board-dependent address (0x68 or 0x69). Detect it on the
        // internal bus and construct with the result -- no build-time address, and a Capsule
        // (indistinguishable from a bare StampS3 via M5.getBoard()) is handled like any other board.
        uint8_t addr{0x68};
        m5::unit::bmi270::detectAddress(
            [](uint8_t a, uint8_t reg, uint8_t& out) { return M5.In_I2C.readRegister(a, reg, &out, 1, 400000); }, addr);
        auto ptr = new m5::unit::UnitBMI270AuxBMM150(addr);
#else
        // GROVE: the bus is not connected yet at get_instance() time, so no probe -- use the
        // build-time address (see DOF_BMI270_ADDRESS above).
        auto ptr = new m5::unit::UnitBMI270AuxBMM150(DOF_BMI270_ADDRESS);
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

TEST_F(TestBMI270, WriteOutputDataRate)
{
    SCOPED_TRACE(ustr);

    // The rate sets the measurement interval, so like the range it is a stop-first setting
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_FALSE(unit->writeAccelODR(AccelODR::Rate50Hz));
    EXPECT_FALSE(unit->writeGyroODR(GyroODR::Rate50Hz));

    // ...and the device keeps what begin() configured
    AccelODR ao{};
    GyroODR go{};
    EXPECT_TRUE(unit->readAccelODR(ao));
    EXPECT_EQ(ao, AccelODR::Rate200Hz);
    EXPECT_TRUE(unit->readGyroODR(go));
    EXPECT_EQ(go, GyroODR::Rate200Hz);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->writeAccelODR(AccelODR::Rate50Hz));
    EXPECT_TRUE(unit->writeGyroODR(GyroODR::Rate50Hz));
    EXPECT_TRUE(unit->readAccelODR(ao));
    EXPECT_EQ(ao, AccelODR::Rate50Hz);
    EXPECT_TRUE(unit->readGyroODR(go));
    EXPECT_EQ(go, GyroODR::Rate50Hz);

    // The rate shares its register with the bandwidth and filter performance, so check the write
    // recomposed rather than clobbered them: the conversion factor still has to match the range the
    // device reports
    AccelRange ar{};
    EXPECT_TRUE(unit->readAccelRange(ar));
    m5::imu::imu_convert_param_t p{};
    unit->imuConvertParam(p);
    EXPECT_FLOAT_EQ(p.accel_res, accel_res(ar));

    // Measurement still flows at the new rate
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int updated{};
    for (int i = 0; i < 20; ++i) {
        unit->update();
        if (unit->updated()) {
            ++updated;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_GT(updated, 0);
}

TEST_F(TestBMI270, BandwidthParameter)
{
    SCOPED_TRACE(ustr);

    // Every REG_ACC_CONF/REG_GYR_CONF field is an argument of the full startPeriodicMeasurement(),
    // so the bandwidth is exercised through that rather than through config_t (which only begin()
    // reads)
    const auto cfg = unit->config();
    EXPECT_TRUE(unit->stopPeriodicMeasurement());

    // The accelerometer's averaging-only bandwidths are reserved while the filter is in Performance
    // mode, so that pair must be refused rather than written to the register as a reserved code
    EXPECT_FALSE(unit->startPeriodicMeasurement(cfg.accel_range, cfg.gyro_range, cfg.accel_odr, cfg.gyro_odr,
                                                FilterPerf::Performance, AccelBwp::Avg16, FilterPerf::Performance,
                                                GyroBwp::Norm));
    EXPECT_FALSE(unit->inPeriodic());

    // The same bandwidth is fine once the filter averages instead of filtering
    EXPECT_TRUE(unit->startPeriodicMeasurement(cfg.accel_range, cfg.gyro_range, cfg.accel_odr, cfg.gyro_odr,
                                               FilterPerf::PowerOptimized, AccelBwp::Avg16, FilterPerf::Performance,
                                               GyroBwp::Norm));
    EXPECT_TRUE(unit->inPeriodic());

    // A non-default filter bandwidth on both sensors is accepted and measurement keeps flowing
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->startPeriodicMeasurement(cfg.accel_range, cfg.gyro_range, cfg.accel_odr, cfg.gyro_odr,
                                               FilterPerf::Performance, AccelBwp::Osr2Avg2, FilterPerf::Performance,
                                               GyroBwp::Osr4));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int updated{};
    for (int i = 0; i < 20; ++i) {
        unit->update();
        if (unit->updated()) {
            ++updated;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_GT(updated, 0);

    // The no-argument overload resumes from the live state, i.e. the bandwidth set above
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());
}

TEST_F(TestBMI270, WriteAuxMagRepetition)
{
    SCOPED_TRACE(ustr);

    const bool has_mag = m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag);
    uint8_t org_xy{}, org_z{}, rep_xy{}, rep_z{};
    EXPECT_TRUE(unit->readAuxMagRepetition(org_xy, org_z));
    // begin() seeds the live state from config_t
    EXPECT_EQ(org_xy, bmm150::preset_rep_xy(unit->config().aux_mag_preset));
    EXPECT_EQ(org_z, bmm150::preset_rep_z(unit->config().aux_mag_preset));

    // Changing the repetition changes how long a measurement takes, so it is refused while running
    EXPECT_FALSE(unit->writeAuxMagPreset(bmm150::Preset::EnhancedRegular));
    EXPECT_TRUE(unit->readAuxMagRepetition(rep_xy, rep_z));
    EXPECT_EQ(rep_xy, org_xy);
    EXPECT_EQ(rep_z, org_z);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());

    // The bridge polls the AUX BMM150 at a fixed 25Hz (40ms), but HighAccuracy takes 49.3ms, so the
    // measurement would never complete and the data registers would stop updating. Unlike a
    // stand-alone UnitBMM150 there is no slower rate to fall back on, so it is rejected outright
    EXPECT_FALSE(unit->writeAuxMagPreset(bmm150::Preset::HighAccuracy));
    // A rejected write leaves the previous value in place
    EXPECT_TRUE(unit->readAuxMagRepetition(rep_xy, rep_z));
    EXPECT_EQ(rep_xy, org_xy);
    EXPECT_EQ(rep_z, org_z);

    // An arbitrary pair that fits is accepted, not just the presets
    EXPECT_TRUE(unit->writeAuxMagRepetition(3, 5));  // 145*7 + 500*6 + 980 = 4995us < 40000us
    EXPECT_TRUE(unit->readAuxMagRepetition(rep_xy, rep_z));
    EXPECT_EQ(rep_xy, 3);
    EXPECT_EQ(rep_z, 5);

    // One that fits is accepted, and the magnetometer still reports if the board has one
    EXPECT_TRUE(unit->writeAuxMagPreset(bmm150::Preset::EnhancedRegular));  // 16.7ms < 40ms
    EXPECT_TRUE(unit->readAuxMagRepetition(rep_xy, rep_z));
    EXPECT_EQ(rep_xy, bmm150::preset_rep_xy(bmm150::Preset::EnhancedRegular));
    EXPECT_EQ(rep_z, bmm150::preset_rep_z(bmm150::Preset::EnhancedRegular));

    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_EQ(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag), has_mag);

    // ...and the data actually keeps moving. This is the check that decides whether the AUX bridge
    // re-triggers the BMM150 every poll (in which case the measurement has to finish within
    // AUX_CONF.aux_offset) or just reads a free-running sensor. EnhancedRegular takes 16.7ms, which
    // would not fit the 10ms reset offset -- so a frozen reading here means the offset matters and
    // the driver's AUX_CONF value is what is keeping it working
    if (has_mag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        int32_t first[3]{};
        bool moved{false}, got_first{false};
        for (int i = 0; i < 60; ++i) {
            unit->update();
            m5::imu::imu_raw_data_t raw{};
            while (m5::imu::any(unit->readImuRaw(raw))) {
                if (raw.mag[0] == 0 && raw.mag[1] == 0 && raw.mag[2] == 0) {
                    continue;
                }
                if (!got_first) {
                    first[0]  = raw.mag[0];
                    first[1]  = raw.mag[1];
                    first[2]  = raw.mag[2];
                    got_first = true;
                } else if (raw.mag[0] != first[0] || raw.mag[1] != first[1] || raw.mag[2] != first[2]) {
                    moved = true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        EXPECT_TRUE(got_first);
        // Sensor noise alone moves the low bits, so a completely constant reading over ~600ms
        // (15 poll periods at 25Hz) means the bridge is handing back a stale sample
        EXPECT_TRUE(moved) << "AUX mag frozen at EnhancedRegular -- measurement is being cut short";
    }

    // Restore what begin() had set up for the tests that follow
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->writeAuxMagRepetition(org_xy, org_z));
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_EQ(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag), has_mag);
}

TEST_F(TestBMI270, AuxDetectionIsCachedAcrossRestart)
{
    SCOPED_TRACE(ustr);

    // Whether this board carries an AUX BMM150 is answered once and remembered, so a plain
    // stop/start must not change the verdict -- and on a board that has one, the second start has
    // to reach data mode without redoing the detection that filled in the trim
    const bool has_mag = m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag);

    // Timing the two paths is what actually shows the caching happens; the verdict alone would stay
    // the same even if every start re-detected. begin() has already answered the question by the
    // time this runs, so a plain stop/start is always the cached path -- sleep() is what puts the
    // detection back to Unknown, because suspend resets the magnetometer.
    //
    // Measured over ROUNDS restarts each: the difference is a BMM150 soft reset plus its 3ms
    // settling delay plus either three trim bursts (present) or two chip ID reads that time out on
    // aux_busy (absent), all of it well under one millis() tick on its own.
    // Only the startPeriodicMeasurement() calls are timed, never the sleep()/wakeup() around them:
    // each round pays that cost once and it lands outside both accumulators, so the whole remaining
    // gap is the detection the cached path skips. Comparing the two paths against each other rather
    // than either against a fixed budget keeps this independent of the board's bus speed
    constexpr int ROUNDS{5};
    uint32_t detect_ms{}, cached_ms{};

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    for (int i = 0; i < ROUNDS; ++i) {
        // Back to Unknown: the next start has to detect again
        EXPECT_TRUE(unit->sleep()) << "round " << i;
        EXPECT_TRUE(unit->wakeup()) << "round " << i;

        auto at = m5::utility::millis();
        EXPECT_TRUE(unit->startPeriodicMeasurement()) << "round " << i;
        detect_ms += m5::utility::millis() - at;
        EXPECT_EQ(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag), has_mag) << "round " << i;
        EXPECT_TRUE(unit->stopPeriodicMeasurement()) << "round " << i;

        // Same call, same state, except the verdict is now cached
        at = m5::utility::millis();
        EXPECT_TRUE(unit->startPeriodicMeasurement()) << "round " << i;
        cached_ms += m5::utility::millis() - at;
        EXPECT_EQ(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag), has_mag) << "round " << i;
        EXPECT_TRUE(unit->stopPeriodicMeasurement()) << "round " << i;
    }

    M5_LOGI("AUX start x%d: re-detect=%lums cached=%lums (mag:%d)", ROUNDS, (unsigned long)detect_ms,
            (unsigned long)cached_ms, has_mag);
    EXPECT_GT(detect_ms, cached_ms) << "AUX detection is not being cached";

    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_EQ(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag), has_mag);

    if (!has_mag) {
        return;
    }

    // The trim survived, so the compensated output is still sane rather than the garbage an
    // uninitialised trim would produce
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    m5::imu::imu_convert_param_t param{};
    unit->imuConvertParam(param);
    bool checked{false};
    for (int i = 0; i < 40 && !checked; ++i) {
        unit->update();
        m5::imu::imu_raw_data_t raw{};
        while (m5::imu::any(unit->readImuRaw(raw))) {
            if (raw.mag[0] == 0 && raw.mag[1] == 0 && raw.mag[2] == 0) {
                continue;
            }
            const float mx   = raw.mag[0] * param.mag_res;
            const float my   = raw.mag[1] * param.mag_res;
            const float mz   = raw.mag[2] * param.mag_res;
            const float norm = std::sqrt(mx * mx + my * my + mz * mz);
            M5_LOGI("AUX mag after restart: %.1fuT (raw=[%ld,%ld,%ld])", norm, (long)raw.mag[0], (long)raw.mag[1],
                    (long)raw.mag[2]);
            // Earth's field is tens of uT; hard iron on a dev board shifts it but cannot take it
            // out of this range, whereas a lost trim lands orders of magnitude away
            EXPECT_GT(norm, 1.0f);
            EXPECT_LT(norm, 2000.0f);
            checked = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(checked);
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
    EXPECT_TRUE(unit->sleep());  // already stopped: still suspends
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
    EXPECT_TRUE(unit->startPeriodicMeasurement(AccelRange::G4, GyroRange::DPS500, AccelODR::Rate50Hz, GyroODR::Rate50Hz,
                                               FilterPerf::Performance, AccelBwp::NormAvg4, FilterPerf::Performance,
                                               GyroBwp::Norm));
    EXPECT_TRUE(unit->inPeriodic());

    AccelRange ar{};
    GyroRange gr{};
    EXPECT_TRUE(unit->readAccelRange(ar));
    EXPECT_EQ(ar, AccelRange::G4);
    EXPECT_TRUE(unit->readGyroRange(gr));
    EXPECT_EQ(gr, GyroRange::DPS500);

    // config() reports what was configured for begin(), not what the device is doing now, so the
    // arguments passed above are verified by reading the registers back
    AccelODR ao{};
    GyroODR go{};
    EXPECT_TRUE(unit->readAccelODR(ao));
    EXPECT_EQ(ao, AccelODR::Rate50Hz);
    EXPECT_TRUE(unit->readGyroODR(go));
    EXPECT_EQ(go, GyroODR::Rate50Hz);

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

    // The no-argument overload resumes with the settings in effect, not the begin-time ones
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_TRUE(unit->readGyroODR(go));
    EXPECT_EQ(go, GyroODR::Rate50Hz);
}

TEST_F(TestBMI270, AxisRemapDefaultIsIdentity)
{
    SCOPED_TRACE(ustr);
    // After begin() the config blob leaves the axis remap at chip default (identity)
    AxisRemap r;
    EXPECT_TRUE(unit->readAxisRemap(r));
    EXPECT_FALSE(r.invert_x);
    EXPECT_FALSE(r.invert_y);
    EXPECT_FALSE(r.invert_z);
    EXPECT_FALSE(r.swap_xy);
    EXPECT_EQ(0, r.rotation_deg);
}

TEST_F(TestBMI270, AxisRemapRoundTrip)
{
    SCOPED_TRACE(ustr);
    // AxisRemap has default member initializers, so C++11 does not permit aggregate init.
    // Build values field-by-field via a helper
    auto mk = [](bool ix, bool iy, bool iz, bool sw, uint16_t rot) {
        AxisRemap r{};
        r.invert_x     = ix;
        r.invert_y     = iy;
        r.invert_z     = iz;
        r.swap_xy      = sw;
        r.rotation_deg = rot;
        return r;
    };
    // Canonical form (rotation_deg=0, invert_x/invert_y/invert_z/swap_xy only) roundtrips exactly
    struct Case {
        AxisRemap in;
        const char* label;
    };
    const Case cases[] = {
        {mk(false, false, false, false, 0), "identity"},
        {mk(true, false, false, false, 0), "invert_x"},
        {mk(false, true, false, false, 0), "invert_y"},
        {mk(false, false, true, false, 0), "invert_z"},
        {mk(false, false, false, true, 0), "swap"},
        {mk(true, true, false, false, 0), "invert_xy"},
        {mk(true, true, true, false, 0), "invert_xyz"},
        {mk(true, true, false, true, 0), "invert_xy+swap"},
        {mk(true, false, true, false, 0), "invert_xz-atomMatrix"},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(c.label);
        ASSERT_TRUE(unit->writeAxisRemap(c.in));
        AxisRemap r{};
        ASSERT_TRUE(unit->readAxisRemap(r));
        EXPECT_EQ(c.in.invert_x, r.invert_x);
        EXPECT_EQ(c.in.invert_y, r.invert_y);
        EXPECT_EQ(c.in.invert_z, r.invert_z);
        EXPECT_EQ(c.in.swap_xy, r.swap_xy);
        EXPECT_EQ(0, r.rotation_deg);
    }
    // rotation_deg=90/180/270 land in the same chip state as some invert+swap combo. The read
    // returns that canonical form (rotation_deg=0). Verify chip-state stability: writing the read
    // value back produces the same read
    const AxisRemap rot_cases[] = {
        mk(false, false, false, false, 90), mk(false, false, false, false, 180), mk(false, false, false, false, 270),
        mk(false, false, true, false, 90),  // rotation + independent invert_z
    };
    for (const auto& in : rot_cases) {
        SCOPED_TRACE(in.rotation_deg);
        ASSERT_TRUE(unit->writeAxisRemap(in));
        AxisRemap first{};
        ASSERT_TRUE(unit->readAxisRemap(first));
        EXPECT_EQ(0, first.rotation_deg);        // canonical decode drops rotation
        EXPECT_EQ(in.invert_z, first.invert_z);  // invert_z is independent, preserved through decode
        ASSERT_TRUE(unit->writeAxisRemap(first));
        AxisRemap second{};
        ASSERT_TRUE(unit->readAxisRemap(second));
        EXPECT_EQ(first.invert_x, second.invert_x);
        EXPECT_EQ(first.invert_y, second.invert_y);
        EXPECT_EQ(first.invert_z, second.invert_z);
        EXPECT_EQ(first.swap_xy, second.swap_xy);
        EXPECT_EQ(0, second.rotation_deg);
    }
    EXPECT_TRUE(unit->resetAxisRemap());
}

TEST_F(TestBMI270, WearableOrientationPresetsAndReset)
{
    SCOPED_TRACE(ustr);
    // Default preset writes identity
    ASSERT_TRUE(unit->writeWearableOrientation(WearableOrientation::Default));
    AxisRemap rdef{};
    ASSERT_TRUE(unit->readAxisRemap(rdef));
    EXPECT_FALSE(rdef.invert_x);
    EXPECT_FALSE(rdef.invert_y);
    EXPECT_FALSE(rdef.invert_z);
    EXPECT_FALSE(rdef.swap_xy);
    EXPECT_EQ(0, rdef.rotation_deg);

    // Each rotation preset produces a distinct non-identity chip state (X/Y only; Z stays)
    ASSERT_TRUE(unit->writeWearableOrientation(WearableOrientation::Rotate90CW));
    AxisRemap r90{};
    ASSERT_TRUE(unit->readAxisRemap(r90));
    const bool r90_is_identity = !r90.invert_x && !r90.invert_y && !r90.invert_z && !r90.swap_xy;
    EXPECT_FALSE(r90_is_identity);
    EXPECT_FALSE(r90.invert_z);  // rotation acts in the X/Y plane only

    ASSERT_TRUE(unit->writeWearableOrientation(WearableOrientation::Rotate180));
    AxisRemap r180{};
    ASSERT_TRUE(unit->readAxisRemap(r180));
    const bool r180_matches_r90 =
        (r180.invert_x == r90.invert_x) && (r180.invert_y == r90.invert_y) && (r180.swap_xy == r90.swap_xy);
    EXPECT_FALSE(r180_matches_r90);
    EXPECT_FALSE(r180.invert_z);

    ASSERT_TRUE(unit->writeWearableOrientation(WearableOrientation::Rotate270CW));
    AxisRemap r270{};
    ASSERT_TRUE(unit->readAxisRemap(r270));
    const bool r270_matches_r180 =
        (r270.invert_x == r180.invert_x) && (r270.invert_y == r180.invert_y) && (r270.swap_xy == r180.swap_xy);
    EXPECT_FALSE(r270_matches_r180);
    EXPECT_FALSE(r270.invert_z);

    // resetAxisRemap() restores identity
    ASSERT_TRUE(unit->resetAxisRemap());
    AxisRemap rreset{};
    ASSERT_TRUE(unit->readAxisRemap(rreset));
    EXPECT_FALSE(rreset.invert_x);
    EXPECT_FALSE(rreset.invert_y);
    EXPECT_FALSE(rreset.invert_z);
    EXPECT_FALSE(rreset.swap_xy);
    EXPECT_EQ(0, rreset.rotation_deg);
}
