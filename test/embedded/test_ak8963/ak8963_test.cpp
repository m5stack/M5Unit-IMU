/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitAK8963

  With -DTEST_BUILTIN the AK8963 embedded in an original (MPU9250-based) classic M5Stack
  Core/Fire is exercised over M5.In_I2C. The MPU9250 aux-I2C bypass is opened first so the
  AK8963 answers at 0x0C. Later Fire revisions (SH200Q/MPU6886 + BMM150) have no AK8963, so
  this env only passes on the original hardware.
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_ak8963.hpp>
#include <imu/mag_detect.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;

class TestAK8963 : public I2CComponentTestBase<UnitAK8963> {
protected:
    virtual UnitAK8963* get_instance() override
    {
        auto ptr = new m5::unit::UnitAK8963();
        if (ptr) {
            auto ccfg = ptr->component_config();
            ptr->component_config(ccfg);
        }
        return ptr;
    }

#if defined(TEST_BUILTIN)
    bool begin() override
    {
        // Open the MPU9250 bypass so the internal AK8963 is reachable at 0x0C.
        m5::unit::imu::enableAuxBypass(
            [](uint8_t a, uint8_t r, uint8_t& o) { return M5.In_I2C.readRegister(a, r, &o, 1, 400000); },
            [](uint8_t a, uint8_t r, uint8_t v) { return M5.In_I2C.writeRegister8(a, r, v, 400000); });
        return Units.add(*unit, M5.In_I2C) && Units.begin();
    }
#endif
};

#if defined(TEST_BUILTIN)
// Diagnostic: dump every device on the internal bus and identify the accel chip. Does not depend on
// the unit fixture (whose begin() fails when the mag is absent), so it always runs. Use it to learn
// what this specific board actually carries (MPU9250 0x68/WHO_AM_I 0x71 + AK8963 0x0C, MPU6886 0x19
// + BMM150 0x10, or none).
TEST(AK8963BusScan, DumpInternalBus)
{
    printf("[scan] In_I2C ACK addresses:");
    for (uint8_t a = 0x08; a <= 0x77; ++a) {
        uint8_t b{};
        if (M5.In_I2C.readRegister(a, 0x00, &b, 1, 100000)) {
            printf(" 0x%02X", a);
        }
    }
    printf("\n");
    uint8_t who{};
    if (M5.In_I2C.readRegister(0x68, 0x75, &who, 1, 100000)) {
        printf("[scan] 0x68 WHO_AM_I(0x75)=0x%02X (MPU6886=0x19 MPU6050=0x68 MPU9250=0x71)\n", who);
    } else {
        printf("[scan] 0x68 no ACK\n");
    }
    uint8_t wia{};
    if (M5.In_I2C.readRegister(0x0C, 0x00, &wia, 1, 100000)) {
        printf("[scan] 0x0C WIA(0x00)=0x%02X (AK8963=0x48)\n", wia);
    } else {
        printf("[scan] 0x0C no ACK\n");
    }
    SUCCEED();
}

TEST_F(TestAK8963, DetectReportsAk8963)
{
    SCOPED_TRACE(ustr);
    const auto kind = m5::unit::imu::detectBuiltinMag(
        [](uint8_t a, uint8_t r, uint8_t& o) { return M5.In_I2C.readRegister(a, r, &o, 1, 400000); });
    EXPECT_EQ(kind, m5::unit::imu::MagKind::AK8963);
}
#endif

TEST_F(TestAK8963, BeginStartsPeriodic)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->inPeriodic());
}

TEST_F(TestAK8963, ImuSpecIsMag)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Mag));
    EXPECT_FALSE(m5::imu::any(unit->imuSpec() & m5::imu::Capability::Accel));
}

TEST_F(TestAK8963, PeriodicMeasurement)
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
        const float mx   = raw.mag[0] * param.mag_res;
        const float my   = raw.mag[1] * param.mag_res;
        const float mz   = raw.mag[2] * param.mag_res;
        const float norm = std::sqrt(mx * mx + my * my + mz * mz);
        EXPECT_TRUE(std::isfinite(norm));
        EXPECT_GT(norm, 10.0f);
        EXPECT_LT(norm, 200.0f);  // built-in mag: allow mild hard-iron (cf. BMM150 built-in test)
        ++checked;
    }
    EXPECT_GT(checked, 0);
}

TEST_F(TestAK8963, SoftReset)
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
