/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for AhrsController (real hardware)

  UnitIMUMini (UnitMPU6886, GROVE) is used as the IMU source, matching
  test_mpu6886/mpu6886_test.cpp's fixture (default I2CComponentTestBase<UnitMPU6886>
  wiring). The unit is assumed static and level on the bench, as in
  TestMPU6886::PeriodicMeasurement.
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_mpu6886.hpp>
#include <imu/ahrs_controller.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace m5::unit::googletest;
using namespace m5::unit;

class TestAHRS : public I2CComponentTestBase<UnitMPU6886> {
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

    m5::imu::AhrsController ahrs{};
};

namespace {

constexpr float STATIC_TOLERANCE_DEG{5.0f};
constexpr float FILTER_TOLERANCE_DEG{8.0f};
constexpr uint32_t STATIC_DURATION_MS{3000};
constexpr uint32_t FILTER_DURATION_MS{2000};
constexpr uint32_t UPDATE_INTERVAL_MS{5};

const char* filter_name(const m5::imu::FilterType type)
{
    switch (type) {
        case m5::imu::FilterType::Complementary:
            return "Complementary";
        case m5::imu::FilterType::Madgwick:
            return "Madgwick";
        case m5::imu::FilterType::Mahony:
            return "Mahony";
        case m5::imu::FilterType::EKF:
            return "EKF";
        case m5::imu::FilterType::Fusion:
            return "Fusion";
        case m5::imu::FilterType::VQF:
            return "VQF";
    }
    return "?";
}

// Drains the unit's periodic buffer and feeds AhrsController for duration_ms (real clock)
void run_updates(m5::unit::UnitUnified& Units, m5::imu::AhrsController& ahrs, const uint32_t duration_ms)
{
    const auto start_at = m5::utility::millis();
    while (m5::utility::millis() - start_at < duration_ms) {
        Units.update();
        ahrs.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_INTERVAL_MS));
    }
}

}  // namespace

TEST_F(TestAHRS, AhrsBeginWithRealUnit)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(ahrs.add(*unit));
    EXPECT_EQ(ahrs.filter(), nullptr);
    EXPECT_TRUE(ahrs.begin());
    // begin() selects FilterType::Madgwick if no filter has been chosen yet (see
    // AhrsController::begin() docs). RTTI is disabled in this build (-fno-rtti), so this is
    // verified indirectly: non-null here plus AhrsController's own contract.
    ASSERT_NE(ahrs.filter(), nullptr);
}

TEST_F(TestAHRS, StaticConvergence)
{
    SCOPED_TRACE(ustr);

    ASSERT_TRUE(ahrs.add(*unit));
    ASSERT_TRUE(ahrs.begin());

    run_updates(Units, ahrs, STATIC_DURATION_MS);

    // 静置想定: roll/pitch は水平姿勢 (0度) 付近に収束するはず
    float roll{}, pitch{}, yaw{};
    ahrs.euler(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, STATIC_TOLERANCE_DEG);
    EXPECT_NEAR(pitch, 0.0f, STATIC_TOLERANCE_DEG);

    float w{}, x{}, y{}, z{};
    ahrs.quaternion(w, x, y, z);
    const float norm = std::sqrt(w * w + x * x + y * y + z * z);
    EXPECT_NEAR(norm, 1.0f, 0.05f);
}

TEST_F(TestAHRS, AllFiltersConverge)
{
    SCOPED_TRACE(ustr);

    ASSERT_TRUE(ahrs.add(*unit));
    ASSERT_TRUE(ahrs.begin());

    constexpr m5::imu::FilterType filter_table[] = {
        m5::imu::FilterType::Complementary, m5::imu::FilterType::Madgwick, m5::imu::FilterType::Mahony,
        m5::imu::FilterType::EKF,           m5::imu::FilterType::Fusion,   m5::imu::FilterType::VQF,
    };

    for (auto&& type : filter_table) {
        auto s = m5::utility::formatString("Filter:%s", filter_name(type));
        SCOPED_TRACE(s);

        ASSERT_TRUE(ahrs.selectFilter(type));
        run_updates(Units, ahrs, FILTER_DURATION_MS);

        // 静置想定。VQF/Fusion 等の初期収束の遅さを考慮し、緩めの tolerance で判定
        float roll{}, pitch{}, yaw{};
        ahrs.euler(roll, pitch, yaw);
        EXPECT_NEAR(roll, 0.0f, FILTER_TOLERANCE_DEG);
        EXPECT_NEAR(pitch, 0.0f, FILTER_TOLERANCE_DEG);
    }
}
