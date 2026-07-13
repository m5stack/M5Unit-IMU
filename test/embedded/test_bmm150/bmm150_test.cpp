/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitBMM150

  BMM150 単体の GROVE unit は存在しないため、この test env 群は GROVE 接続前提の
  wiring (I2CComponentTestBase の既定 begin()) のまま用意している。実機での
  BMM150 実行は built-in (chip がボードに直結された) 構成が必要になる:
    - 古い M5Stack Core/Fire (MPU6886 + BMM150 構成、m5::board_t::board_M5Stack) は
      M5Unified 内蔵 IMU として BMM150 を M5.In_I2C 経由で保持している
      (M5Unified/src/utility/IMU_Class.cpp:123, :130 の board_M5Stack 分岐/
      M5Unified/src/utility/imu/BMM150_Class.cpp)
    - CoreS3 / AtomS3R は BMI270 の AUX インタフェース経由 (Task 6, UnitBMI270 側で対応)
  内蔵バスでの実行方法の前例は、M5Unit-ENV の
  examples/UnitUnified/Paper/main/Paper.cpp (M5Paper 内蔵 SHT30 を
  `M5.getBoard() == m5::board_t::board_M5Paper` で判定し `Units.add(sht30, M5.In_I2C)`
  で束ねる、ただし example であり test ではない) と、実際の embedded *test* での
  同パターンの適用例である M5Unit-CRYPTO/test/embedded/test_id/id_test.cpp
  (`#if defined(USING_M5CORE2_AWS_BUILTIN)` で `Units.add(*unit, M5.In_I2C)` に
  差し替える TestID::begin() オーバーライド) を参照。BMM150 を built-in 構成の
  実機で回す場合は、同様に `USING_M5STACK_BUILTIN_BMM150` 相当のガードと
  `M5.In_I2C` への差し替えを追加する想定 (本 Task では未実装、要フォローアップ)。
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

    const auto d = unit->latest();
    // 補償後の磁場 (16 LSB/uT) を uT に戻し、地磁気の常識的な範囲 (10-100uT) に収まることを確認
    const float mx   = d.comp_x() / 16.0f;
    const float my   = d.comp_y() / 16.0f;
    const float mz   = d.comp_z() / 16.0f;
    const float norm = std::sqrt(mx * mx + my * my + mz * mz);
    EXPECT_GT(norm, 10.0f);
    EXPECT_LT(norm, 100.0f);
}

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
