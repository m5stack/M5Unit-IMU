/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/ahrs_controller.hpp>
#include <imu/imu_data_source.hpp>
#include <imu/imu_types.hpp>
#include <vector>

using namespace m5::imu;

namespace {

// Pop-semantics mock IMUDataSource (see test/native/test_imu_data_source). Uses an identity
// imu_convert_param_t by default so pushed raw values equal the resulting physical values,
// keeping the arithmetic in each test readable.
class MockSource : public IMUDataSource {
public:
    explicit MockSource(const Capability cap) : _cap(cap)
    {
        _param.accel_res   = 1.0f;
        _param.gyro_res    = 1.0f;
        _param.mag_res     = 1.0f;
        _param.temp_res    = 1.0f;
        _param.temp_offset = 0.0f;
    }

    Capability imuSpec() const override
    {
        return _cap;
    }
    Capability readImuRaw(imu_raw_data_t& out) override
    {
        if (_pos >= _samples.size()) {
            return Capability::None;
        }
        out = _samples[_pos++];
        ++_pop_count;
        return _cap;
    }
    void imuConvertParam(imu_convert_param_t& out) const override
    {
        out = _param;
    }

    void pushAccelGyro(const int16_t ax, const int16_t ay, const int16_t az, const int16_t gx, const int16_t gy,
                       const int16_t gz)
    {
        imu_raw_data_t d{};
        d.accel[0] = ax;
        d.accel[1] = ay;
        d.accel[2] = az;
        d.gyro[0]  = gx;
        d.gyro[1]  = gy;
        d.gyro[2]  = gz;
        _samples.push_back(d);
    }
    void pushMag(const int32_t mx, const int32_t my, const int32_t mz)
    {
        imu_raw_data_t d{};
        d.mag[0] = mx;
        d.mag[1] = my;
        d.mag[2] = mz;
        _samples.push_back(d);
    }

    inline size_t remaining() const
    {
        return _samples.size() - _pos;
    }
    inline size_t popCount() const
    {
        return _pop_count;
    }

private:
    Capability _cap;
    imu_convert_param_t _param{};
    std::vector<imu_raw_data_t> _samples{};
    size_t _pos{};
    size_t _pop_count{};
};

// Records every update()/reset() call so tests can assert on the AhrsController <-> filter
// boundary (dt division, mag nullptr-vs-pointer, injection) without depending on any real
// filter's numeric convergence.
class MockFilter : public AhrsFilterBase {
public:
    void reset() override
    {
        ++_reset_count;
    }
    void update(const float dt, const float accel[3], const float gyro[3], const float* mag) override
    {
        ++_update_count;
        _dts.push_back(dt);
        _last_mag_is_null = (mag == nullptr);
        for (int i = 0; i < 3; ++i) {
            _last_accel[i] = accel[i];
            _last_gyro[i]  = gyro[i];
            _last_mag[i]   = mag ? mag[i] : 0.0f;
        }
    }
    void quaternion(float& w, float& x, float& y, float& z) const override
    {
        w = _qw;
        x = _qx;
        y = _qy;
        z = _qz;
    }

    inline int resetCount() const
    {
        return _reset_count;
    }
    inline int updateCount() const
    {
        return _update_count;
    }
    inline const std::vector<float>& dts() const
    {
        return _dts;
    }
    inline bool lastMagIsNull() const
    {
        return _last_mag_is_null;
    }
    inline const float* lastAccel() const
    {
        return _last_accel;
    }
    inline const float* lastGyro() const
    {
        return _last_gyro;
    }
    inline const float* lastMag() const
    {
        return _last_mag;
    }

private:
    int _reset_count{};
    int _update_count{};
    std::vector<float> _dts{};
    bool _last_mag_is_null{true};
    float _last_accel[3]{};
    float _last_gyro[3]{};
    float _last_mag[3]{};
    float _qw{1.0f};
    float _qx{};
    float _qy{};
    float _qz{};
};

}  // namespace

TEST(AhrsController, MergeNineAxisUpdatesAllCapabilities)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    MockSource mag(Capability::Mag);
    ag.pushAccelGyro(0, 0, 1, 0, 0, 90);
    mag.pushMag(20, 0, -40);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.add(mag));
    EXPECT_TRUE(ahrs.begin());

    const Capability updated = ahrs.update(0.01f);
    EXPECT_TRUE(any(updated & Capability::Accel));
    EXPECT_TRUE(any(updated & Capability::Gyro));
    EXPECT_TRUE(any(updated & Capability::Mag));

    const imu_physical_data_t& latest = ahrs.latest();
    EXPECT_FLOAT_EQ(latest.accel[2], 1.0f);
    EXPECT_FLOAT_EQ(latest.mag[0], 20.0f);
    EXPECT_FLOAT_EQ(latest.mag[2], -40.0f);
}

TEST(AhrsController, DuplicateCapabilityKeepsFirstSource)
{
    MockSource first(Capability::Accel | Capability::Gyro);
    MockSource second(Capability::Accel);
    first.pushAccelGyro(0, 0, 1, 0, 0, 0);
    second.pushAccelGyro(9, 9, 9, 0, 0, 0);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(first));
    EXPECT_TRUE(ahrs.add(second));  // add() itself still succeeds
    EXPECT_TRUE(ahrs.begin());

    ahrs.update(0.01f);
    EXPECT_FLOAT_EQ(ahrs.latest().accel[0], 0.0f);  // first's value, not second's 9
    EXPECT_EQ(second.popCount(), 1u);               // still drained, just discarded
}

TEST(AhrsController, UpdateDrainsAllQueuedSamples)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    for (int i = 0; i < 5; ++i) {
        ag.pushAccelGyro(0, 0, 1, 0, 0, static_cast<int16_t>(10 * (i + 1)));
    }

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());

    const Capability updated = ahrs.update(0.05f);
    EXPECT_TRUE(any(updated & Capability::Gyro));
    EXPECT_EQ(ag.popCount(), 5u);
    EXPECT_EQ(ag.remaining(), 0u);

    imu_raw_data_t dummy{};
    EXPECT_FALSE(any(ag.readImuRaw(dummy)));
}

TEST(AhrsController, MagOverflowSentinelKeepsLastValidMag)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    ag.pushAccelGyro(0, 0, 1, 0, 0, 0);
    MockSource mag(Capability::Mag);
    mag.pushMag(20, 0, -40);    // valid
    mag.pushMag(-32768, 5, 5);  // sentinel -> rejected, previous value must survive

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.add(mag));
    EXPECT_TRUE(ahrs.begin());

    ahrs.update(0.01f);
    EXPECT_FLOAT_EQ(ahrs.latest().mag[0], 20.0f);
    EXPECT_FLOAT_EQ(ahrs.latest().mag[1], 0.0f);
    EXPECT_FLOAT_EQ(ahrs.latest().mag[2], -40.0f);
}

TEST(AhrsController, BeginFailsWithoutAccelAndGyro)
{
    MockSource mag(Capability::Mag);
    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(mag));
    EXPECT_FALSE(ahrs.begin());
}

TEST(AhrsController, BeginSucceedsWithAccelGyroAndSelectsDefaultFilter)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_EQ(ahrs.filter(), nullptr);
    EXPECT_TRUE(ahrs.begin());
    EXPECT_NE(ahrs.filter(), nullptr);
}

TEST(AhrsController, SelectFilterResetsAttitude)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    for (int i = 0; i < 50; ++i) {
        ag.pushAccelGyro(0, 0, 1, 0, 0, 90);  // sustained yaw rate to move off identity
    }

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());
    ahrs.update(1.0f);

    float w{}, x{}, y{}, z{};
    ahrs.quaternion(w, x, y, z);
    EXPECT_LT(w, 0.999f);  // moved away from identity

    EXPECT_TRUE(ahrs.selectFilter(FilterType::Mahony));
    ahrs.quaternion(w, x, y, z);
    EXPECT_NEAR(w, 1.0f, 1e-4f);
    EXPECT_NEAR(x, 0.0f, 1e-4f);
    EXPECT_NEAR(y, 0.0f, 1e-4f);
    EXPECT_NEAR(z, 0.0f, 1e-4f);
}

TEST(AhrsController, SetFilterInjectsMockAndReceivesUpdates)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    ag.pushAccelGyro(0, 0, 1, 1, 2, 3);
    ag.pushAccelGyro(0, 0, 1, 4, 5, 6);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());  // selects the default (Madgwick) filter first

    MockFilter mock;
    ahrs.setFilter(mock);
    EXPECT_EQ(ahrs.filter(), &mock);

    ahrs.update(0.02f);
    EXPECT_EQ(mock.updateCount(), 2);
    EXPECT_TRUE(mock.lastMagIsNull());  // no mag source was added
}

TEST(AhrsController, MockFilterReceivesValidMagAndLatestAccelGyro)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    ag.pushAccelGyro(1, 2, 3, 4, 5, 6);
    MockSource mag(Capability::Mag);
    mag.pushMag(20, 0, -40);  // valid, no overflow sentinel

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.add(mag));
    EXPECT_TRUE(ahrs.begin());  // selects the default (Madgwick) filter first

    MockFilter mock;
    ahrs.setFilter(mock);

    ahrs.update(0.01f);
    EXPECT_EQ(mock.updateCount(), 1);
    EXPECT_FALSE(mock.lastMagIsNull());  // a valid mag sample must reach the filter as non-null

    const float* last_mag = mock.lastMag();
    EXPECT_FLOAT_EQ(last_mag[0], 20.0f);
    EXPECT_FLOAT_EQ(last_mag[1], 0.0f);
    EXPECT_FLOAT_EQ(last_mag[2], -40.0f);

    const float* last_accel = mock.lastAccel();
    EXPECT_FLOAT_EQ(last_accel[0], 1.0f);
    EXPECT_FLOAT_EQ(last_accel[1], 2.0f);
    EXPECT_FLOAT_EQ(last_accel[2], 3.0f);

    const float* last_gyro = mock.lastGyro();
    EXPECT_FLOAT_EQ(last_gyro[0], 4.0f);
    EXPECT_FLOAT_EQ(last_gyro[1], 5.0f);
    EXPECT_FLOAT_EQ(last_gyro[2], 6.0f);
}

TEST(AhrsController, UpdateWithDtSplitsEvenlyAcrossGyroSamples)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    for (int i = 0; i < 4; ++i) {
        ag.pushAccelGyro(0, 0, 1, 0, 0, 0);
    }

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());

    MockFilter mock;
    ahrs.setFilter(mock);

    ahrs.update(0.08f);  // 0.08 / 4 samples == 0.02 per sample
    EXPECT_EQ(mock.updateCount(), 4);
    for (const float dt : mock.dts()) {
        EXPECT_NEAR(dt, 0.02f, 1e-6f);
    }
}

TEST(AhrsController, EulerAndLatestAreConsistentAfterUpdate)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    for (int i = 0; i < 100; ++i) {
        ag.pushAccelGyro(0, 0, 1, 0, 0, 0);  // static and level
    }

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());  // default Madgwick filter

    ahrs.update(1.0f);  // 100 samples over 1s

    float roll{}, pitch{}, yaw{};
    ahrs.euler(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1.0f);
    EXPECT_NEAR(pitch, 0.0f, 1.0f);

    EXPECT_FLOAT_EQ(ahrs.latest().accel[2], 1.0f);
    EXPECT_FLOAT_EQ(ahrs.latest().gyro[0], 0.0f);
}

TEST(AhrsController, RealClockUpdateSkipsFirstCallThenDrains)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    ag.pushAccelGyro(0, 0, 1, 0, 0, 0);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());

    const Capability first = ahrs.update();  // seeds the clock only, does not drain yet
    EXPECT_FALSE(any(first));
    EXPECT_EQ(ag.popCount(), 0u);

    const Capability second = ahrs.update();  // a previous timestamp now exists
    EXPECT_TRUE(any(second & Capability::Gyro));
    EXPECT_EQ(ag.popCount(), 1u);
}
