/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/ahrs_controller.hpp>
#include <imu/axis_order.hpp>
#include <imu/imu_data_source.hpp>
#include <imu/imu_types.hpp>
#include <math/quaternion.hpp>
#include <cmath>
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
            // Fully drained: release consumed samples so long push/drain loops (e.g. the 3000-sample
            // auto-calibration convergence test) keep the buffer flat instead of growing without
            // bound -- unbounded growth exhausts DRAM on embedded targets (std::bad_alloc). Partial
            // drains (the 32-sample-per-call cap) never reach here, so undrained samples are kept.
            _samples.clear();
            _pos = 0;
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
    // Single 9-axis sample from one source (e.g. a BMI270 with its AUX BMM150 enabled),
    // for exercising accel/gyro vs. mag axis remap independently on the same source.
    void pushAccelGyroMag(const int16_t ax, const int16_t ay, const int16_t az, const int16_t gx, const int16_t gy,
                          const int16_t gz, const int32_t mx, const int32_t my, const int32_t mz)
    {
        imu_raw_data_t d{};
        d.accel[0] = ax;
        d.accel[1] = ay;
        d.accel[2] = az;
        d.gyro[0]  = gx;
        d.gyro[1]  = gy;
        d.gyro[2]  = gz;
        d.mag[0]   = mx;
        d.mag[1]   = my;
        d.mag[2]   = mz;
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

    // Optional IMUDataSource power hooks, counted so tests can assert on the forwarding
    bool sleep() override
    {
        ++_sleep_count;
        return _power_supported;
    }
    bool wakeup() override
    {
        ++_wakeup_count;
        return _power_supported;
    }
    inline void powerSupported(const bool supported)
    {
        _power_supported = supported;
    }
    inline size_t sleepCount() const
    {
        return _sleep_count;
    }
    inline size_t wakeupCount() const
    {
        return _wakeup_count;
    }

private:
    Capability _cap;
    imu_convert_param_t _param{};
    std::vector<imu_raw_data_t> _samples{};
    size_t _pos{};
    size_t _pop_count{};
    bool _power_supported{true};
    size_t _sleep_count{};
    size_t _wakeup_count{};
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

// ---------------------------------------------------------------------------------------
// Axis remap (Task 9)
// ---------------------------------------------------------------------------------------

TEST(AhrsController, AxisOrderRoundTripAndRejectsUnknownSource)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    MockSource other(Capability::Mag);  // never added

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));

    AxisOrder order;
    order.permutation = AxisPermutation::YXZ;
    order.invert_y    = true;
    EXPECT_TRUE(ahrs.writeAxisOrder(ag, order));

    AxisOrder readback;
    EXPECT_TRUE(ahrs.readAxisOrder(ag, readback));
    EXPECT_EQ(readback.permutation, AxisPermutation::YXZ);
    EXPECT_TRUE(readback.invert_y);

    EXPECT_FALSE(ahrs.writeAxisOrder(other, order));
    EXPECT_FALSE(ahrs.readAxisOrder(other, readback));
}

TEST(AhrsController, AxisOrderIsAppliedBeforeLatestAndFilter)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    ag.pushAccelGyro(1, 2, 3, 4, 5, 6);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));

    AxisOrder order;
    order.permutation = AxisPermutation::YXZ;  // output = (Y, X, Z)
    order.invert_z    = true;
    EXPECT_TRUE(ahrs.writeAxisOrder(ag, order));
    EXPECT_TRUE(ahrs.begin());

    ahrs.update(0.01f);

    const imu_physical_data_t& latest = ahrs.latest();
    EXPECT_FLOAT_EQ(latest.accel[0], 2.0f);
    EXPECT_FLOAT_EQ(latest.accel[1], 1.0f);
    EXPECT_FLOAT_EQ(latest.accel[2], -3.0f);
    EXPECT_FLOAT_EQ(latest.gyro[0], 5.0f);
    EXPECT_FLOAT_EQ(latest.gyro[1], 4.0f);
    EXPECT_FLOAT_EQ(latest.gyro[2], -6.0f);
}

TEST(AhrsController, MagAxisOrderRoundTripAndRejectsUnknownSource)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    MockSource other(Capability::Mag);  // never added

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));

    AxisOrder order;
    order.invert_x = true;
    order.invert_z = true;
    EXPECT_TRUE(ahrs.writeMagAxisOrder(ag, order));

    AxisOrder readback;
    EXPECT_TRUE(ahrs.readMagAxisOrder(ag, readback));
    EXPECT_EQ(readback.permutation, AxisPermutation::XYZ);
    EXPECT_TRUE(readback.invert_x);
    EXPECT_FALSE(readback.invert_y);
    EXPECT_TRUE(readback.invert_z);

    EXPECT_FALSE(ahrs.writeMagAxisOrder(other, order));
    EXPECT_FALSE(ahrs.readMagAxisOrder(other, readback));
}

TEST(AhrsController, MagAxisOrderDefaultsToIdentityIndependentlyOfAccelGyroAxisOrder)
{
    // CoreS3-style setup: accel/gyro keep their own (non-identity) remap, mag on the very
    // same source is left at the default identity because writeMagAxisOrder() was never
    // called for it.
    MockSource agm(Capability::Accel | Capability::Gyro | Capability::Mag);
    agm.pushAccelGyroMag(1, 2, 3, 4, 5, 6, 7, 8, 9);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(agm));

    AxisOrder order;
    order.permutation = AxisPermutation::YXZ;  // output = (Y, X, Z)
    order.invert_z    = true;
    EXPECT_TRUE(ahrs.writeAxisOrder(agm, order));
    EXPECT_TRUE(ahrs.begin());

    ahrs.update(0.01f);

    const imu_physical_data_t& latest = ahrs.latest();
    // accel/gyro: YXZ + invert_z, same as AxisOrderIsAppliedBeforeLatestAndFilter above.
    EXPECT_FLOAT_EQ(latest.accel[0], 2.0f);
    EXPECT_FLOAT_EQ(latest.accel[1], 1.0f);
    EXPECT_FLOAT_EQ(latest.accel[2], -3.0f);
    EXPECT_FLOAT_EQ(latest.gyro[0], 5.0f);
    EXPECT_FLOAT_EQ(latest.gyro[1], 4.0f);
    EXPECT_FLOAT_EQ(latest.gyro[2], -6.0f);
    // mag: no writeMagAxisOrder() call -> identity, untouched by the accel/gyro remap.
    EXPECT_FLOAT_EQ(latest.mag[0], 7.0f);
    EXPECT_FLOAT_EQ(latest.mag[1], 8.0f);
    EXPECT_FLOAT_EQ(latest.mag[2], 9.0f);
}

TEST(AhrsController, MagAxisOrderAppliedIndependentlyOfAccelGyroAxisOrderOnSameSource)
{
    // AtomS3R-style setup: accel/gyro and mag come from the same 9-axis source but need two
    // genuinely different corrections.
    MockSource agm(Capability::Accel | Capability::Gyro | Capability::Mag);
    agm.pushAccelGyroMag(1, 2, 3, 4, 5, 6, 7, 8, 9);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(agm));

    AxisOrder ag_order;
    ag_order.permutation = AxisPermutation::YXZ;
    ag_order.invert_y    = true;
    EXPECT_TRUE(ahrs.writeAxisOrder(agm, ag_order));

    AxisOrder mag_order;
    mag_order.invert_x = true;
    mag_order.invert_z = true;
    EXPECT_TRUE(ahrs.writeMagAxisOrder(agm, mag_order));

    EXPECT_TRUE(ahrs.begin());
    ahrs.update(0.01f);

    const imu_physical_data_t& latest = ahrs.latest();
    // accel/gyro: YXZ + invert_y -> (Y, -X, Z)
    EXPECT_FLOAT_EQ(latest.accel[0], 2.0f);
    EXPECT_FLOAT_EQ(latest.accel[1], -1.0f);
    EXPECT_FLOAT_EQ(latest.accel[2], 3.0f);
    EXPECT_FLOAT_EQ(latest.gyro[0], 5.0f);
    EXPECT_FLOAT_EQ(latest.gyro[1], -4.0f);
    EXPECT_FLOAT_EQ(latest.gyro[2], 6.0f);
    // mag: identity permutation + invert_x/invert_z -> (-X, Y, -Z), a completely different
    // transform from the one applied to accel/gyro above.
    EXPECT_FLOAT_EQ(latest.mag[0], -7.0f);
    EXPECT_FLOAT_EQ(latest.mag[1], 8.0f);
    EXPECT_FLOAT_EQ(latest.mag[2], -9.0f);
}

TEST(AhrsController, UserAxisOrderRoundTripDefaultsToIdentity)
{
    AhrsController ahrs;

    AxisOrder readback;
    ahrs.readUserAxisOrder(readback);
    EXPECT_EQ(readback.permutation, AxisPermutation::XYZ);
    EXPECT_FALSE(readback.invert_x);
    EXPECT_FALSE(readback.invert_y);
    EXPECT_FALSE(readback.invert_z);

    AxisOrder user;
    user.permutation = AxisPermutation::ZXY;
    user.invert_z    = true;
    ahrs.writeUserAxisOrder(user);

    ahrs.readUserAxisOrder(readback);
    EXPECT_EQ(readback.permutation, AxisPermutation::ZXY);
    EXPECT_FALSE(readback.invert_x);
    EXPECT_FALSE(readback.invert_y);
    EXPECT_TRUE(readback.invert_z);
}

TEST(AhrsController, UserAxisOrderComposesOnTopOfFixedAxisOrder)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    ag.pushAccelGyro(1, 2, 3, 4, 5, 6);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));

    AxisOrder fixed;  // board correction, as a preset would install it
    fixed.permutation = AxisPermutation::YXZ;
    fixed.invert_y    = true;
    EXPECT_TRUE(ahrs.writeAxisOrder(ag, fixed));

    AxisOrder user;  // user reassignment layered on top
    user.invert_x = true;
    ahrs.writeUserAxisOrder(user);

    EXPECT_TRUE(ahrs.begin());
    ahrs.update(0.01f);

    const imu_physical_data_t& latest = ahrs.latest();
    // fixed: (1,2,3) -> (2,-1,3); user then negates X -> (-2,-1,3)
    EXPECT_FLOAT_EQ(latest.accel[0], -2.0f);
    EXPECT_FLOAT_EQ(latest.accel[1], -1.0f);
    EXPECT_FLOAT_EQ(latest.accel[2], 3.0f);
    EXPECT_FLOAT_EQ(latest.gyro[0], -5.0f);
    EXPECT_FLOAT_EQ(latest.gyro[1], -4.0f);
    EXPECT_FLOAT_EQ(latest.gyro[2], 6.0f);

    // readAxisOrder still reports the fixed order, never the composed one
    AxisOrder readback;
    EXPECT_TRUE(ahrs.readAxisOrder(ag, readback));
    EXPECT_EQ(readback.permutation, AxisPermutation::YXZ);
    EXPECT_TRUE(readback.invert_y);
    EXPECT_FALSE(readback.invert_x);
}

TEST(AhrsController, UserAxisOrderAppliesToMagAsWell)
{
    MockSource agm(Capability::Accel | Capability::Gyro | Capability::Mag);
    agm.pushAccelGyroMag(1, 2, 3, 4, 5, 6, 7, 8, 9);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(agm));

    AxisOrder mag_fixed;  // AUX die mounted differently, accel/gyro left at identity
    mag_fixed.invert_x = true;
    mag_fixed.invert_z = true;
    EXPECT_TRUE(ahrs.writeMagAxisOrder(agm, mag_fixed));

    AxisOrder user;
    user.permutation = AxisPermutation::YXZ;
    ahrs.writeUserAxisOrder(user);

    EXPECT_TRUE(ahrs.begin());
    ahrs.update(0.01f);

    const imu_physical_data_t& latest = ahrs.latest();
    // accel has no fixed correction, so the user swap alone applies: (1,2,3) -> (2,1,3)
    EXPECT_FLOAT_EQ(latest.accel[0], 2.0f);
    EXPECT_FLOAT_EQ(latest.accel[1], 1.0f);
    EXPECT_FLOAT_EQ(latest.accel[2], 3.0f);
    // mag fixed: (7,8,9) -> (-7,8,-9); user swap of X/Y -> (8,-7,-9)
    EXPECT_FLOAT_EQ(latest.mag[0], 8.0f);
    EXPECT_FLOAT_EQ(latest.mag[1], -7.0f);
    EXPECT_FLOAT_EQ(latest.mag[2], -9.0f);
}

TEST(AhrsController, UserAxisOrderSetBeforeAddAppliesToLaterSources)
{
    AhrsController ahrs;

    AxisOrder user;
    user.permutation = AxisPermutation::YXZ;
    ahrs.writeUserAxisOrder(user);  // installed before any source exists

    MockSource ag(Capability::Accel | Capability::Gyro);
    ag.pushAccelGyro(1, 2, 3, 4, 5, 6);
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());
    ahrs.update(0.01f);

    const imu_physical_data_t& latest = ahrs.latest();
    EXPECT_FLOAT_EQ(latest.accel[0], 2.0f);
    EXPECT_FLOAT_EQ(latest.accel[1], 1.0f);
    EXPECT_FLOAT_EQ(latest.accel[2], 3.0f);
}

TEST(AhrsController, CapabilityQueriesReflectAddedSources)
{
    AhrsController ahrs;
    EXPECT_FALSE(ahrs.hasAccel());
    EXPECT_FALSE(ahrs.hasGyro());
    EXPECT_FALSE(ahrs.hasTemp());
    EXPECT_FALSE(ahrs.hasValidMag());

    MockSource ag(Capability::Accel | Capability::Gyro | Capability::Temp);
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.hasAccel());
    EXPECT_TRUE(ahrs.hasGyro());
    EXPECT_TRUE(ahrs.hasTemp());
    EXPECT_FALSE(ahrs.hasValidMag());  // no magnetometer source at all
}

TEST(AhrsController, HasValidMagTurnsTrueOnlyAfterAValidMagSample)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    MockSource mag(Capability::Mag);
    ag.pushAccelGyro(0, 0, 1, 0, 0, 0);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.add(mag));
    EXPECT_TRUE(ahrs.begin());

    ahrs.update(0.01f);
    EXPECT_FALSE(ahrs.hasValidMag());  // mag source present, but no sample has arrived yet

    mag.pushMag(20, 0, -40);
    ahrs.update(0.01f);
    EXPECT_TRUE(ahrs.hasValidMag());
}

TEST(AhrsController, LastMicrosIsStampedOnlyBySelfClockedUpdate)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    ag.pushAccelGyro(0, 0, 1, 0, 0, 0);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());
    EXPECT_EQ(ahrs.lastMicros(), 0u);

    ahrs.update(0.01f);  // explicit-dt overload bypasses the clock
    EXPECT_EQ(ahrs.lastMicros(), 0u);

    ahrs.update();  // self-clocked overload stamps it
    EXPECT_NE(ahrs.lastMicros(), 0u);
}

TEST(AhrsController, SleepAndWakeupReachEverySource)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    MockSource mag(Capability::Mag);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.add(mag));

    EXPECT_TRUE(ahrs.sleep());
    EXPECT_EQ(ag.sleepCount(), 1u);
    EXPECT_EQ(mag.sleepCount(), 1u);

    EXPECT_TRUE(ahrs.wakeup());
    EXPECT_EQ(ag.wakeupCount(), 1u);
    EXPECT_EQ(mag.wakeupCount(), 1u);
}

TEST(AhrsController, SleepAsksEverySourceEvenWhenNoneSupportsIt)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    MockSource mag(Capability::Mag);
    ag.powerSupported(false);
    mag.powerSupported(false);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.add(mag));

    EXPECT_FALSE(ahrs.sleep());  // nothing could be suspended...
    EXPECT_EQ(ag.sleepCount(), 1u);
    EXPECT_EQ(mag.sleepCount(), 1u);  // ...but both were still asked

    EXPECT_FALSE(ahrs.wakeup());
    EXPECT_EQ(ag.wakeupCount(), 1u);
    EXPECT_EQ(mag.wakeupCount(), 1u);
}

TEST(AhrsController, SleepSucceedsIfAnySingleSourceSupportsIt)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    MockSource mag(Capability::Mag);
    ag.powerSupported(false);  // only the magnetometer can sleep

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.add(mag));

    EXPECT_TRUE(ahrs.sleep());
    EXPECT_EQ(ag.sleepCount(), 1u);
    EXPECT_EQ(mag.sleepCount(), 1u);
}

TEST(AhrsController, WakeupDiscardsTheClockSoTheNextUpdateReseeds)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    ag.pushAccelGyro(0, 0, 1, 0, 0, 0);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());

    EXPECT_FALSE(any(ahrs.update()));  // seeds the clock
    EXPECT_TRUE(any(ahrs.update() & Capability::Gyro));
    EXPECT_EQ(ag.popCount(), 1u);

    EXPECT_TRUE(ahrs.wakeup());

    // The clock was discarded, so this behaves like the very first call again: no drain,
    // no dt spanning the sleep handed to the filter.
    ag.pushAccelGyro(0, 0, 1, 0, 0, 0);
    EXPECT_FALSE(any(ahrs.update()));
    EXPECT_EQ(ag.popCount(), 1u);

    EXPECT_TRUE(any(ahrs.update() & Capability::Gyro));
    EXPECT_EQ(ag.popCount(), 2u);
}

// ---------------------------------------------------------------------------------------
// Auto-calibration (Task 9)
// ---------------------------------------------------------------------------------------

TEST(AhrsController, AutoCalibrationReadWriteRoundTrip)
{
    AhrsController ahrs;
    uint8_t accel{}, gyro{}, mag{};
    ahrs.readAutoCalibration(accel, gyro, mag);
    EXPECT_EQ(accel, 0);
    EXPECT_EQ(gyro, 0);
    EXPECT_EQ(mag, 0);

    ahrs.writeAutoCalibration(10, 20, 30);
    ahrs.readAutoCalibration(accel, gyro, mag);
    EXPECT_EQ(accel, 10);
    EXPECT_EQ(gyro, 20);
    EXPECT_EQ(mag, 30);
}

TEST(AhrsController, AutoCalibrationConvergesGyroBias)
{
    MockSource ag(Capability::Accel | Capability::Gyro);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());
    ahrs.writeAutoCalibration(0, 128, 0);

    // 30s of stationary data (dt = 0.01s) with a constant 2dps gyro bias, one sample per
    // update() call so the controller's 32-sample-per-call drain cap never kicks in.
    constexpr int SAMPLE_COUNT = 3000;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        ag.pushAccelGyro(0, 0, 1, 0, 0, 2);
        ahrs.update(0.01f);
    }

    EXPECT_NEAR(ahrs.latest().gyro[2], 0.0f, 0.3f);
}

TEST(AhrsController, MagCalibrationRadiusIsLearnedFromTheFieldByDefault)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    MockSource mag(Capability::Mag);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.add(mag));
    EXPECT_TRUE(ahrs.begin());
    EXPECT_FLOAT_EQ(ahrs.readMagCalibrationRadius(), 0.0f);  // unseeded until data arrives
    ahrs.writeAutoCalibration(0, 0, 128);

    // A 50uT field, i.e. roughly what a magnetometer sees in Japan and twice the fixed 24uT
    // sphere M5Unified assumes. Sweeping the direction keeps the samples on that sphere.
    constexpr float FIELD_UT{50.0f};
    for (int i = 0; i < 3000; ++i) {
        const float angle = static_cast<float>(i) * 0.02f;
        ag.pushAccelGyro(0, 0, 1, 0, 0, 0);
        mag.pushMag(static_cast<int32_t>(FIELD_UT * std::cos(angle)),
                    static_cast<int32_t>(FIELD_UT * std::sin(angle)), 0);
        ahrs.update(0.01f);
    }

    // The estimate must land on the field actually present, not on a hard-coded radius
    EXPECT_NEAR(ahrs.readMagCalibrationRadius(), FIELD_UT, 10.0f);
}

TEST(AhrsController, MagCalibrationRadiusCanBePinnedAndReleased)
{
    AhrsController ahrs;

    ahrs.writeMagCalibrationRadius(48.0f);
    EXPECT_FLOAT_EQ(ahrs.readMagCalibrationRadius(), 48.0f);

    // Back to estimating: the pinned value is dropped so the next samples re-seed it
    ahrs.writeMagCalibrationRadius(0.0f);
    EXPECT_FLOAT_EQ(ahrs.readMagCalibrationRadius(), 0.0f);

    ahrs.writeMagCalibrationRadius(30.0f);
    EXPECT_FLOAT_EQ(ahrs.readMagCalibrationRadius(), 30.0f);
    ahrs.writeMagCalibrationRadius(-1.0f);  // negative is treated as "estimate it"
    EXPECT_FLOAT_EQ(ahrs.readMagCalibrationRadius(), 0.0f);
}

TEST(AhrsController, AutoCalibrationOffWhenStrengthZero)
{
    MockSource ag(Capability::Accel | Capability::Gyro);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());
    // Auto-calibration strength defaults to 0 (off); no writeAutoCalibration() call.

    for (int i = 0; i < 500; ++i) {
        ag.pushAccelGyro(0, 0, 1, 0, 0, 2);
        ahrs.update(0.01f);
    }

    EXPECT_NEAR(ahrs.latest().gyro[2], 2.0f, 1.0e-4f);  // bias left untouched
}

TEST(AhrsController, CalibrationExportImportRoundTrip)
{
    AhrsController ahrs;
    AhrsController::calibration_data_t data{};
    data.version         = 1;
    data.accel_offset[0] = 0.01f;
    data.accel_offset[1] = -0.02f;
    data.accel_offset[2] = 0.03f;
    data.gyro_offset[0]  = 1.0f;
    data.gyro_offset[1]  = -0.5f;
    data.gyro_offset[2]  = 2.0f;
    data.mag_offset[0]   = 5.0f;
    data.mag_offset[1]   = -5.0f;
    data.mag_offset[2]   = 10.0f;

    EXPECT_TRUE(ahrs.writeCalibration(data));

    AhrsController::calibration_data_t readback{};
    EXPECT_TRUE(ahrs.readCalibration(readback));
    EXPECT_EQ(readback.version, 1u);
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(readback.accel_offset[i], data.accel_offset[i]);
        EXPECT_FLOAT_EQ(readback.gyro_offset[i], data.gyro_offset[i]);
        EXPECT_FLOAT_EQ(readback.mag_offset[i], data.mag_offset[i]);
    }
}

TEST(AhrsController, WriteCalibrationRejectsUnknownVersion)
{
    AhrsController ahrs;
    AhrsController::calibration_data_t data{};
    data.version = 999;
    EXPECT_FALSE(ahrs.writeCalibration(data));
}

TEST(AhrsController, WrittenCalibrationOffsetIsSubtractedFromSamples)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    ag.pushAccelGyro(0, 0, 1, 0, 0, 5);

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());

    AhrsController::calibration_data_t data{};
    data.version        = 1;
    data.gyro_offset[2] = 2.0f;
    EXPECT_TRUE(ahrs.writeCalibration(data));

    ahrs.update(0.01f);
    EXPECT_FLOAT_EQ(ahrs.latest().gyro[2], 3.0f);  // 5 - 2 offset, strength still 0
}

TEST(AhrsController, CalibrateGyroExplicitDtSetsOffsetToMean)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    constexpr int SAMPLE_COUNT = 20;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        ag.pushAccelGyro(0, 0, 1, 0, 0, 3);  // constant 3dps bias
    }

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());

    EXPECT_TRUE(ahrs.calibrateGyro(SAMPLE_COUNT, 0.01f));

    AhrsController::calibration_data_t data{};
    ahrs.readCalibration(data);
    EXPECT_NEAR(data.gyro_offset[2], 3.0f, 1.0e-4f);

    ag.pushAccelGyro(0, 0, 1, 0, 0, 3);
    ahrs.update(0.01f);
    EXPECT_NEAR(ahrs.latest().gyro[2], 0.0f, 1.0e-4f);
}

TEST(AhrsController, CalibrateGyroFailsWithoutGyroSource)
{
    MockSource mag(Capability::Mag);
    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(mag));
    EXPECT_FALSE(ahrs.calibrateGyro(10, 0.01f));
}

// ---------------------------------------------------------------------------------------
// Yaw tare
// ---------------------------------------------------------------------------------------

TEST(AhrsController, TareYawZeroesYawKeepsRollPitchAndClearRestoresPreTareYaw)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    for (int i = 0; i < 50; ++i) {
        ag.pushAccelGyro(0, 0, 1, 0, 0, 90);  // level accel, sustained yaw rate -> yaw drifts, roll/pitch stay ~0
    }

    AhrsController ahrs;
    EXPECT_TRUE(ahrs.add(ag));
    EXPECT_TRUE(ahrs.begin());  // default Madgwick filter
    ahrs.update(1.0f);          // dt split evenly across the 50 drained samples

    float roll_before{}, pitch_before{}, yaw_before{};
    ahrs.euler(roll_before, pitch_before, yaw_before);
    EXPECT_GT(std::fabs(yaw_before), 5.0f);  // moved well off zero yaw

    ahrs.tareYaw();

    float roll_after{}, pitch_after{}, yaw_after{};
    ahrs.euler(roll_after, pitch_after, yaw_after);
    EXPECT_NEAR(yaw_after, 0.0f, 1.0e-3f);
    EXPECT_NEAR(roll_after, roll_before, 1.0e-3f);
    EXPECT_NEAR(pitch_after, pitch_before, 1.0e-3f);

    // quaternion() and euler() must agree on the tared attitude.
    float w{}, x{}, y{}, z{};
    ahrs.quaternion(w, x, y, z);
    float roll_from_q{}, pitch_from_q{}, yaw_from_q{};
    m5::math::Quaternion(w, x, y, z).toEuler(roll_from_q, pitch_from_q, yaw_from_q);
    EXPECT_NEAR(roll_from_q, roll_after, 1.0e-4f);
    EXPECT_NEAR(pitch_from_q, pitch_after, 1.0e-4f);
    EXPECT_NEAR(yaw_from_q, yaw_after, 1.0e-4f);

    ahrs.clearYawTare();

    float roll_cleared{}, pitch_cleared{}, yaw_cleared{};
    ahrs.euler(roll_cleared, pitch_cleared, yaw_cleared);
    EXPECT_NEAR(yaw_cleared, yaw_before, 1.0e-3f);
    EXPECT_NEAR(roll_cleared, roll_before, 1.0e-3f);
    EXPECT_NEAR(pitch_cleared, pitch_before, 1.0e-3f);
}

TEST(AhrsController, TareYawIsNoOpWithoutAFilter)
{
    AhrsController ahrs;  // no source/filter set up yet
    ahrs.tareYaw();       // must not crash

    float w{}, x{}, y{}, z{};
    ahrs.quaternion(w, x, y, z);
    EXPECT_FLOAT_EQ(w, 1.0f);
    EXPECT_FLOAT_EQ(x, 0.0f);
    EXPECT_FLOAT_EQ(y, 0.0f);
    EXPECT_FLOAT_EQ(z, 0.0f);
}
