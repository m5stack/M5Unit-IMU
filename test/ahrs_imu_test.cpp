/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/ahrs_imu.hpp>
#include <imu/axis_order.hpp>
#include <imu/imu_data_source.hpp>
#include <imu/imu_types.hpp>
#include <vector>

using namespace m5::imu;

namespace {

// Minimal pop-semantics mock (same contract as the one in ahrs_controller_test.cpp): an
// identity imu_convert_param_t keeps pushed raw values equal to the resulting physical values.
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
            _samples.clear();
            _pos = 0;
            return Capability::None;
        }
        out = _samples[_pos++];
        return _cap;
    }
    void imuConvertParam(imu_convert_param_t& out) const override
    {
        out = _param;
    }

    void push(const int16_t ax, const int16_t ay, const int16_t az, const int16_t gx, const int16_t gy,
              const int16_t gz, const int32_t mx, const int32_t my, const int32_t mz, const int16_t temp)
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
        d.temp     = temp;
        _samples.push_back(d);
    }

private:
    Capability _cap;
    imu_convert_param_t _param{};
    std::vector<imu_raw_data_t> _samples{};
    size_t _pos{};
};

constexpr Capability NINE_AXIS{Capability::Accel | Capability::Gyro | Capability::Mag | Capability::Temp};

}  // namespace

// ---------------------------------------------------------------------------------------
// M5Unified compatible getters
// ---------------------------------------------------------------------------------------

TEST(AhrsImu, GettersReturnLatestSampleAndCapabilityFlags)
{
    MockSource src(NINE_AXIS);
    src.push(1, 2, 3, 4, 5, 6, 7, 8, 9, 25);

    AhrsImu imu;
    EXPECT_TRUE(imu.add(src));
    EXPECT_TRUE(imu.begin());
    imu.update(0.01f);

    float x{}, y{}, z{};
    EXPECT_TRUE(imu.getAccel(&x, &y, &z));
    EXPECT_FLOAT_EQ(x, 1.0f);
    EXPECT_FLOAT_EQ(y, 2.0f);
    EXPECT_FLOAT_EQ(z, 3.0f);

    EXPECT_TRUE(imu.getGyro(&x, &y, &z));
    EXPECT_FLOAT_EQ(x, 4.0f);
    EXPECT_FLOAT_EQ(y, 5.0f);
    EXPECT_FLOAT_EQ(z, 6.0f);

    EXPECT_TRUE(imu.getMag(&x, &y, &z));
    EXPECT_FLOAT_EQ(x, 7.0f);
    EXPECT_FLOAT_EQ(y, 8.0f);
    EXPECT_FLOAT_EQ(z, 9.0f);

    float temp{};
    EXPECT_TRUE(imu.getTemp(&temp));
    EXPECT_FLOAT_EQ(temp, 25.0f);
}

TEST(AhrsImu, GettersReportFalseForAbsentCapabilities)
{
    MockSource ag(Capability::Accel | Capability::Gyro);
    ag.push(1, 2, 3, 4, 5, 6, 0, 0, 0, 0);

    AhrsImu imu;
    EXPECT_TRUE(imu.add(ag));
    EXPECT_TRUE(imu.begin());
    imu.update(0.01f);

    float x{}, y{}, z{};
    EXPECT_TRUE(imu.getAccel(&x, &y, &z));
    EXPECT_TRUE(imu.getGyro(&x, &y, &z));
    EXPECT_FALSE(imu.getMag(&x, &y, &z));  // 6-axis setup: no valid mag ever seen
    float temp{};
    EXPECT_FALSE(imu.getTemp(&temp));
}

TEST(AhrsImu, GettersToleratePartialNullptrArguments)
{
    MockSource src(NINE_AXIS);
    src.push(1, 2, 3, 4, 5, 6, 7, 8, 9, 25);

    AhrsImu imu;
    EXPECT_TRUE(imu.add(src));
    EXPECT_TRUE(imu.begin());
    imu.update(0.01f);

    float only_z{};
    EXPECT_TRUE(imu.getAccel(nullptr, nullptr, &only_z));
    EXPECT_FLOAT_EQ(only_z, 3.0f);
    EXPECT_TRUE(imu.getGyro(nullptr, nullptr, nullptr));
    EXPECT_TRUE(imu.getTemp(nullptr));
}

TEST(AhrsImu, DataSuffixedAliasesForwardToTheirGetters)
{
    MockSource src(NINE_AXIS);
    src.push(1, 2, 3, 4, 5, 6, 7, 8, 9, 25);

    AhrsImu imu;
    EXPECT_TRUE(imu.add(src));
    EXPECT_TRUE(imu.begin());
    imu.update(0.01f);

    float ax{}, ay{}, az{}, gx{}, gy{}, gz{}, mx{}, my{}, mz{};
    EXPECT_TRUE(imu.getAccelData(&ax, &ay, &az));
    EXPECT_TRUE(imu.getGyroData(&gx, &gy, &gz));
    EXPECT_TRUE(imu.getGyroMag(&mx, &my, &mz));  // M5Unified spells the mag alias this way
    EXPECT_FLOAT_EQ(ax, 1.0f);
    EXPECT_FLOAT_EQ(gy, 5.0f);
    EXPECT_FLOAT_EQ(mz, 9.0f);
}

TEST(AhrsImu, IsEnabledRequiresBothAccelAndGyro)
{
    AhrsImu imu;
    EXPECT_FALSE(imu.isEnabled());

    MockSource accel_only(Capability::Accel);
    EXPECT_TRUE(imu.add(accel_only));
    EXPECT_FALSE(imu.isEnabled());

    MockSource gyro_only(Capability::Gyro);
    EXPECT_TRUE(imu.add(gyro_only));
    EXPECT_TRUE(imu.isEnabled());
}

// ---------------------------------------------------------------------------------------
// imu_data_t / attitude
// ---------------------------------------------------------------------------------------

TEST(AhrsImu, GetImuDataCopiesLatestSampleAndTimestamp)
{
    MockSource src(NINE_AXIS);
    src.push(1, 2, 3, 4, 5, 6, 7, 8, 9, 25);

    AhrsImu imu;
    EXPECT_TRUE(imu.add(src));
    EXPECT_TRUE(imu.begin());
    imu.update(0.01f);

    AhrsImu::imu_data_t data{};
    imu.getImuData(&data);
    EXPECT_FLOAT_EQ(data.accel.x, 1.0f);
    EXPECT_FLOAT_EQ(data.accel.y, 2.0f);
    EXPECT_FLOAT_EQ(data.accel.z, 3.0f);
    EXPECT_FLOAT_EQ(data.gyro.x, 4.0f);
    EXPECT_FLOAT_EQ(data.gyro.z, 6.0f);
    EXPECT_FLOAT_EQ(data.mag.x, 7.0f);
    EXPECT_FLOAT_EQ(data.mag.z, 9.0f);
    EXPECT_EQ(data.usec, static_cast<uint32_t>(imu.lastMicros()));

    // The union aliases must agree with the named members
    EXPECT_FLOAT_EQ(data.value[0], data.accel.x);
    EXPECT_FLOAT_EQ(data.value[4], data.gyro.y);
    EXPECT_FLOAT_EQ(data.value[8], data.mag.z);
    EXPECT_FLOAT_EQ(data.sensor[2].x, data.mag.x);
}

TEST(AhrsImu, GetImuDataByReferenceMatchesPointerOverloadAndIgnoresNullptr)
{
    MockSource src(NINE_AXIS);
    src.push(1, 2, 3, 4, 5, 6, 7, 8, 9, 25);

    AhrsImu imu;
    EXPECT_TRUE(imu.add(src));
    EXPECT_TRUE(imu.begin());
    imu.update(0.01f);

    imu.getImuData(nullptr);  // must not crash

    AhrsImu::imu_data_t copy{};
    imu.getImuData(&copy);
    const AhrsImu::imu_data_t& ref = imu.getImuData();
    for (int i = 0; i < 9; ++i) {
        EXPECT_FLOAT_EQ(ref.value[i], copy.value[i]);
    }
    EXPECT_EQ(ref.usec, copy.usec);
}

TEST(AhrsImu, GetAhrsDataMapsPitchRollYawInM5UnifiedArgumentOrder)
{
    MockSource src(Capability::Accel | Capability::Gyro);
    for (int i = 0; i < 8; ++i) {
        src.push(0, 0, 1, 3, 5, 7, 0, 0, 0, 0);
    }

    AhrsImu imu;
    EXPECT_TRUE(imu.add(src));
    EXPECT_TRUE(imu.begin());
    for (int i = 0; i < 8; ++i) {
        imu.update(0.01f);
    }

    float roll{}, pitch{}, yaw{};
    imu.euler(roll, pitch, yaw);

    float out_pitch{}, out_roll{}, out_yaw{};
    imu.getAhrsData(&out_pitch, &out_roll, &out_yaw);
    EXPECT_FLOAT_EQ(out_pitch, pitch);
    EXPECT_FLOAT_EQ(out_roll, roll);
    EXPECT_FLOAT_EQ(out_yaw, yaw);

    imu.getAhrsData(nullptr, nullptr, nullptr);  // must not crash
}

// ---------------------------------------------------------------------------------------
// Calibration aliases
// ---------------------------------------------------------------------------------------

TEST(AhrsImu, SetCalibrationForwardsToAutoCalibrationStrengths)
{
    AhrsImu imu;
    imu.setCalibration(10, 20, 30);

    uint8_t accel{}, gyro{}, mag{};
    imu.readAutoCalibration(accel, gyro, mag);
    EXPECT_EQ(accel, 10);
    EXPECT_EQ(gyro, 20);
    EXPECT_EQ(mag, 30);
}

TEST(AhrsImu, ClearOffsetDataZeroesEveryStoredOffset)
{
    AhrsImu imu;

    AhrsController::calibration_data_t seeded;
    seeded.accel_offset[0] = 0.25f;
    seeded.gyro_offset[1]  = -1.5f;
    seeded.mag_offset[2]   = 12.0f;
    EXPECT_TRUE(imu.writeCalibration(seeded));

    imu.clearOffsetData();

    AhrsController::calibration_data_t out;
    EXPECT_TRUE(imu.readCalibration(out));
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(out.accel_offset[i], 0.0f);
        EXPECT_FLOAT_EQ(out.gyro_offset[i], 0.0f);
        EXPECT_FLOAT_EQ(out.mag_offset[i], 0.0f);
    }
}

// ---------------------------------------------------------------------------------------
// setAxisOrder family
// ---------------------------------------------------------------------------------------

TEST(AhrsImu, SetAxisOrderInstallsTheUserAxisOrder)
{
    AhrsImu imu;
    EXPECT_TRUE(imu.setAxisOrder(AhrsImu::axis_y_neg, AhrsImu::axis_x_pos, AhrsImu::axis_z_pos));

    AxisOrder user;
    imu.readUserAxisOrder(user);
    EXPECT_EQ(user.permutation, AxisPermutation::YXZ);
    EXPECT_TRUE(user.invert_x);  // the sign rides with the output slot, here slot X
    EXPECT_FALSE(user.invert_y);
    EXPECT_FALSE(user.invert_z);
}

TEST(AhrsImu, SetAxisOrderReordersReportedSamples)
{
    MockSource src(NINE_AXIS);
    src.push(1, 2, 3, 4, 5, 6, 7, 8, 9, 25);

    AhrsImu imu;
    EXPECT_TRUE(imu.add(src));
    EXPECT_TRUE(imu.setAxisOrder(AhrsImu::axis_y_pos, AhrsImu::axis_x_pos, AhrsImu::axis_z_pos));
    EXPECT_TRUE(imu.begin());
    imu.update(0.01f);

    float x{}, y{}, z{};
    EXPECT_TRUE(imu.getAccel(&x, &y, &z));
    EXPECT_FLOAT_EQ(x, 2.0f);  // X and Y swapped
    EXPECT_FLOAT_EQ(y, 1.0f);
    EXPECT_FLOAT_EQ(z, 3.0f);

    EXPECT_TRUE(imu.getMag(&x, &y, &z));  // the user order reaches mag too
    EXPECT_FLOAT_EQ(x, 8.0f);
    EXPECT_FLOAT_EQ(y, 7.0f);
    EXPECT_FLOAT_EQ(z, 9.0f);
}

TEST(AhrsImu, SetAxisOrderRejectsNonPermutationsAndLeavesUserOrderUntouched)
{
    AhrsImu imu;
    EXPECT_TRUE(imu.setAxisOrder(AhrsImu::axis_z_pos, AhrsImu::axis_y_pos, AhrsImu::axis_x_pos));

    // Duplicate axis (X twice) is not a permutation
    EXPECT_FALSE(imu.setAxisOrder(AhrsImu::axis_x_pos, AhrsImu::axis_x_neg, AhrsImu::axis_z_pos));
    // ...and the previously installed order must survive the rejection
    AxisOrder user;
    imu.readUserAxisOrder(user);
    EXPECT_EQ(user.permutation, AxisPermutation::ZYX);
}

TEST(AhrsImu, RightHandedDerivesThePositiveThirdAxis)
{
    AhrsImu imu;
    EXPECT_TRUE(imu.setAxisOrderRightHanded(AhrsImu::axis_x_pos, AhrsImu::axis_y_pos));

    AxisOrder user;
    imu.readUserAxisOrder(user);
    EXPECT_EQ(user.permutation, AxisPermutation::XYZ);
    EXPECT_FALSE(user.invert_x);
    EXPECT_FALSE(user.invert_y);
    EXPECT_FALSE(user.invert_z);  // +X, +Y -> +Z
}

TEST(AhrsImu, LeftHandedFlipsTheDerivedThirdAxis)
{
    AhrsImu imu;
    EXPECT_TRUE(imu.setAxisOrderLeftHanded(AhrsImu::axis_x_pos, AhrsImu::axis_y_pos));

    AxisOrder user;
    imu.readUserAxisOrder(user);
    EXPECT_EQ(user.permutation, AxisPermutation::XYZ);
    EXPECT_FALSE(user.invert_x);
    EXPECT_FALSE(user.invert_y);
    EXPECT_TRUE(user.invert_z);  // +X, +Y -> -Z
}

TEST(AhrsImu, RightHandedSwappedAxesYieldANegatedThirdAxis)
{
    AhrsImu imu;
    EXPECT_TRUE(imu.setAxisOrderRightHanded(AhrsImu::axis_y_pos, AhrsImu::axis_x_pos));

    AxisOrder user;
    imu.readUserAxisOrder(user);
    EXPECT_EQ(user.permutation, AxisPermutation::YXZ);
    EXPECT_TRUE(user.invert_z);  // +Y, +X -> -Z
}
