/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/imu_data_source.hpp>
#include <vector>

using namespace m5::imu;

namespace {
// Pop semantics のリファレンス実装を兼ねるモック
class MockSource : public IMUDataSource {
public:
    Capability imuSpec() const override
    {
        return Capability::Accel | Capability::Gyro;
    }
    Capability readImuRaw(imu_raw_data_t& out) override
    {
        if (_pos >= _samples.size()) {
            return Capability::None;
        }
        out = _samples[_pos++];
        return imuSpec();
    }
    void imuConvertParam(imu_convert_param_t& out) const override
    {
        out = imu_convert_param_t{};
    }
    void push(const int16_t ax)
    {
        imu_raw_data_t d{};
        d.accel[0] = ax;
        _samples.push_back(d);
    }

private:
    std::vector<imu_raw_data_t> _samples{};
    size_t _pos{};
};
}  // namespace

TEST(IMUDataSource, PopSemanticsDrainsAllThenNone)
{
    MockSource src;
    src.push(100);
    src.push(200);

    imu_raw_data_t d{};
    int count{};
    while (any(src.readImuRaw(d))) {
        ++count;
    }
    EXPECT_EQ(count, 2);
    EXPECT_EQ(d.accel[0], 200);
    EXPECT_FALSE(any(src.readImuRaw(d)));  // 消費後は None
}
