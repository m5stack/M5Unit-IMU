/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/imu_detect.hpp>
#include <map>

using namespace m5::unit::imu;

namespace {
// A fake I2C bus: (addr,reg)->value. Missing addr => NAK (read returns false). An addr that is
// present but whose reg is unmapped still ACKs and reads 0x00 (mirrors a real device answering an
// undefined register), so a probe can tell "device present" from "no device".
struct FakeBus {
    std::map<std::pair<uint8_t, uint8_t>, uint8_t> mem;

    bool read(const uint8_t addr, const uint8_t reg, uint8_t& out)
    {
        auto it = mem.find({addr, reg});
        if (it == mem.end()) {
            for (const auto& kv : mem) {
                if (kv.first.first == addr) {
                    out = 0x00;
                    return true;
                }
            }
            return false;  // address absent -> NAK
        }
        out = it->second;
        return true;
    }
};
}  // namespace

TEST(ImuDetect, Mpu6886ByWhoAmI)
{
    FakeBus bus;
    bus.mem[{0x68, 0x75}] = 0x19;  // MPU6886 WHO_AM_I
    EXPECT_EQ(detectBuiltinImu([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              ImuKind::MPU6886);
}

TEST(ImuDetect, Mpu6050ByWhoAmI)
{
    FakeBus bus;
    bus.mem[{0x68, 0x75}] = 0x68;  // MPU6050 WHO_AM_I
    EXPECT_EQ(detectBuiltinImu([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              ImuKind::MPU6050);
}

TEST(ImuDetect, Mpu9250ByWhoAmI)
{
    FakeBus bus;
    bus.mem[{0x68, 0x75}] = 0x71;  // MPU9250 WHO_AM_I
    EXPECT_EQ(detectBuiltinImu([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              ImuKind::MPU9250);
}

TEST(ImuDetect, Bmi270At0x68)
{
    FakeBus bus;
    bus.mem[{0x68, 0x00}] = 0x24;  // BMI270 CHIP_ID; its 0x75 is unmapped -> reads 0x00 (not an MPU id)
    EXPECT_EQ(detectBuiltinImu([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              ImuKind::BMI270);
}

TEST(ImuDetect, Bmi270At0x69)
{
    FakeBus bus;
    bus.mem[{0x69, 0x00}] = 0x24;  // BMI270 at the alternate address (CoreS3 etc.)
    EXPECT_EQ(detectBuiltinImu([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              ImuKind::BMI270);
}

TEST(ImuDetect, Sh200qByWhoAmI)
{
    FakeBus bus;
    bus.mem[{0x6C, 0x30}] = 0x18;  // SH200Q WHO_AM_I at 0x6C
    EXPECT_EQ(detectBuiltinImu([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              ImuKind::SH200Q);
}

TEST(ImuDetect, NoneWhenBusEmpty)
{
    FakeBus bus;
    EXPECT_EQ(detectBuiltinImu([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              ImuKind::None);
}

TEST(ImuDetect, MpuWinsWhenBmi270ChipIdAlsoAliasesAt0x68)
{
    // A real MPU answers WHO_AM_I at 0x75; the MPU family must be decided before the BMI270 CHIP_ID
    // probe so an MPU is never mistaken for a BMI270.
    FakeBus bus;
    bus.mem[{0x68, 0x75}] = 0x19;  // MPU6886
    bus.mem[{0x68, 0x00}] = 0x24;  // coincidental 0x24 at reg 0x00 must not override the MPU verdict
    EXPECT_EQ(detectBuiltinImu([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              ImuKind::MPU6886);
}
