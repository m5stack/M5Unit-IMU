/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <imu/mag_detect.hpp>
#include <map>

using namespace m5::unit::imu;

namespace {
// A fake I2C bus: (addr,reg)->value. Missing addr => NAK (read returns false).
struct FakeBus {
    std::map<std::pair<uint8_t, uint8_t>, uint8_t> mem;
    std::map<std::pair<uint8_t, uint8_t>, uint8_t> writes;

    bool read(const uint8_t addr, const uint8_t reg, uint8_t& out)
    {
        auto it = mem.find({addr, reg});
        if (it == mem.end()) {
            // Address present but register unknown still ACKs if the address exists at all.
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
    bool write(const uint8_t addr, const uint8_t reg, const uint8_t val)
    {
        writes[{addr, reg}] = val;
        return true;
    }
};
}  // namespace

TEST(MagDetect, FindsBmm150ByAddressAck)
{
    FakeBus bus;
    bus.mem[{0x10, 0x40}] = 0x32;  // BMM150 present
    EXPECT_EQ(detectBuiltinMag([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              MagKind::BMM150);
}

TEST(MagDetect, FindsBmm150EvenWhenChipIdReadsZeroSuspended)
{
    FakeBus bus;
    bus.mem[{0x10, 0x4B}] = 0x00;  // device present at 0x10 but CHIP_ID(0x40) not in map -> reads 0x00 with ACK
    EXPECT_EQ(detectBuiltinMag([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              MagKind::BMM150);
}

TEST(MagDetect, FindsAk8963ByWhoAmI)
{
    FakeBus bus;
    bus.mem[{0x0C, 0x00}] = 0x48;  // AK8963 present
    EXPECT_EQ(detectBuiltinMag([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              MagKind::AK8963);
}

TEST(MagDetect, NoneWhenBusEmpty)
{
    FakeBus bus;
    EXPECT_EQ(detectBuiltinMag([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); }),
              MagKind::None);
}

TEST(MagDetect, EnableAuxBypassNoOpWithoutMpu9250)
{
    FakeBus bus;
    bus.mem[{0x68, 0x75}] = 0x19;  // MPU6886, not MPU9250
    const bool enabled = enableAuxBypass([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); },
                                         [&](uint8_t a, uint8_t r, uint8_t v) { return bus.write(a, r, v); });
    EXPECT_FALSE(enabled);
    EXPECT_TRUE(bus.writes.empty());  // must not touch a non-MPU9250 host's registers
}

TEST(MagDetect, EnableAuxBypassSetsBypassOnMpu9250)
{
    FakeBus bus;
    bus.mem[{0x68, 0x75}] = 0x71;  // MPU9250
    bus.mem[{0x68, 0x6A}] = 0x20;  // USER_CTRL with I2C_MST_EN set
    const bool enabled = enableAuxBypass([&](uint8_t a, uint8_t r, uint8_t& o) { return bus.read(a, r, o); },
                                         [&](uint8_t a, uint8_t r, uint8_t v) { return bus.write(a, r, v); });
    EXPECT_TRUE(enabled);
    EXPECT_EQ((bus.writes[{0x68, 0x6A}]), 0x00);  // I2C_MST_EN cleared
    EXPECT_EQ((bus.writes[{0x68, 0x37}]), 0x02);  // BYPASS_EN set
}
