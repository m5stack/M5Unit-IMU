/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <unit/unit_bmm150_data.hpp>

#include "bmm150_oracle.hpp"

using namespace m5::unit::bmm150;

namespace {

// Packs a signed 13bit value into the LSB/MSB register pair as the sensor
// would (value occupies bits[15:3] of the 16bit word), so that Data::x_raw()
// / y_raw() (which shift right by 3) recover it exactly.
void set_xy_raw(Data& d, const size_t lsb_idx, const size_t msb_idx, const int16_t raw13)
{
    const uint16_t word = static_cast<uint16_t>(static_cast<uint16_t>(raw13) << 3);
    d.raw[lsb_idx]      = static_cast<uint8_t>(word & 0xFF);
    d.raw[msb_idx]      = static_cast<uint8_t>((word >> 8) & 0xFF);
}

// Packs a signed 15bit value into DATA_Z_LSB/MSB (bits[15:1]).
void set_z_raw(Data& d, const int16_t raw15)
{
    const uint16_t word = static_cast<uint16_t>(static_cast<uint16_t>(raw15) << 1);
    d.raw[4]            = static_cast<uint8_t>(word & 0xFF);
    d.raw[5]            = static_cast<uint8_t>((word >> 8) & 0xFF);
}

// Packs an unsigned 14bit RHALL value into bits[15:2], with DRDY in bit0.
void set_rhall_raw(Data& d, const uint16_t rhall14, const bool drdy = false)
{
    const uint16_t word = static_cast<uint16_t>((rhall14 << 2) | (drdy ? 0x01 : 0x00));
    d.raw[6]            = static_cast<uint8_t>(word & 0xFF);
    d.raw[7]            = static_cast<uint8_t>((word >> 8) & 0xFF);
}

oracle::trim_registers to_oracle_trim(const trim_t& t)
{
    oracle::trim_registers ot{};
    ot.dig_x1   = t.dig_x1;
    ot.dig_y1   = t.dig_y1;
    ot.dig_x2   = t.dig_x2;
    ot.dig_y2   = t.dig_y2;
    ot.dig_z1   = t.dig_z1;
    ot.dig_z2   = t.dig_z2;
    ot.dig_z3   = t.dig_z3;
    ot.dig_z4   = t.dig_z4;
    ot.dig_xy1  = t.dig_xy1;
    ot.dig_xy2  = t.dig_xy2;
    ot.dig_xyz1 = t.dig_xyz1;
    return ot;
}

// Representative trim set taken from the plan (values observed on a real
// BMM150 unit).
trim_t make_trim_a()
{
    trim_t t{};
    t.dig_x1   = 0;
    t.dig_y1   = 0;
    t.dig_x2   = 26;
    t.dig_y2   = 26;
    t.dig_z1   = 24747;
    t.dig_z2   = 763;
    t.dig_z3   = 0;
    t.dig_z4   = 0;
    t.dig_xy1  = 29;
    t.dig_xy2  = -3;
    t.dig_xyz1 = 7053;
    return t;
}

// Same as above with every field perturbed by +1, to exercise the formulas
// with non-trivial values in every trim slot.
trim_t make_trim_b()
{
    trim_t t   = make_trim_a();
    t.dig_x1   = static_cast<int8_t>(t.dig_x1 + 1);
    t.dig_y1   = static_cast<int8_t>(t.dig_y1 + 1);
    t.dig_x2   = static_cast<int8_t>(t.dig_x2 + 1);
    t.dig_y2   = static_cast<int8_t>(t.dig_y2 + 1);
    t.dig_z1   = static_cast<uint16_t>(t.dig_z1 + 1);
    t.dig_z2   = static_cast<int16_t>(t.dig_z2 + 1);
    t.dig_z3   = static_cast<int16_t>(t.dig_z3 + 1);
    t.dig_z4   = static_cast<int16_t>(t.dig_z4 + 1);
    t.dig_xy1  = static_cast<uint8_t>(t.dig_xy1 + 1);
    t.dig_xy2  = static_cast<int8_t>(t.dig_xy2 + 1);
    t.dig_xyz1 = static_cast<uint16_t>(t.dig_xyz1 + 1);
    return t;
}

const int16_t XY_GRID[]     = {-4095, -2048, -1, 0, 1, 2047, 4095};
const int16_t Z_GRID[]      = {-16383, -8192, -1, 0, 1, 8191, 16383};
const uint16_t RHALL_GRID[] = {0, 3000, 6000, 13000};

}  // namespace

TEST(BMM150Data, BitUnpackXY)
{
    Data d{};
    // msb=0xFF, lsb=0xC0 -> word 0xFFC0 -> raw13 = -8 (negative case)
    d.raw[1] = 0xFF;
    d.raw[0] = 0xC0;
    EXPECT_EQ(d.x_raw(), -8);

    d.raw[3] = 0xFF;
    d.raw[2] = 0xC0;
    EXPECT_EQ(d.y_raw(), -8);

    // Positive round-trip via the packing helper too.
    set_xy_raw(d, 0, 1, 1234);
    EXPECT_EQ(d.x_raw(), 1234);
    set_xy_raw(d, 2, 3, -1234);
    EXPECT_EQ(d.y_raw(), -1234);
}

TEST(BMM150Data, BitUnpackZ)
{
    Data d{};
    // msb=0xFF, lsb=0xF0 -> word 0xFFF0 -> raw15 = -8 (negative case)
    d.raw[5] = 0xFF;
    d.raw[4] = 0xF0;
    EXPECT_EQ(d.z_raw(), -8);

    set_z_raw(d, 12345);
    EXPECT_EQ(d.z_raw(), 12345);
}

TEST(BMM150Data, BitUnpackRhallAndDrdy)
{
    Data d{};
    set_rhall_raw(d, 12345, true);
    EXPECT_EQ(d.rhall(), 12345);
    EXPECT_TRUE(d.drdy());

    Data d2{};
    set_rhall_raw(d2, 0, false);
    EXPECT_EQ(d2.rhall(), 0);
    EXPECT_FALSE(d2.drdy());
}

TEST(BMM150Data, AllAccessorsByteOffset)
{
    Data d{};
    set_xy_raw(d, 0, 1, 1000);
    set_xy_raw(d, 2, 3, -1000);
    set_z_raw(d, 5000);
    set_rhall_raw(d, 8000, true);

    EXPECT_EQ(d.x_raw(), 1000);
    EXPECT_EQ(d.y_raw(), -1000);
    EXPECT_EQ(d.z_raw(), 5000);
    EXPECT_EQ(d.rhall(), 8000);
    EXPECT_TRUE(d.drdy());
}

TEST(BMM150Data, OverflowXY)
{
    Data d{};
    d.trim = make_trim_a();

    set_xy_raw(d, 0, 1, -4096);
    set_rhall_raw(d, 6000);
    EXPECT_EQ(d.comp_x(), OVERFLOW_OUTPUT);

    Data d2{};
    d2.trim = make_trim_a();
    set_xy_raw(d2, 2, 3, -4096);
    set_rhall_raw(d2, 6000);
    EXPECT_EQ(d2.comp_y(), OVERFLOW_OUTPUT);
}

TEST(BMM150Data, OverflowZ)
{
    Data d{};
    d.trim = make_trim_a();
    set_z_raw(d, -16384);
    set_rhall_raw(d, 6000);
    EXPECT_EQ(d.comp_z(), OVERFLOW_OUTPUT);
}

TEST(BMM150Data, CompensateXMatchesOracle)
{
    const trim_t trims[] = {make_trim_a(), make_trim_b()};
    for (const trim_t& trim : trims) {
        const oracle::trim_registers ot = to_oracle_trim(trim);
        for (const int16_t raw : XY_GRID) {
            for (const uint16_t rh : RHALL_GRID) {
                Data d{};
                d.trim = trim;
                set_xy_raw(d, 0, 1, raw);
                set_rhall_raw(d, rh);
                EXPECT_EQ(d.comp_x(), oracle::compensate_x(raw, rh, ot)) << "raw=" << raw << " rhall=" << rh;
            }
        }
    }
}

TEST(BMM150Data, AdjScalesCompensatedTo16LsbPerMicroTesla)
{
    Data d{};
    d.trim = make_trim_a();
    set_xy_raw(d, 0, 1, 1000);
    set_z_raw(d, 5000);
    set_rhall_raw(d, 8000);
    // adj_* carries the compensated integer uT into the library-wide 16 LSB/uT raw scale
    EXPECT_EQ(d.adj_x(), static_cast<int32_t>(d.comp_x()) * 16);
    EXPECT_EQ(d.adj_y(), static_cast<int32_t>(d.comp_y()) * 16);
    EXPECT_EQ(d.adj_z(), static_cast<int32_t>(d.comp_z()) * 16);
}

TEST(BMM150Data, AdjPreservesOverflowSentinel)
{
    // The overflow sentinel must pass through unscaled, or is_mag_overflow() (which tests == -32768)
    // stops recognizing it once adj_* multiplies by 16.
    Data d{};
    d.trim = make_trim_a();
    set_xy_raw(d, 0, 1, -4096);
    set_rhall_raw(d, 6000);
    EXPECT_EQ(d.comp_x(), OVERFLOW_OUTPUT);
    EXPECT_EQ(d.adj_x(), static_cast<int32_t>(OVERFLOW_OUTPUT));

    Data dz{};
    dz.trim = make_trim_a();
    set_z_raw(dz, -16384);
    set_rhall_raw(dz, 6000);
    EXPECT_EQ(dz.comp_z(), OVERFLOW_OUTPUT);
    EXPECT_EQ(dz.adj_z(), static_cast<int32_t>(OVERFLOW_OUTPUT));
}

TEST(BMM150Data, CompensateYMatchesOracle)
{
    const trim_t trims[] = {make_trim_a(), make_trim_b()};
    for (const trim_t& trim : trims) {
        const oracle::trim_registers ot = to_oracle_trim(trim);
        for (const int16_t raw : XY_GRID) {
            for (const uint16_t rh : RHALL_GRID) {
                Data d{};
                d.trim = trim;
                set_xy_raw(d, 2, 3, raw);
                set_rhall_raw(d, rh);
                EXPECT_EQ(d.comp_y(), oracle::compensate_y(raw, rh, ot)) << "raw=" << raw << " rhall=" << rh;
            }
        }
    }
}

TEST(BMM150Data, CompensateZMatchesOracle)
{
    const trim_t trims[] = {make_trim_a(), make_trim_b()};
    for (const trim_t& trim : trims) {
        const oracle::trim_registers ot = to_oracle_trim(trim);
        for (const int16_t raw : Z_GRID) {
            for (const uint16_t rh : RHALL_GRID) {
                Data d{};
                d.trim = trim;
                set_z_raw(d, raw);
                set_rhall_raw(d, rh);
                EXPECT_EQ(d.comp_z(), oracle::compensate_z(raw, rh, ot)) << "raw=" << raw << " rhall=" << rh;
            }
        }
    }
}

TEST(BMM150Data, PresetRepValues)
{
    EXPECT_EQ(preset_rep_xy(Preset::LowPower), 0x01);
    EXPECT_EQ(preset_rep_z(Preset::LowPower), 0x02);
    EXPECT_EQ(preset_rep_xy(Preset::Regular), 0x04);
    EXPECT_EQ(preset_rep_z(Preset::Regular), 0x0E);
    EXPECT_EQ(preset_rep_xy(Preset::EnhancedRegular), 0x07);
    EXPECT_EQ(preset_rep_z(Preset::EnhancedRegular), 0x1A);
    EXPECT_EQ(preset_rep_xy(Preset::HighAccuracy), 0x17);
    EXPECT_EQ(preset_rep_z(Preset::HighAccuracy), 0x52);
}

TEST(BMM150Data, RepetitionCountsFromRegisterValues)
{
    // nXY = 1 + 2*REPXY, nZ = 1 + REPZ -- the presets have to land on the datasheet's Table 3 counts
    EXPECT_EQ(repetition_xy(preset_rep_xy(Preset::LowPower)), 3);
    EXPECT_EQ(repetition_z(preset_rep_z(Preset::LowPower)), 3);
    EXPECT_EQ(repetition_xy(preset_rep_xy(Preset::Regular)), 9);
    EXPECT_EQ(repetition_z(preset_rep_z(Preset::Regular)), 15);
    EXPECT_EQ(repetition_xy(preset_rep_xy(Preset::EnhancedRegular)), 15);
    EXPECT_EQ(repetition_z(preset_rep_z(Preset::EnhancedRegular)), 27);
    EXPECT_EQ(repetition_xy(preset_rep_xy(Preset::HighAccuracy)), 47);
    EXPECT_EQ(repetition_z(preset_rep_z(Preset::HighAccuracy)), 83);

    // Arbitrary values are legal too: only odd nXY is reachable, and the registers span 8 bits
    EXPECT_EQ(repetition_xy(0), 1);
    EXPECT_EQ(repetition_z(0), 1);
    EXPECT_EQ(repetition_xy(255), 511);
    EXPECT_EQ(repetition_z(255), 256);
}

TEST(BMM150Data, MeasurementTimeMatchesTheDatasheetMaxOdr)
{
    // Table 3's "max ODR in forced mode" column is 1/measurement_time; reproducing all four rows is
    // what pins the 145us/500us/980us relation this file uses
    struct {
        Preset preset;
        uint32_t max_odr_hz;
    } const expected[] = {
        {Preset::LowPower, 300},  // datasheet says ">300"
        {Preset::Regular, 100},
        {Preset::EnhancedRegular, 60},
        {Preset::HighAccuracy, 20},
    };
    for (const auto& e : expected) {
        const uint32_t us  = measurement_time_us(preset_rep_xy(e.preset), preset_rep_z(e.preset));
        const uint32_t odr = 1000000U / us;
        EXPECT_GE(odr, e.max_odr_hz) << "preset index " << static_cast<int>(e.preset);
        // ...and not wildly above it either (LowPower is the open-ended ">300" row)
        if (e.preset != Preset::LowPower) {
            EXPECT_LT(odr, e.max_odr_hz + e.max_odr_hz / 4) << "preset index " << static_cast<int>(e.preset);
        }
    }
}

TEST(BMM150Data, HighAccuracyDoesNotFitThirtyHertz)
{
    // The 30Hz default period is 33.3ms but HighAccuracy needs 49.3ms, so the measurement never
    // completes and the data registers stop updating -- repetition_fits() has to reject that pair
    EXPECT_FALSE(repetition_fits(DataRate::Rate30Hz, preset_rep_xy(Preset::HighAccuracy),
                                 preset_rep_z(Preset::HighAccuracy)));
    // 20Hz (50ms) is its documented ceiling and does fit
    EXPECT_TRUE(repetition_fits(DataRate::Rate20Hz, preset_rep_xy(Preset::HighAccuracy),
                                preset_rep_z(Preset::HighAccuracy)));
    // Every other preset keeps up with 30Hz
    EXPECT_TRUE(repetition_fits(DataRate::Rate30Hz, preset_rep_xy(Preset::Regular), preset_rep_z(Preset::Regular)));
    EXPECT_TRUE(repetition_fits(DataRate::Rate30Hz, preset_rep_xy(Preset::EnhancedRegular),
                                preset_rep_z(Preset::EnhancedRegular)));
    EXPECT_TRUE(repetition_fits(DataRate::Rate30Hz, preset_rep_xy(Preset::LowPower), preset_rep_z(Preset::LowPower)));
    // The slowest rate has room for anything, the maximum repetitions for nothing faster
    EXPECT_TRUE(repetition_fits(DataRate::Rate2Hz, 255, 255));
    EXPECT_FALSE(repetition_fits(DataRate::Rate10Hz, 255, 255));
}
