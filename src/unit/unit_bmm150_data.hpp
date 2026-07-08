/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_bmm150_data.hpp
  @brief BMM150 setting enums and measurement data (framework independent)
*/
#ifndef M5_UNIT_IMU_UNIT_UNIT_BMM150_DATA_HPP
#define M5_UNIT_IMU_UNIT_UNIT_BMM150_DATA_HPP

#include <array>
#include <cstdint>

namespace m5 {
namespace unit {
namespace bmm150 {

/*!
  @enum DataRate
  @brief Output data rate (REG_OP_MODE register bits 5:3, value order is NOT
  monotonic in Hz)
 */
enum class DataRate : uint8_t {
    Rate10Hz = 0x00,  //!< @brief 10Hz (power-on default)
    Rate2Hz  = 0x01,  //!< @brief 2Hz
    Rate6Hz  = 0x02,  //!< @brief 6Hz
    Rate8Hz  = 0x03,  //!< @brief 8Hz
    Rate15Hz = 0x04,  //!< @brief 15Hz
    Rate20Hz = 0x05,  //!< @brief 20Hz
    Rate25Hz = 0x06,  //!< @brief 25Hz
    Rate30Hz = 0x07,  //!< @brief 30Hz
};

/*!
  @enum Preset
  @brief Preset accuracy/power modes, controlling the REG_REP_XY/REG_REP_Z repetition
  counts (see preset_rep_xy()/preset_rep_z())
 */
enum class Preset : uint8_t {
    LowPower,         //!< @brief Lowest power, lowest accuracy
    Regular,          //!< @brief Balanced accuracy/power (recommended default)
    EnhancedRegular,  //!< @brief Enhanced accuracy over Regular
    HighAccuracy,     //!< @brief Highest accuracy, highest power
};

//! @brief REG_REP_XY register value (0x51) for the given preset
constexpr uint8_t preset_rep_xy(const Preset p)
{
    return (p == Preset::LowPower)          ? 0x01
           : (p == Preset::Regular)         ? 0x04
           : (p == Preset::EnhancedRegular) ? 0x07
                                            : 0x17;
}

//! @brief REG_REP_Z register value (0x52) for the given preset
constexpr uint8_t preset_rep_z(const Preset p)
{
    return (p == Preset::LowPower)          ? 0x02
           : (p == Preset::Regular)         ? 0x0E
           : (p == Preset::EnhancedRegular) ? 0x1A
                                            : 0x52;
}

//! @brief Sentinel returned by comp_x()/comp_y()/comp_z() on overflow or invalid input
constexpr int16_t OVERFLOW_OUTPUT{-32768};

namespace internal {
constexpr int16_t XY_OVERFLOW_RAW{-4096};
constexpr int16_t Z_OVERFLOW_RAW{-16384};
constexpr int32_t Z_POSITIVE_SATURATION{32767};
constexpr int32_t Z_NEGATIVE_SATURATION{-32767};
}  // namespace internal

/*!
  @struct trim_t
  @brief Factory trim (calibration) values read from registers 0x5D-0x71,
  consumed by the compensation formulas in Data::comp_x()/comp_y()/comp_z()
 */
struct trim_t {
    int8_t dig_x1{0};      //!< @brief 0x5D
    int8_t dig_y1{0};      //!< @brief 0x5E
    int8_t dig_x2{0};      //!< @brief 0x64
    int8_t dig_y2{0};      //!< @brief 0x65
    uint16_t dig_z1{0};    //!< @brief 0x6A/0x6B
    int16_t dig_z2{0};     //!< @brief 0x68/0x69
    int16_t dig_z3{0};     //!< @brief 0x6E/0x6F
    int16_t dig_z4{0};     //!< @brief 0x62/0x63
    uint8_t dig_xy1{0};    //!< @brief 0x71
    int8_t dig_xy2{0};     //!< @brief 0x70
    uint16_t dig_xyz1{0};  //!< @brief 0x6C/0x6D (14bit)
};

/*!
  @struct Data
  @brief One measurement (8 bytes burst from REG_DATA_X_LSB, register 0x42)
 */
struct Data {
    std::array<uint8_t, 8> raw{};  //!< @brief [0..1] X, [2..3] Y, [4..5] Z, [6..7] RHALL (bit0 of [6] = DRDY)
    trim_t trim{};                 //!< @brief Trim values used by the compensation formulas

    ///@name Raw ADC values
    ///@{
    //! @brief Raw magnetometer X (13bit signed, register bits[15:3])
    inline int16_t x_raw() const
    {
        const int16_t v = static_cast<int16_t>((raw[1] << 8) | raw[0]);
        return static_cast<int16_t>(v >> 3);
    }
    //! @brief Raw magnetometer Y (13bit signed, register bits[15:3])
    inline int16_t y_raw() const
    {
        const int16_t v = static_cast<int16_t>((raw[3] << 8) | raw[2]);
        return static_cast<int16_t>(v >> 3);
    }
    //! @brief Raw magnetometer Z (15bit signed, register bits[15:1])
    inline int16_t z_raw() const
    {
        const int16_t v = static_cast<int16_t>((raw[5] << 8) | raw[4]);
        return static_cast<int16_t>(v >> 1);
    }
    //! @brief Raw hall resistance (14bit unsigned, register bits[15:2])
    inline uint16_t rhall() const
    {
        const uint16_t v = static_cast<uint16_t>((raw[7] << 8) | raw[6]);
        return static_cast<uint16_t>(v >> 2);
    }
    //! @brief Data ready flag (bit0 of register 0x48)
    inline bool drdy() const
    {
        return (raw[6] & 0x01) != 0;
    }
    ///@}

    ///@name Compensated values (Bosch trim compensation, 16 LSB/uT)
    ///@{
    //! @brief Compensated magnetometer X (OVERFLOW_OUTPUT on overflow)
    inline int16_t comp_x() const
    {
        const int16_t data_x = x_raw();
        if (data_x == internal::XY_OVERFLOW_RAW) {
            return OVERFLOW_OUTPUT;
        }
        uint16_t base = rhall();
        if (base == 0) {
            base = trim.dig_xyz1;
        }
        if (base == 0) {
            return OVERFLOW_OUTPUT;
        }

        const int32_t xyz1_scaled = static_cast<int32_t>(trim.dig_xyz1) * 16384;
        const uint16_t normalized = static_cast<uint16_t>(xyz1_scaled / base) - static_cast<uint16_t>(0x4000);
        const int16_t centered    = static_cast<int16_t>(normalized);
        const int32_t squared     = static_cast<int32_t>(centered) * static_cast<int32_t>(centered);
        const int32_t cross_term  = static_cast<int32_t>(trim.dig_xy2) * (squared / 128);
        const int32_t xy1_scaled  = static_cast<int32_t>(static_cast<int16_t>(trim.dig_xy1) * 128);
        const int32_t linear_term = static_cast<int32_t>(centered) * xy1_scaled;
        const int32_t combined    = ((cross_term + linear_term) / 512) + static_cast<int32_t>(0x100000);
        const int32_t gain    = static_cast<int32_t>(static_cast<int16_t>(trim.dig_x2) + static_cast<int16_t>(0xA0));
        const int32_t scale   = (combined * gain) / 4096;
        const int32_t applied = static_cast<int32_t>(data_x) * scale;

        int16_t result = static_cast<int16_t>(applied / 8192);
        result         = static_cast<int16_t>((result + static_cast<int16_t>(trim.dig_x1) * 8) / 16);
        return result;
    }
    //! @brief Compensated magnetometer Y (OVERFLOW_OUTPUT on overflow)
    inline int16_t comp_y() const
    {
        const int16_t data_y = y_raw();
        if (data_y == internal::XY_OVERFLOW_RAW) {
            return OVERFLOW_OUTPUT;
        }
        uint16_t base = rhall();
        if (base == 0) {
            base = trim.dig_xyz1;
        }
        if (base == 0) {
            return OVERFLOW_OUTPUT;
        }

        const int32_t normalized_full = (static_cast<int32_t>(trim.dig_xyz1) * 16384) / base;
        const uint16_t normalized     = static_cast<uint16_t>(normalized_full) - static_cast<uint16_t>(0x4000);
        const int16_t centered        = static_cast<int16_t>(normalized);
        const int32_t squared         = static_cast<int32_t>(centered) * static_cast<int32_t>(centered);
        const int32_t cross_term      = static_cast<int32_t>(trim.dig_xy2) * (squared / 128);
        const int32_t xy1_scaled      = static_cast<int32_t>(static_cast<int16_t>(trim.dig_xy1) * 128);
        const int32_t combined_num    = cross_term + static_cast<int32_t>(centered) * xy1_scaled;
        const int32_t combined        = combined_num / 512;
        const int32_t gain    = static_cast<int32_t>(static_cast<int16_t>(trim.dig_y2) + static_cast<int16_t>(0xA0));
        const int32_t scale   = ((combined + static_cast<int32_t>(0x100000)) * gain) / 4096;
        const int32_t applied = static_cast<int32_t>(data_y) * scale;

        int16_t result = static_cast<int16_t>(applied / 8192);
        result         = static_cast<int16_t>((result + static_cast<int16_t>(trim.dig_y1) * 8) / 16);
        return result;
    }
    //! @brief Compensated magnetometer Z (OVERFLOW_OUTPUT on overflow), saturated to +-32767 before scaling
    inline int16_t comp_z() const
    {
        const int16_t data_z = z_raw();
        if (data_z == internal::Z_OVERFLOW_RAW) {
            return OVERFLOW_OUTPUT;
        }
        const uint16_t rh = rhall();
        if (trim.dig_z2 == 0 || trim.dig_z1 == 0 || rh == 0 || trim.dig_xyz1 == 0) {
            return OVERFLOW_OUTPUT;
        }

        const int16_t hall_offset =
            static_cast<int16_t>(static_cast<int16_t>(rh) - static_cast<int16_t>(trim.dig_xyz1));
        const int32_t z3_term    = (static_cast<int32_t>(trim.dig_z3) * static_cast<int32_t>(hall_offset)) / 4;
        const int32_t z_delta    = static_cast<int32_t>(data_z - trim.dig_z4) * 32768;
        const int32_t z1_raw     = static_cast<int32_t>(trim.dig_z1) * (static_cast<int16_t>(rh) * 2);
        const int16_t z1_rounded = static_cast<int16_t>((z1_raw + 32768) / 65536);

        int32_t result = (z_delta - z3_term) / (trim.dig_z2 + z1_rounded);

        if (result > internal::Z_POSITIVE_SATURATION) {
            result = internal::Z_POSITIVE_SATURATION;
        } else if (result < internal::Z_NEGATIVE_SATURATION) {
            result = internal::Z_NEGATIVE_SATURATION;
        }
        result /= 16;

        return static_cast<int16_t>(result);
    }
    ///@}
};

}  // namespace bmm150
}  // namespace unit
}  // namespace m5
#endif
