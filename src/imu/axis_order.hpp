/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file axis_order.hpp
  @brief Per-source axis permutation and inversion applied to raw-to-physical IMU samples
*/
#ifndef M5_UNIT_IMU_IMU_AXIS_ORDER_HPP
#define M5_UNIT_IMU_IMU_AXIS_ORDER_HPP

#include <cstdint>

namespace m5 {
namespace imu {

/*!
  @enum AxisPermutation
  @brief Which raw axis feeds each output slot, read left-to-right
  @details e.g. YXZ means output[0] = raw Y, output[1] = raw X, output[2] = raw Z.
 */
enum class AxisPermutation : uint8_t {
    XYZ,  //!< @brief Identity: output = (X, Y, Z)
    XZY,  //!< @brief output = (X, Z, Y)
    YXZ,  //!< @brief output = (Y, X, Z)
    YZX,  //!< @brief output = (Y, Z, X)
    ZXY,  //!< @brief output = (Z, X, Y)
    ZYX,  //!< @brief output = (Z, Y, X)
};

/*!
  @struct AxisOrder
  @brief Chip-frame-to-body-frame axis remap: a permutation plus a per-output-axis sign flip
  @details Applied by AhrsController::update() to every accel/gyro/mag sample from a given
  IMUDataSource, right after raw-to-physical conversion and before auto-calibration offsets
  are subtracted (see AhrsController::writeAxisOrder()/readAxisOrder()). Mirrors
  M5Unified's internal_axisorder_t, operating directly on physical-unit float triplets.

  The default (AxisPermutation::XYZ, no inversion) is the identity transform, i.e. exactly
  what a source already reports in its own chip frame.
 */
struct AxisOrder {
    AxisPermutation permutation{AxisPermutation::XYZ};  //!< @brief Axis permutation
    bool invert_x{false};                               //!< @brief Negate output X after permutation
    bool invert_y{false};                               //!< @brief Negate output Y after permutation
    bool invert_z{false};                               //!< @brief Negate output Z after permutation

    /*!
      @brief Apply this permutation and sign flip to a physical-unit XYZ triplet, in place
      @param v[in,out] 3-element array (accel, gyro, or mag), overwritten with the remapped values
     */
    inline void apply(float (&v)[3]) const
    {
        const float src[3] = {v[0], v[1], v[2]};
        switch (permutation) {
            case AxisPermutation::XYZ:
                v[0] = src[0];
                v[1] = src[1];
                v[2] = src[2];
                break;
            case AxisPermutation::XZY:
                v[0] = src[0];
                v[1] = src[2];
                v[2] = src[1];
                break;
            case AxisPermutation::YXZ:
                v[0] = src[1];
                v[1] = src[0];
                v[2] = src[2];
                break;
            case AxisPermutation::YZX:
                v[0] = src[1];
                v[1] = src[2];
                v[2] = src[0];
                break;
            case AxisPermutation::ZXY:
                v[0] = src[2];
                v[1] = src[0];
                v[2] = src[1];
                break;
            case AxisPermutation::ZYX:
                v[0] = src[2];
                v[1] = src[1];
                v[2] = src[0];
                break;
        }
        if (invert_x) {
            v[0] = -v[0];
        }
        if (invert_y) {
            v[1] = -v[1];
        }
        if (invert_z) {
            v[2] = -v[2];
        }
    }
};

/*!
  @brief Source-axis index feeding each output slot, for a given permutation
  @param p Permutation
  @return Pointer to a static 3-element array: out[i] = src[result[i]]
 */
inline const uint8_t* axis_permutation_indices(const AxisPermutation p)
{
    static const uint8_t table[6][3] = {
        {0, 1, 2},  // XYZ
        {0, 2, 1},  // XZY
        {1, 0, 2},  // YXZ
        {1, 2, 0},  // YZX
        {2, 0, 1},  // ZXY
        {2, 1, 0},  // ZYX
    };
    return table[static_cast<uint8_t>(p)];
}

/*!
  @brief Map a source-axis index triplet back to an AxisPermutation
  @param p 3-element array holding a permutation of {0,1,2} (out[i] = src[p[i]])
  @return The matching AxisPermutation; AxisPermutation::XYZ if p is not a valid permutation
 */
inline AxisPermutation permutation_from_indices(const uint8_t (&p)[3])
{
    const uint8_t code = static_cast<uint8_t>((p[0] << 4) | (p[1] << 2) | p[2]);
    switch (code) {
        case (0 << 4) | (1 << 2) | 2:
            return AxisPermutation::XYZ;
        case (0 << 4) | (2 << 2) | 1:
            return AxisPermutation::XZY;
        case (1 << 4) | (0 << 2) | 2:
            return AxisPermutation::YXZ;
        case (1 << 4) | (2 << 2) | 0:
            return AxisPermutation::YZX;
        case (2 << 4) | (0 << 2) | 1:
            return AxisPermutation::ZXY;
        case (2 << 4) | (1 << 2) | 0:
            return AxisPermutation::ZYX;
        default:
            return AxisPermutation::XYZ;
    }
}

/*!
  @brief Compose two axis orders into one
  @param first Axis order applied first
  @param second Axis order applied second
  @return An AxisOrder R such that R.apply(v) == second.apply(first.apply(v)) for every v
  @details AxisOrder is a signed axis permutation, so the composition is itself an AxisOrder
  (no information is lost). Used by AhrsController to fold a global user axis order onto each
  source's fixed board correction without applying two transforms per sample.
 */
inline AxisOrder compose(const AxisOrder& first, const AxisOrder& second)
{
    const uint8_t* pa   = axis_permutation_indices(first.permutation);
    const uint8_t* pb   = axis_permutation_indices(second.permutation);
    const bool inv_a[3] = {first.invert_x, first.invert_y, first.invert_z};
    const bool inv_b[3] = {second.invert_x, second.invert_y, second.invert_z};

    uint8_t pc[3]{};
    bool inv_c[3]{};
    for (int i = 0; i < 3; ++i) {
        pc[i]    = pa[pb[i]];
        inv_c[i] = inv_b[i] != inv_a[pb[i]];
    }
    AxisOrder result;
    result.permutation = permutation_from_indices(pc);
    result.invert_x    = inv_c[0];
    result.invert_y    = inv_c[1];
    result.invert_z    = inv_c[2];
    return result;
}

}  // namespace imu
}  // namespace m5
#endif
