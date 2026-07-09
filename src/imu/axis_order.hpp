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
  M5Unified's internal_axisorder_t, reworked to operate directly on physical-unit float
  triplets instead of a packed 3-bit-per-axis raw index.

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

}  // namespace imu
}  // namespace m5
#endif
