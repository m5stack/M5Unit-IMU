/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file presets.hpp
  @brief Board-specific AhrsController setup for M5Stack products with a built-in IMU
  @details Each function below only registers the given unit(s) with an AhrsController and
  applies the axis remap M5Unified's IMU_Class::begin() hard-codes for that board (see
  research/m5unified-port.md §軸リマップ, ported from M5Unified's internal_axisorder_t /
  IMU_Class.cpp board switch). None of these functions call unit.begin() themselves -- the
  caller must have already begun the unit(s) (I2C bus ready) -- and none of them decide which
  board is running; callers pick which preset to call (e.g. via M5.getBoard()).

  @note Not covered by native tests: everything these functions touch (AhrsController /
  AxisOrder) already has native coverage from Task 9. The unit types used here
  (UnitMPU6886/UnitBMI270/UnitBMM150) pull in M5UnitUnified + Arduino/ESP-IDF headers and
  cannot build under test_native (see platformio.ini's test_native build_src_filter, which
  excludes this file); presets.cpp is instead verified by compiling under an embedded env
  (e.g. test_MPU6886_Core2).
*/
#ifndef M5_UNIT_IMU_IMU_PRESETS_HPP
#define M5_UNIT_IMU_IMU_PRESETS_HPP

#include "../unit/unit_bmi270.hpp"
#include "../unit/unit_bmm150.hpp"
#include "../unit/unit_mpu6886.hpp"
#include "../unit/unit_ak8963.hpp"
#include "../unit/unit_sh200q.hpp"
#include "ahrs_controller.hpp"

namespace m5 {
namespace imu {
namespace presets {

/*!
  @brief M5Stack Core2's built-in MPU6886
  @param ahrs Controller to register mpu6886 with
  @param mpu6886 Core2's built-in MPU6886 (must already be begin()'d)
  @return False if ahrs already holds the maximum number of sources (see AhrsController::add())
  @details M5Unified's IMU_Class::begin() has no case for board_M5StackCore2 in its
  axis-order switch (IMU_Class.cpp), i.e. the identity transform is already correct here.
  AhrsController::add() defaults every newly added source to the identity AxisOrder, so this
  function does not call writeAxisOrder().
 */
bool core2(AhrsController& ahrs, m5::unit::UnitMPU6886& mpu6886);

/*!
  @brief M5Stack CoreS3/CoreS3 SE/StackChan's built-in BMI270 (with AUX BMM150 magnetometer)
  @param ahrs Controller to register bmi270 with
  @param bmi270 CoreS3's built-in BMI270 at address 0x69, with the AUX BMM150 probed and
  enabled (config_t::enable_aux_bmm150, the default -- see UnitBMI270::begin())
  @return False if ahrs already holds the maximum number of sources (see AhrsController::add())
  @details M5Unified's IMU_Class::begin(): for board_M5StackCoreS3 / board_M5StackChan with
  the BMI270 at address 0x69, only the magnetometer's Y and Z axes are inverted
  (accel/gyro are left as identity). accel/gyro keep the default identity AxisOrder; only
  the AUX BMM150's Mag samples get the Y/Z inversion, via
  AhrsController::writeMagAxisOrder().
 */
bool coreS3(AhrsController& ahrs, m5::unit::UnitBMI270& bmi270);

/*!
  @brief AtomS3R/AtomS3R CAM/AtomS3R Ext's built-in BMI270 (with AUX BMM150 magnetometer)
  @param ahrs Controller to register bmi270 with
  @param bmi270 AtomS3R's built-in BMI270, with the AUX BMM150 probed and enabled
  (config_t::enable_aux_bmm150, the default -- see UnitBMI270::begin())
  @return False if ahrs already holds the maximum number of sources (see AhrsController::add())
  @details M5Unified's IMU_Class::begin(): for board_M5AtomS3R / board_M5AtomS3RCam /
  board_M5AtomS3RExt, accel/gyro get X/Y swapped plus Y inverted (axis_order_yxz |
  axis_invert_y), while mag gets X and Z inverted instead (axis_invert_x | axis_invert_z) --
  two different corrections from the same chip. This function applies the accel/gyro
  correction (AxisPermutation::YXZ, invert_y) via AhrsController::writeAxisOrder(), and the
  independent mag correction (invert_x | invert_z) via
  AhrsController::writeMagAxisOrder().
 */
bool atomS3R(AhrsController& ahrs, m5::unit::UnitBMI270& bmi270);

/*!
  @brief Classic M5Stack Core/Fire's built-in MPU6886 + discrete BMM150 magnetometer
  @param ahrs Controller to register mpu6886 and bmm150 with
  @param mpu6886 The board's built-in MPU6886 (must already be begin()'d)
  @param bmm150 The board's built-in, separately-wired BMM150 (must already be begin()'d)
  @return False if either add() call fails (see AhrsController::add())
  @details M5Unified's IMU_Class::begin(): for board_M5Stack with an MPU6886 + BMM150
  combination, the magnetometer's X and Z axes are inverted. mpu6886 and bmm150 are two
  independent IMUDataSource instances here, so this maps cleanly onto
  AhrsController::writeAxisOrder() -- mpu6886 keeps the identity transform, only bmm150 gets
  the correction.
 */
bool m5stackCoreFire(AhrsController& ahrs, m5::unit::UnitMPU6886& mpu6886, m5::unit::UnitBMM150& bmm150);

/*!
  @brief Fire 2019.7's built-in SH200Q + discrete BMM150 magnetometer
  @param ahrs Controller to register sh200q and bmm150 with
  @param sh200q The board's built-in SH200Q (must already be begin()'d)
  @param bmm150 The board's built-in, separately-wired BMM150 (must already be begin()'d)
  @return False if either add() call fails (see AhrsController::add())
  @details The BMM150's X/Z inversion is a board-mounting property of board_M5Stack (M5Unified
  gates it on the board, not on the accel chip), so the same mag correction as the MPU6886
  generation applies. The SH200Q accel/gyro stay at identity: no M5Unified reference table exists
  for it and field reports suggest its accel and gyro axis systems may disagree -- confirm on 2019.7
  hardware before committing a correction (see docs/imu_orientation).
 */
bool m5stackCoreFire(AhrsController& ahrs, m5::unit::UnitSH200Q& sh200q, m5::unit::UnitBMM150& bmm150);

/*!
  @brief AtomMatrix's built-in MPU6886
  @param ahrs Controller to register mpu6886 with
  @param mpu6886 AtomMatrix's built-in MPU6886 (must already be begin()'d)
  @return False if ahrs already holds the maximum number of sources (see AhrsController::add())
  @details M5Unified's IMU_Class::begin(): for board_M5AtomMatrix, accel and gyro both get
  X and Z inverted ("IMU is oriented differently"). Both capabilities come from the same
  MPU6886 source and need the identical correction, so this maps cleanly onto
  AhrsController::writeAxisOrder() (one AxisOrder for the whole source).
 */
bool atomMatrix(AhrsController& ahrs, m5::unit::UnitMPU6886& mpu6886);

/*!
  @brief ATOM Matrix v1.1's built-in BMI270 (the MPU6886 was replaced by a BMI270 at v1.1)
  @param ahrs Controller to register bmi270 with
  @param bmi270 ATOM Matrix v1.1's built-in BMI270 (6-axis, no AUX magnetometer; must already be
  begin()'d)
  @return False if ahrs already holds the maximum number of sources (see AhrsController::add())
  @details M5Unified's IMU_Class::begin() gives the AtomMatrix BMI270 the same accel/gyro X/Z
  inversion as the original MPU6886 AtomMatrix (the board orientation is unchanged across the chip
  swap). Both capabilities come from the same source and need the identical correction, so this maps
  onto a single AhrsController::writeAxisOrder().
 */
bool atomMatrix(AhrsController& ahrs, m5::unit::UnitBMI270& bmi270);

/*!
  @brief Original (MPU9250-based) classic M5Stack's built-in AK8963 magnetometer
  @param ahrs Controller to register ak8963 with
  @param ak8963 The MPU9250-internal AK8963 (must already be begin()'d; the MPU9250 bypass must
  already be open -- see m5::unit::imu::enableAuxBypass)
  @return False if ahrs already holds the maximum number of sources (see AhrsController::add())
  @details M5Unified's IMU_Class::begin(): for an MPU9250-internal AK8963 (gated on the accel/gyro
  being an MPU9250, not on board id), the magnetometer's X/Y axes are swapped and Z inverted
  (axis_order_yxz | axis_invert_z). AK8963 is a pure-Mag source, so the correction is applied via
  AhrsController::writeMagAxisOrder() (the mag remap path).
 */
bool mpu9250Ak8963(AhrsController& ahrs, m5::unit::UnitAK8963& ak8963);

}  // namespace presets
}  // namespace imu
}  // namespace m5
#endif
