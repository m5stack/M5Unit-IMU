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

  @note **Known limitation**: AhrsController::writeAxisOrder() (Task 9) stores one AxisOrder
  per *source*, applied uniformly to every capability (accel/gyro/mag) that source provides
  (see ahrs_controller.cpp). M5Unified's original internal_axisorder_t is per *capability*
  instead (3 independent slots: sensor_index_accel/gyro/mag), because on some boards a
  combo chip's AUX magnetometer die is mounted at a different orientation than its own
  accel/gyro die. coreS3() and atomS3R() below both hit this: CoreS3 needs an
  accel/gyro-identity + mag-only correction, and AtomS3R needs two genuinely different
  corrections for accel/gyro vs. mag, from the *same* UnitBMI270 source. Neither is
  representable with a single AxisOrder, so:
  - coreS3() leaves the axis order at the default identity (correct for accel/gyro; mag Y/Z
    sign is left uncorrected).
  - atomS3R() applies the accel/gyro correction (needed for basic gravity-referenced
    roll/pitch); mag ends up with the same (wrong) permutation/inversion instead of its own,
    which mainly shows up as an incorrect magnetic yaw reference.
  Fixing this properly needs AhrsController to grow a second, capability-scoped axis order
  (e.g. a dedicated writeMagAxisOrder()) -- out of scope for this file; flagged here for a
  follow-up task.

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
  (accel/gyro are left as identity). See the file-level @note above: this function currently
  cannot apply that mag-only correction (AhrsController only has one AxisOrder per source),
  so it leaves the whole source at the identity transform -- accel/gyro come out correct,
  mag Y/Z sign is not corrected.
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
  two different corrections from the same chip. See the file-level @note above: this
  function applies the accel/gyro correction (AxisPermutation::YXZ, invert_y), which is what
  AhrsController::writeAxisOrder() stores per-source; mag ends up remapped the same way
  instead of its own invert_x|invert_z, which mainly shows up as an incorrect magnetic yaw
  reference rather than broken roll/pitch.
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

}  // namespace presets
}  // namespace imu
}  // namespace m5
#endif
