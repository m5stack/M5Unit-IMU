/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file presets.cpp
  @brief Board-specific AhrsController setup for M5Stack products with a built-in IMU
*/
#include "presets.hpp"

namespace m5 {
namespace imu {
namespace presets {

bool core2(AhrsController& ahrs, m5::unit::UnitMPU6886& mpu6886)
{
    // See presets.hpp: board_M5StackCore2 has no axis correction in M5Unified, identity
    // (AhrsController::add()'s default) is already correct.
    return ahrs.add(mpu6886);
}

bool coreS3(AhrsController& ahrs, m5::unit::UnitBMI270& bmi270)
{
    if (!ahrs.add(bmi270)) {
        return false;
    }
    // accel/gyro stay at the default identity; only the AUX BMM150's Y/Z axes are inverted.
    AxisOrder mag_order;
    mag_order.invert_y = true;
    mag_order.invert_z = true;
    ahrs.writeMagAxisOrder(bmi270, mag_order);
    return true;
}

bool atomS3R(AhrsController& ahrs, m5::unit::UnitBMI270& bmi270)
{
    if (!ahrs.add(bmi270)) {
        return false;
    }
    // Accel/gyro correction.
    AxisOrder order;
    order.permutation = AxisPermutation::YXZ;
    order.invert_y    = true;
    ahrs.writeAxisOrder(bmi270, order);

    // Independent AUX BMM150 (mag) correction.
    AxisOrder mag_order;
    mag_order.invert_x = true;
    mag_order.invert_z = true;
    ahrs.writeMagAxisOrder(bmi270, mag_order);
    return true;
}

bool m5stackCoreFire(AhrsController& ahrs, m5::unit::UnitMPU6886& mpu6886, m5::unit::UnitBMM150& bmm150)
{
    const bool added_mpu6886 = ahrs.add(mpu6886);
    const bool added_bmm150  = ahrs.add(bmm150);
    if (!added_mpu6886 || !added_bmm150) {
        return false;
    }
    // BMM150 is a pure-Mag source; the mag remap path uses the mag axis order (see
    // ahrs_controller.cpp), so the X/Z inversion must be written via writeMagAxisOrder.
    AxisOrder mag_order;
    mag_order.invert_x = true;
    mag_order.invert_z = true;
    ahrs.writeMagAxisOrder(bmm150, mag_order);
    return true;
}

bool m5stackCoreFire(AhrsController& ahrs, m5::unit::UnitSH200Q& sh200q, m5::unit::UnitBMM150& bmm150)
{
    const bool added_sh200q = ahrs.add(sh200q);
    const bool added_bmm150 = ahrs.add(bmm150);
    if (!added_sh200q || !added_bmm150) {
        return false;
    }
    // The BMM150 mounting (and thus its X/Z inversion) is a board property of board_M5Stack,
    // identical to the MPU6886 generation. The SH200Q accel/gyro stay at identity until the
    // 2019.7 orientation is confirmed on hardware (see presets.hpp).
    AxisOrder mag_order;
    mag_order.invert_x = true;
    mag_order.invert_z = true;
    ahrs.writeMagAxisOrder(bmm150, mag_order);
    return true;
}

bool atomMatrix(AhrsController& ahrs, m5::unit::UnitMPU6886& mpu6886)
{
    if (!ahrs.add(mpu6886)) {
        return false;
    }
    AxisOrder order;
    order.invert_x = true;
    order.invert_z = true;
    ahrs.writeAxisOrder(mpu6886, order);
    return true;
}

bool atomMatrix(AhrsController& ahrs, m5::unit::UnitBMI270& bmi270)
{
    if (!ahrs.add(bmi270)) {
        return false;
    }
    // ATOM Matrix v1.1 swapped the MPU6886 for a BMI270 but keeps the same mounting orientation, so
    // M5Unified applies the identical accel/gyro X/Z inversion (IMU_Class.cpp). The BMI270 here is a
    // 6-axis part (no AUX magnetometer), so only the accel/gyro axis order is written.
    AxisOrder order;
    order.invert_x = true;
    order.invert_z = true;
    ahrs.writeAxisOrder(bmi270, order);
    return true;
}

bool mpu9250Ak8963(AhrsController& ahrs, m5::unit::UnitAK8963& ak8963)
{
    if (!ahrs.add(ak8963)) {
        return false;
    }
    AxisOrder mag_order;
    mag_order.permutation = AxisPermutation::YXZ;
    mag_order.invert_z    = true;
    ahrs.writeMagAxisOrder(ak8963, mag_order);
    return true;
}

}  // namespace presets
}  // namespace imu
}  // namespace m5
