/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitIMU series with AhrsDemo
*/
// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_MINI_IMU_WITH_BMM350) && !defined(USING_BUILTIN_IMU) && \
    !defined(USING_UNIT_DOF6) && !defined(USING_UNIT_DOF9) && !defined(USING_UNIT_DOF10) &&                     \
    !defined(USING_UNIT_MINI_IMU_PRO)
// For UnitMiniIMU (U095) - 6-axis, GROVE
// #define USING_UNIT_MINI_IMU
// For UnitMiniIMU + UnitBMM350 (U231) - 9-axis on GROVE (0x68 + 0x14 coexist)
// #define USING_UNIT_MINI_IMU_WITH_BMM350
// For built-in IMU (Core2 / CoreS3 / AtomS3R / M5Stack Core/Fire)
// #define USING_BUILTIN_IMU
// For UnitDoF6 (U228) -- 6-axis BMI270
// #define USING_UNIT_DOF6
// For UnitDoF9 (U229) -- 9-axis BMI270 + BMM350
// #define USING_UNIT_DOF9
// For UnitDoF10 (U230) -- 10-axis BMI270 + BMM350 + SPA06 barometer
// #define USING_UNIT_DOF10
// For UnitMiniIMUPro (U171) -- 9-axis BMI270 + BMM150 (its BMP280 is out of scope, see M5Unit-ENV)
// #define USING_UNIT_MINI_IMU_PRO
#endif

#include "main/AhrsDemo.cpp"
