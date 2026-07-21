/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitIMU series with PlotToSerial
*/
// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_BMM350) && !defined(USING_UNIT_DOF6) && \
    !defined(USING_UNIT_DOF9) && !defined(USING_UNIT_DOF10) && !defined(USING_UNIT_MINI_IMU_PRO)
// For UnitMiniIMU (U095)
// #define USING_UNIT_MINI_IMU
// For UnitBMM350 (U231)
// #define USING_UNIT_BMM350
// For UnitDoF6 (U228) -- 6-axis BMI270
// #define USING_UNIT_DOF6
// For UnitDoF9 (U229) -- 9-axis BMI270 + BMM350
// #define USING_UNIT_DOF9
// For UnitDoF10 (U230) -- 10-axis BMI270 + BMM350 + SPA06 barometer
// #define USING_UNIT_DOF10
// For UnitMiniIMUPro (U171) -- 10-axis BMI270 + BMM150 + BMP280 barometer
// #define USING_UNIT_MINI_IMU_PRO
#endif

#include "main/PlotToSerial.cpp"
