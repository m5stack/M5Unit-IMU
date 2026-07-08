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
#if !defined(USING_UNIT_IMU_MINI) && !defined(USING_UNIT_BMM350)
// For UnitIMUMini (SKU: TBD)
#define USING_UNIT_IMU_MINI
// For UnitBMM350 (U231)
// #define USING_UNIT_BMM350
// Following devices will be supported in later plans:
// USING_UNIT_DOF6 (U228) / USING_BUILTIN_IMU
#endif

#include "main/PlotToSerial.cpp"
