# M5Unit-IMU

## Overview

Contains M5Stack **UNIT IMU** series related case programs. M5Unit-IMU is an
[M5UnitUnified](https://github.com/m5stack/M5UnitUnified) driver library for M5Stack IMU units and
the built-in IMUs of the Core / Stick / Atom series, with an `AhrsController` layer that fuses them
through selectable attitude filters (Complementary / Madgwick / Mahony / EKF / Fusion / VQF).

### SKU:U095

Unit Mini IMU is a 6-axis attitude sensor that integrates a 3-axis gravity accelerometer and a
3-axis gyroscope, capable of real-time calculation of tilt angles and acceleration. The chip uses
the MPU6886, which features a 16-bit ADC, built-in programmable digital filters, and an on-chip
temperature sensor. It communicates with the host via an I2C interface (addr:0x68) and supports
low-power mode.

### SKU:U228

Unit DoF6 (TBD)

### SKU:U229

Unit DoF9 (TBD)

### SKU:U230

Unit DoF10 (TBD)

### SKU:U231

Unit BMM350 (TBD)

## Related Link

- [Document & Datasheet - UnitMiniIMU (U095)](https://docs.m5stack.com/en/unit/imu)
- Document & Datasheet - Unit DoF6 (U228) — TBD
- Document & Datasheet - Unit DoF9 (U229) — TBD
- Document & Datasheet - Unit DoF10 (U230) — TBD
- Document & Datasheet - Unit BMM350 (U231) — TBD
- [Built-in IMU orientation diagrams](docs/imu_orientation/README.md) — per-board axis-orientation
  references for the built-in IMUs (for setting the correct axis remap)

## Required Libraries:

- [M5UnitUnified](https://github.com/m5stack/M5UnitUnified)
- [M5Utility](https://github.com/m5stack/M5Utility)
- [M5HAL](https://github.com/m5stack/M5HAL)

## License

- [M5Unit-IMU - MIT](LICENSE)

## Examples

See [examples/UnitUnified](examples/UnitUnified) — `PlotToSerial` (raw values) and `AhrsDemo`
(attitude, with an on-device 3D gizmo and, on WiFi boards, a browser viewer at 192.168.4.1).

### For ArduinoIDE settings

You must choose a define symbol for the unit/device you will use. (Uncomment the corresponding
`#define` in the example, or specify it with compile options.)

- PlotToSerial

```cpp
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_BMM350)
// For UnitMiniIMU (SKU: TBD)
#define USING_UNIT_MINI_IMU
// For UnitBMM350 (U231)
// #define USING_UNIT_BMM350
#endif
```

- AhrsDemo

```cpp
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_MINI_IMU_WITH_BMM350) && !defined(USING_BUILTIN_IMU)
// For UnitMiniIMU (SKU: TBD) - 6-axis, GROVE
#define USING_UNIT_MINI_IMU
// For UnitMiniIMU + UnitBMM350 (U231) - 9-axis on GROVE (0x68 + 0x14 coexist)
// #define USING_UNIT_MINI_IMU_WITH_BMM350
// For built-in IMU (Core2 / CoreS3 / AtomS3R / M5Stack Core/Fire)
// #define USING_BUILTIN_IMU
#endif
```

### For ESP-IDF settings

> **NOTE:** These examples target ESP-IDF 5.x (>=5.0). M5Unified / M5GFX do not yet support
> ESP-IDF 6.x; stay on the latest 5.x release.

`common/variant.cmake` maps the Kconfig `CONFIG_EXAMPLE_USING_*` choice to the source-level
`USING_*` macro shared with the Arduino build, so you do not edit the example source.

```sh
cd examples/UnitUnified/AhrsDemo
idf.py set-target esp32s3
idf.py menuconfig
# -> "M5Unit-IMU example" -> "Target device" -> choose ONE:
#      UnitMiniIMU (MPU6886) / UnitBMM350 (U231) /
#      UnitMiniIMU + UnitBMM350, 9-axis GROVE (AhrsDemo only) /
#      Built-in IMU, M5.getBoard() auto-detect (AhrsDemo only)
idf.py build flash monitor
```

## Doxygen document

[GitHub Pages](https://m5stack.github.io/M5Unit-IMU/)

If you want to generate documents on your local machine, execute the following command

```
bash docs/doxy.sh
```

It will output it under docs/html  
If you want to output Git commit hashes to html, do it for the git cloned folder.

### Required

- [Doxygen](https://www.doxygen.nl/)
- [Git](https://git-scm.com/) (Output commit hash to html)
