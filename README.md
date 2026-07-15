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

> **Multiple DoF units on one bus:** BMM350 (0x14) and SPA06-003 (0x76) have fixed addresses; only the
> BMI270 is switchable (SW1 0x68/0x69, `DOF_BMI270_ADDRESS` in the examples). To use more than one
> DoF9/DoF10 on a single host, either (a) use DoF6 pairs (BMI270 only), (b) put the second unit on a
> separate bus (GROVE Port C on the free Arduino `Wire` — M5Unified uses In_I2C/Ex_I2C, not `Wire`), or
> (c) use a UnitPaHub: each unit on its own mux channel coexists at the same address (the framework
> selects the channel per transaction; see M5Unit-PaHub for hub examples).
>
> ```cpp
> // (c) sketch: two DoF9 on one host via UnitPaHub channels
> // paHub.add(dofA, 0); paHub.add(dofB, 1);
> // Units.add(paHub, Wire); Units.begin();  // channels are selected automatically per transaction
> ```
>
> On a classic M5Stack Core the DoF BMI270 (0x68) collides with the built-in IMU on GROVE Port A, so
> route the unit to GROVE Port C (`Wire`) — the PlotToSerial/AhrsDemo examples do this automatically.

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
#if !defined(USING_UNIT_MINI_IMU) && !defined(USING_UNIT_BMM350) && !defined(USING_UNIT_DOF6) && \
    !defined(USING_UNIT_DOF9) && !defined(USING_UNIT_DOF10)
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
#endif
```

For the DoF units the BMI270 I2C address follows the board's SW1 switch (0x68 default / 0x69) and is
fixed at compile time via `DOF_BMI270_ADDRESS` (edit the `#define` in the example, or add
`-D DOF_BMI270_ADDRESS=0x69` to the PlatformIO `build_flags`). The example probes the bus at startup
and warns if the build target does not match the switch position.

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
#      UnitDoF6 (U228) / UnitDoF9 (U229) / UnitDoF10 (U230) (PlotToSerial only) /
#      UnitMiniIMU + UnitBMM350, 9-axis GROVE (AhrsDemo only) /
#      Built-in IMU, M5.getBoard() auto-detect (AhrsDemo only)
#    For the DoF units, "DoF BMI270 I2C address (SW1 switch)" selects 0x68 (default) / 0x69.
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
