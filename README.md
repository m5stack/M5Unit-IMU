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

### SKU:U171

Unit Mini IMU-Pro is a 10-axis motion sensor unit: a BMI270 6-axis accel/gyro (0x68) and a BMP280
barometer (0x76) on the GROVE I2C bus, plus a BMM150 3-axis magnetometer wired to the BMI270's AUX
interface rather than to the GROVE bus (driver: `UnitMiniIMUPro` composite; the BMP280 driver
`UnitBMP280` comes from M5Unit-ENV). Because the magnetometer is an AUX target, `imu()` alone
provides all nine axes -- the same arrangement as the built-in IMU on CoreS3 and AtomS3R.

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
>
> **UnitMiniIMU + UnitBMM350 (9-axis pair, AhrsDemo):** both units share ONE GROVE bus — split the
> port with a hub (e.g. Unit HUB). 0x68 + 0x14 coexist without an address conflict. On a classic
> M5Stack Core the shared bus is routed to GROVE Port C (needs the M5GO/Fire base) for the same
> built-in-IMU reason; the example does this automatically.
>
> **Built-in IMU + UnitBMM350 (AhrsDemo `USING_BUILTIN_IMU_WITH_BMM350`):** most hosts with a
> built-in IMU have no magnetometer (see the table below), so plugging a Unit BMM350 into the GROVE
> port is the cheapest way to make them 9-axis. The magnetometer needs no bus juggling — 0x14
> collides with nothing — except on a classic M5Stack Core, where the default GROVE port *is* the
> internal bus the built-in IMU already uses, so the example routes it to Port C.
>
> Unlike the DoF units, the two devices are separate enclosures: **mount the Unit BMM350 in the same
> orientation as the host**, or express the difference with `AhrsController::writeMagAxisOrder()`.
> The library corrects the BMM350 chip against the unit's own case; how that case sits relative to
> the host is a mounting decision only you know.

## Built-in IMUs by board revision

Many M5Stack boards changed their built-in IMU across production revisions **without changing the
board id** -- `M5.getBoard()` cannot tell the generations apart, so the examples probe the I2C bus
at startup (`detectBuiltinImu` / `detectBuiltinMag`) and pick the driver from what actually
responds. All chips below have drivers in this library.

| Board | Revision | Accel/Gyro | Magnetometer |
|---|---|---|---|
| M5Stack Basic | all | none (a few undocumented early lots: MPU9250) | none |
| M5Stack Gray | 2017 (original) | MPU6050 (0x68) | MAG3110 (0x0E) |
| | 2017.12 | MPU9250 (0x68) | AK8963 (0x0C, in-package) |
| | 2019.6 - current | MPU6886 (0x68) | BMM150 (0x10) |
| M5Stack Fire | - 2019.6 (original) | MPU9250 (0x68) | AK8963 (0x0C, in-package) |
| | 2019.7 | SH200Q (0x6C) | BMM150 (0x10) |
| | 2019.8 - 2021.7 | MPU6886 (0x68) | BMM150 (0x10) |
| | v2.6 / v2.7 (2021.8 -) | MPU6886 (0x68) | none (removed) |
| Core2 / for AWS / v1.1 | | MPU6886 (0x68) | none |
| Core2 v1.3 | | BMI270 (0x68) | none |
| CoreS3 / CoreS3 Lite | | BMI270 (0x69) | BMM150 (via BMI270 AUX) |
| CoreS3 SE | | none | none |
| Station (Bat) | | MPU6886 (0x68) | none |
| M5StickC | 2019.3 (original) | SH200Q (0x6C) | none |
| | 2019.8 - | MPU6886 (0x68) | none |
| StickC Plus / Plus2 / T2 | | MPU6886 (0x68) | none |
| StickS3 | | BMI270 (0x68) | none |
| ATOM Matrix | original | MPU6886 (0x68) | none |
| | v1.1 | BMI270 (0x68) | none |
| AtomS3 | | MPU6886 (0x68) | none |
| AtomS3R | | BMI270 (0x68) | BMM150 (via BMI270 AUX) |
| Capsule | v1.0 / v1.1 | BMI270 (0x69) | none |
| Cardputer-Adv | | BMI270 (0x68) | none |
| PaperS3 | | BMI270 (0x68) | none |
| Tab5 | | BMI270 (0x68) | none |
| Nesso N1 | | BMI270 (0x68) | none |

The list is not exhaustive (boards without an established IMU configuration are omitted). Mounting
orientation differs per board -- see the
[built-in IMU orientation diagrams](docs/imu_orientation/README.md).

## Related Link

- [Unit Mini IMU & Datasheet](https://docs.m5stack.com/en/products/sku/U095)
- [Unit DoF6 & Datasheet](https://docs.m5stack.com/en/products/sku/U228)
- [Unit DoF9 & Datasheet](https://docs.m5stack.com/en/products/sku/U229)
- [Unit DoF10 & Datasheet](https://docs.m5stack.com/en/products/sku/U230)
- [Unit BMM350 & Datasheet](https://docs.m5stack.com/en/products/sku/U231)
- [Unit Mini IMU-Pro & Datasheet](https://docs.m5stack.com/en/products/sku/U171)
- [Built-in IMU orientation diagrams](docs/imu_orientation/README.md) — per-board axis-orientation
  references for the built-in IMUs (for setting the correct axis remap)

## Required Libraries:

- [M5UnitUnified](https://github.com/m5stack/M5UnitUnified)
- [M5Utility](https://github.com/m5stack/M5Utility)
- [M5HAL](https://github.com/m5stack/M5HAL)
- [M5Unit-ENV](https://github.com/m5stack/M5Unit-ENV) — barometer drivers (UnitSPA06 / UnitBMP280)
  used by Unit DoF10 / Unit Mini IMU-Pro

## License

- [M5Unit-IMU - MIT](LICENSE)

## AHRS filters

`AhrsController` fuses any accel/gyro (+ optional mag) source through one of six attitude
filters, selectable at runtime (`selectFilter()`). All six share the same conventions -- the
output quaternion is the body-to-world rotation, and supplying a magnetometer automatically
upgrades 6-axis to 9-axis operation -- so switching filters never changes the coordinate system,
only the estimation characteristics:

| Filter | Origin | Characteristics | Pick it for |
|---|---|---|---|
| Complementary | classic textbook technique | Lightest, responsive; less accurate under dynamic motion | Minimal CPU, first steps |
| Madgwick | Madgwick 2010 | Light and proven; the paper-default gain (beta 0.1) settles slowly from a cold start | General 6/9-axis use |
| Mahony | Mahony et al. 2008 | PI feedback absorbs gyro bias (with Ki > 0) | 6-axis with noticeable gyro drift |
| EKF | standard extended Kalman filter | Estimates gyro bias in-state; highest CPU cost | Accuracy-first applications |
| Fusion | [x-io Technologies](https://github.com/xioTechnologies/Fusion) (MIT) | Built-in 3 s startup ramp: fastest to settle; widely deployed | A practical default choice |
| VQF | [Laidig & Seel 2023](https://github.com/dlaidig/vqf) (MIT) | Recent research; very low noise at rest | Static accuracy, research |

Default gains are each paper's standard values. Because those defaults make Madgwick/Mahony slow
to converge onto the magnetic heading from a cold start, AhrsDemo temporarily raises their gain
for 3 seconds after a filter is selected (the initialization strategy Madgwick's own report
recommends, mirroring Fusion's built-in ramp); steady-state behavior stays at the paper defaults.

### Third-party licenses

The AHRS layer vendors two third-party filter implementations (unmodified, under
`src/imu/filter/third_party/`):

- [Fusion](https://github.com/xioTechnologies/Fusion) — Copyright (c) 2021 x-io Technologies,
  [MIT](src/imu/filter/third_party/fusion/LICENSE.md)
- [VQF](https://github.com/dlaidig/vqf) — Copyright (c) 2021 Daniel Laidig,
  [MIT](src/imu/filter/third_party/vqf/LICENSE)

The Complementary / Madgwick / Mahony / EKF filters are original implementations, re-derived from
their published papers (Madgwick 2010; Mahony, Hamel & Pflimlin 2008).

## Magnetometer interference

All magnetometers this library drives — BMM150, BMM350, AK8963, MAG3110, whether on a GROVE unit
or built into a host — are affected by nearby magnetic fields, which distort or saturate the
reading and throw off the AHRS heading (yaw). M5Stack's own note for the BMM150 applies to every
magnetometer here:

> Products containing magnets may interfere with the magnetic sensor, resulting in abnormal
> readings. Remove any magnets and keep the sensor away from strong magnetic fields.

In practice this means keeping the sensor clear of speaker/charger magnets (including a host's own
magnetic back), motors, and **steel desks or metal enclosures** — a steel surface alone can push a
magnetometer into saturation. After relocating away from interference, perform a figure-eight
motion so the auto-calibration can re-estimate the hard-iron offset (AhrsDemo does this via its
hold-to-calibrate control). A 6-axis (accel + gyro) attitude estimate is unaffected; only the
magnetic heading depends on a clean field.

Note that calibration can only remove a distortion that is **fixed to the sensor** — a magnetised
component on the board, which turns with it. A steel desk is fixed to the *world* instead, so what
it does to the field changes as you move the device through it, and no amount of calibration
removes it; measured on a DoF9, the field magnitude read 49 µT (the correct local value) in one
spot and 88 µT after moving it a short distance across the same desk. If the magnitude changes
when you *move* the sensor rather than when you *turn* it, the environment is the problem, not the
calibration.

## Drift and auto-calibration

Yaw drift on a 6-axis (accel + gyro) setup comes almost entirely from the gyroscope's zero-rate
offset, which integrates straight into the heading. **`AhrsController` therefore corrects the gyro
offset by default** (`writeAutoCalibration()` starts at strength 1 for the gyro, 0 for accel and
mag): it watches for the device being still and subtracts what it measures, which also follows the
drift as the die warms up. `calibrateGyro()` does the same thing once, deterministically, if you
would rather calibrate at a known moment — the AhrsDemo example calls it at boot.

Accelerometer and magnetometer auto-calibration stay **off** by default: their calibration fits
samples to a sphere of a known size, and only the gyro's (the origin) and the accelerometer's
(1 G) are the same everywhere. The magnetometer's is the local field strength, which ranges over
roughly 25–65 µT worldwide, so it has to be learned per location (see
`writeMagCalibrationRadius()`).

Roll and pitch do not drift — gravity gives them an absolute reference. Yaw on a 6-axis setup has
no such reference and cannot be held indefinitely by any filter; removing the bias turns the error
from something that grows linearly with time into something that grows like its square root. Add a
magnetometer, or use `tareYaw()` to re-zero the reported heading, when an absolute heading matters.

## Examples

See [examples/UnitUnified](examples/UnitUnified) — `PlotToSerial` (raw values) and `AhrsDemo`
(attitude, with an on-device 3D gizmo and, on WiFi boards, a browser viewer at 192.168.4.1).

### For ArduinoIDE settings

You must choose a define symbol for the unit/device you will use. (Uncomment the corresponding
`#define` in the example, or specify it with compile options.)

- PlotToSerial

```cpp
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
```

For the DoF units the BMI270 I2C address follows the board's SW1 switch (0x68 default / 0x69) and is
fixed at compile time via `DOF_BMI270_ADDRESS` (edit the `#define` in the example, or add
`-D DOF_BMI270_ADDRESS=0x69` to the PlatformIO `build_flags`). The example probes the bus at startup
and warns if the build target does not match the switch position.

- AhrsDemo

```cpp
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
// For UnitMiniIMUPro (U171) -- 10-axis BMI270 + BMM150 + BMP280 barometer
// #define USING_UNIT_MINI_IMU_PRO
#endif
```

### For ESP-IDF settings

> **NOTE:** These examples target ESP-IDF 5.x (>=5.0). M5Unified / M5GFX do not yet support
> ESP-IDF 6.x; stay on the latest 5.x release.

> **NOTE:** AhrsDemo's WiFi SoftAP + browser viewer is Arduino-only (it uses the arduino-esp32
> `WiFi.h` / `WebServer.h`, which do not exist on ESP-IDF native). ESP-IDF builds output to
> Serial and the LCD only.

`common/variant.cmake` maps the Kconfig `CONFIG_EXAMPLE_USING_*` choice to the source-level
`USING_*` macro shared with the Arduino build, so you do not edit the example source.

```sh
cd examples/UnitUnified/AhrsDemo
idf.py set-target esp32s3
idf.py menuconfig
# -> "M5Unit-IMU example" -> "Target device" -> choose ONE:
#      UnitMiniIMU (U095, MPU6886) / UnitBMM350 (U231) /
#      UnitDoF6 (U228, BMI270) / UnitDoF9 (U229, BMI270+BMM350) /
#      UnitDoF10 (U230, BMI270+BMM350+SPA06) /
#      UnitMiniIMUPro (U171, BMI270+BMM150+BMP280) /
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
