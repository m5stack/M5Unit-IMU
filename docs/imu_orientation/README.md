# Built-in IMU orientation diagrams

Axis-orientation ("IMU 三軸方向") reference images for M5Stack hosts with a built-in IMU,
collected from the official M5Stack documentation. Use them to set the correct axis remap
(`ahrs.writeAxisOrder` / `m5::imu::presets::*`) for each board.

| Board | Built-in IMU | Image | Source |
|---|---|---|---|
| Core2 / Core2 v1.1 / AWS | MPU6886 (0x68) | `imu-core2.jpg` | https://docs.m5stack.com/en/core/core2 |
| CoreS3 / CoreS3-Lite | BMI270 (0x69) + BMM150 (0x10, AUX) | `imu-cores3.jpg` | https://docs.m5stack.com/en/core/CoreS3 |
| Fire | MPU6886 (0x68) + BMM150 (0x10) | `imu-fire.jpg` | https://docs.m5stack.com/en/core/fire |
| Gray | MPU6886 (0x68) + BMM150 (0x10) | `imu-gray.jpg` | https://docs.m5stack.com/en/core/gray |
| AtomMatrix | MPU6886 (0x68) | `imu-atom-matrix.jpg` | https://docs.m5stack.com/en/core/ATOM%20Matrix |
| AtomS3 | MPU6886 (0x68) | `imu-atoms3.jpg` | https://docs.m5stack.com/en/core/AtomS3 |
| AtomS3R (+CAM/Ext/M12) | BMI270 (0x68) | `imu-atoms3r.jpg` | https://docs.m5stack.com/en/core/AtomS3R |
| StickC-Plus | MPU6886 (0x68) | `imu-stickcplus.jpg` | https://docs.m5stack.com/en/core/m5stickc_plus |
| StickC-Plus2 | MPU6886 (0x68) | `imu-stickcplus2.jpg` | https://docs.m5stack.com/en/core/M5StickC%20PLUS2 |
| StickS3 | BMI270 (0x68) | `imu-sticks3.jpg` | https://docs.m5stack.com/en/arduino/m5sticks3/imu |
| Capsule | BMI270 (0x69) | `imu-capsule.jpg` | https://docs.m5stack.com/en/core/M5Capsule |
| PaperS3 | BMI270 (0x68) | `imu-papers3.jpg` | https://docs.m5stack.com/en/core/papers3 |

Notes:
- Gray and Fire share the same MPU6886 + BMM150 axis diagram (identical layout).
- Images are © M5Stack, reproduced here for development reference. All were converted to JPEG
  (Gray/Fire were originally .webp).
