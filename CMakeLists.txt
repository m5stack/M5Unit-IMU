# ESP-IDF component manifest for M5Unit-IMU.
# Upstream: https://github.com/m5stack/M5Unit-IMU
#
# IMU sensors (MPU6886 / BMI270 / BMM150 / BMM350) over I2C. The public headers pull in
# M5UnitUnified, M5Utility (m5::container::CircularBuffer) and M5Unit-ENV (barometer drivers:
# UnitSPA06 / UnitBMP280).
file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.c")
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES M5UnitUnified M5Utility M5Unit-ENV
)
