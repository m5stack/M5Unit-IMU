# ESP-IDF component manifest for M5Unit-IMU.
# Upstream: https://github.com/m5stack/M5Unit-IMU
#
# IMU sensors (MPU6886 / BMI270 / BMM150 / BMM350) over I2C. The public headers only
# pull in M5UnitUnified; M5Utility is used in the .cpp internals only.
file(GLOB_RECURSE SRCS "src/*.cpp")
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES M5UnitUnified
    PRIV_REQUIRES M5Utility
)
