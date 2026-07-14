# Dependencies (M5Unit-IMU, M5UnitUnified, M5Unified, ...) are pulled in via
# main/idf_component.yml and auto-added as requirements by the component manager.
idf_component_register(
    SRCS "PlotToSerial.cpp"
    INCLUDE_DIRS ".")

# The unit variant is selected via Kconfig (main/Kconfig.projbuild -> common/Kconfig.variant)
# with `idf.py menuconfig` or sdkconfig.defaults (e.g. CONFIG_EXAMPLE_USING_UNIT_MINI_IMU=y). The
# CONFIG_* -> USING_* mapping is shared by all UnitUnified examples. Include it after the register
# call (it needs ${COMPONENT_LIB}).
include("${CMAKE_CURRENT_LIST_DIR}/../../common/variant.cmake")
