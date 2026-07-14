# Map the Kconfig "Target device" choice (see common/Kconfig.variant) to the source-level USING_*
# macro that both the Arduino and ESP-IDF builds use. Include this from an example's
# main/CMakeLists.txt *after* idf_component_register() (it needs ${COMPONENT_LIB}).
# Default (nothing selected): UnitMiniIMU (MPU6886).
# EXAMPLE_USING_UNIT_MINI_IMU_WITH_BMM350 / EXAMPLE_USING_BUILTIN_IMU are AhrsDemo-only variants
# (PlotToSerial only recognizes USING_UNIT_MINI_IMU/USING_UNIT_BMM350; selecting one of these two
# for PlotToSerial would hit its `#error Please choose unit!`).
if(CONFIG_EXAMPLE_USING_UNIT_BMM350)
    set(M5UNIT_VARIANT USING_UNIT_BMM350)
elseif(CONFIG_EXAMPLE_USING_UNIT_MINI_IMU_WITH_BMM350)
    set(M5UNIT_VARIANT USING_UNIT_MINI_IMU_WITH_BMM350)
elseif(CONFIG_EXAMPLE_USING_BUILTIN_IMU)
    set(M5UNIT_VARIANT USING_BUILTIN_IMU)
else()
    set(M5UNIT_VARIANT USING_UNIT_MINI_IMU)
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${M5UNIT_VARIANT})
