# Map the Kconfig "Target device" choice (see common/Kconfig.variant) to the source-level USING_*
# macro that both the Arduino and ESP-IDF builds use. Include this from an example's
# main/CMakeLists.txt *after* idf_component_register() (it needs ${COMPONENT_LIB}).
# Default (nothing selected): UnitIMUMini (MPU6886).
if(CONFIG_EXAMPLE_USING_UNIT_BMM350)
    set(M5UNIT_VARIANT USING_UNIT_BMM350)
else()
    set(M5UNIT_VARIANT USING_UNIT_IMU_MINI)
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${M5UNIT_VARIANT})
