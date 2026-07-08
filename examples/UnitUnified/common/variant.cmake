# Map the Kconfig "Target device" choice (see common/Kconfig.variant) to the source-level USING_*
# macro that both the Arduino and ESP-IDF builds use. Include this from an example's
# main/CMakeLists.txt *after* idf_component_register() (it needs ${COMPONENT_LIB}).
# Only one device is supported so far (UnitIMUMini); more choices will be added to
# common/Kconfig.variant (and this mapping) in later plans.
set(M5UNIT_VARIANT USING_UNIT_IMU_MINI)
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${M5UNIT_VARIANT})
