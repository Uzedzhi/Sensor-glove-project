idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_i2c esp_timer
)
set_target_properties(${COMPONENT_LIB} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
# ESP-IDF adds its own default -std flag after CMake's CXX_STANDARD flag.
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)
