cmake_minimum_required(VERSION 3.16)

# Keep application code in src/ instead of ESP-IDF's usual main/ directory.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/src")
# Build only src and its declared dependencies, not every ESP-IDF component.
set(COMPONENTS "src")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sensor_glove_blink)
