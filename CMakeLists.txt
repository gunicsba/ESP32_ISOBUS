cmake_minimum_required(VERSION 3.16.0)

include(FetchContent)
FetchContent_Declare(
  espidflib
  GIT_REPOSITORY https://github.com/UncleRus/esp-idf-lib.git
)
FetchContent_MakeAvailable(espidflib)

set(EXTRA_COMPONENT_DIRS
    ${espidflib_SOURCE_DIR}/components
    ${CMAKE_SOURCE_DIR}/src
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(ESP32_ISOBUS)

