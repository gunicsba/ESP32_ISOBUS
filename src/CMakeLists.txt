# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources *.cpp *.c)

set(EXTRA_COMPONENT_DIRS
    ${CMAKE_SOURCE_DIR}/.pio/libdeps/esp32s3/esp-idf-lib/components
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "include"
    REQUIRES pca9685
)
target_add_binary_data(${COMPONENT_TARGET} "object_pool/object_pool.iop" BINARY)