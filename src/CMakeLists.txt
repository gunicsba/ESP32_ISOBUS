# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources *.cpp *.c)

set(EXTRA_COMPONENT_DIRS
    ../.pio/build/esp32s3/_deps/espidflib-src/components
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "include"
    ${espidflib_SOURCE_DIR}/components
    REQUIRES pca9685
)
target_add_binary_data(${COMPONENT_TARGET} "object_pool/object_pool.iop" BINARY)