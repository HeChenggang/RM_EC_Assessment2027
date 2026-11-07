cmake_minimum_required(VERSION 3.22)
include(cmake/gcc-arm-none-eabi.cmake)
project(gimbal C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(gimbal)
# Reuse the checked-in HAL/CMSIS package; board sources remain independent.
add_subdirectory(cmake/stm32cubemx)
target_link_libraries(gimbal stm32cubemx)
target_sources(gimbal PRIVATE ../../modules/imu/bmi088.c)
target_include_directories(gimbal PRIVATE ../../modules/imu ../../config)
target_compile_options(gimbal PRIVATE -mthumb)

add_custom_command(TARGET gimbal POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:gimbal> ${CMAKE_CURRENT_BINARY_DIR}/gimbal.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:gimbal> ${CMAKE_CURRENT_BINARY_DIR}/gimbal.bin
    VERBATIM)
