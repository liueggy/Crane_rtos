cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Crane_rtos)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 交叉编译兜底：当 CLion 等 IDE 直接传 -DCMAKE_C_COMPILER=arm-none-eabi-gcc
# 而不走 cmake/gcc-arm-none-eabi.cmake toolchain file 时，CMake 默认按原生
# Windows 编译器执行可执行链接测试，传入 --major-image-version / -lkernel32
# 等 ARM 链接器不认识的参数。
# 此处提前标记交叉编译环境，跳过可执行链接测试。
# 必须在 project() 之前设置 CMAKE_SYSTEM_NAME 和 CMAKE_TRY_COMPILE_TARGET_TYPE。
if(CMAKE_C_COMPILER MATCHES "arm-none-eabi-")
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
    # 标记未使用 toolchain file，project() 之后补全编译/链接标志
    set(ARM_TOOLCHAIN_FALLBACK TRUE)
endif()

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 未使用 toolchain file 时，从 cmake/gcc-arm-none-eabi.cmake 同步必要标志
if(ARM_TOOLCHAIN_FALLBACK)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m3 -mthumb -Wall -fdata-sections -ffunction-sections -fstack-usage")
    set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mcpu=cortex-m3 -mthumb -x assembler-with-cpp -MMD -MP")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mcpu=cortex-m3 -mthumb -T \"${CMAKE_SOURCE_DIR}/STM32F103XX_FLASH.ld\" --specs=nano.specs -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections -Wl,--print-memory-usage")
    unset(ARM_TOOLCHAIN_FALLBACK)
endif()

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/oled.c
    Core/Src/font.c
    Core/Src/app_config.c
    Core/Src/app_state.c
    Core/Src/buzzer.c
    Core/Src/dc_motor.c
    Core/Src/encoder.c
    Core/Src/motor_control.c
    Core/Src/ui_manager.c
    Core/Src/servo_control.c
    Core/Src/camera_tilt.c
    Core/Src/photo_sensor.c
    Core/Src/stepper_axis.c
    Core/Src/chassis_motion.c
    Core/Src/safety_manager.c
    Core/Src/input_manager.c
    Core/Src/infrared_remote.c
    Core/Src/k230_link.c
    Core/Src/box_calibration.c
    Core/Src/z_calibration.c
    Core/Src/initialization_debug.c
    Core/Src/odometry_calibration.c
    Core/Src/bean_pickup_demo.c
    Core/Src/bean_sequence_demo.c
    Core/Src/xy_waypoint_demo.c
    Core/Src/drop_demo.c
    Core/Src/world_map.c
    Core/Src/vision_survey.c
    Core/Src/vision_route_demo.c
    Core/Src/mission_planner.c
    Core/Src/robot_routes.c
    Core/Src/route_executor.c
    Core/Src/robot_controller.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
