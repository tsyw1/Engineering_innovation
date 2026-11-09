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
set(CMAKE_PROJECT_NAME Engineering_innovation)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        BSP/motor_can.c
        BSP/motor_can.h
        BSP/motor_TTL.c
        BSP/motor_TTL.h
        BSP/chassis_mecanum.c
        BSP/chassis_mecanum.h
        BSP/chassis_odometry_motion_config.h
        BSP/chassis_velocity_test.c
        BSP/chassis_velocity_test.h
        BSP/square_odometry_test.c
        BSP/square_odometry_test.h
        BSP/heading_hold_test.c
        BSP/heading_hold_test.h
        BSP/arm_z_servo_axis.c
        BSP/arm_z_servo_axis.h
        BSP/arm_lift_axis.c
        BSP/arm_lift_axis.h
        BSP/arm_horizontal_axis.c
        BSP/arm_horizontal_axis.h
        BSP/arm_controller.c
        BSP/arm_controller.h
        BSP/servo.c
        BSP/servo.h
        BSP/joho_bus_servo.c
        BSP/joho_bus_servo.h
        BSP/servo_uart_test.c
        BSP/servo_uart_test.h
        BSP/gripper_servo.c
        BSP/gripper_servo.h
        BSP/material_tray_servo.c
        BSP/material_tray_servo.h
        BSP/IMU_IIC.c
        BSP/IMU_IIC.h
        BSP/QR.c
        BSP/QR.h
        BSP/task_plan.c
        BSP/task_plan.h
        BSP/vision_uart.c
        BSP/vision_uart.h
        BSP/vision_fine_adjust.c
        BSP/vision_fine_adjust.h
        APP/task1_start_qr.c
        APP/task1_start_qr.h
        APP/task2_platform_pick.c
        APP/task2_platform_pick.h
          APP/task3_coarse_ring.c
          APP/task3_coarse_ring.h
          APP/task4_ring_return.c
          APP/task4_ring_return.h
          APP/task5_temporary_ring.c
          APP/task5_temporary_ring.h
          APP/task6_platform_pick.c
          APP/task6_platform_pick.h
          APP/task7_coarse_ring.c
          APP/task7_coarse_ring.h
          APP/task8_ring_return.c
          APP/task8_ring_return.h
          APP/task9_temporary_stack.c
          APP/task9_temporary_stack.h
          APP/task10_return_home.c
          APP/task10_return_home.h
          APP/target_ring_place_test.c
        APP/target_ring_place_test.h
        APP/chassis_full_flow_test.c
        APP/chassis_full_flow_test.h
        APP/chassis_motion_closed_loop.c
        APP/chassis_motion_closed_loop.h
        BSP/screen.c
        BSP/screen.h
        BSP/start_zone.h
        BSP/HWT101CT.c
        BSP/HWT101CT.h
        Algorithm/JY901S_ACCEL_VELOCITY.c
        Algorithm/JY901S_ACCEL_VELOCITY.h
        Algorithm/VELOCITY_COMPLEMENTARY_FILTER.c
        Algorithm/VELOCITY_COMPLEMENTARY_FILTER.h
        Algorithm/VELOCITY_ODOMETRY.c
        Algorithm/VELOCITY_ODOMETRY.h
        Algorithm/HEADING_HOLD_CONTROLLER.c
        Algorithm/HEADING_HOLD_CONTROLLER.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/APP
    ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm
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
