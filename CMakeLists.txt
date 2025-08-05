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

# Add Preprocessor Macro Definitions
add_definitions(-DARM_MATH_CM4)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME New_FrameWork_Test)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        USER/Component/Algorithm/alg_ramp.c
        USER/Component/Algorithm/alg_ramp.h

)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
        USER/Component/Algorithm
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        USER/Component/Algorithm/alg_pid_zkd.cpp
        USER/Component/Algorithm/drv_math.cpp
        USER/Component/Algorithm/alg_pid_zkd_capi.cpp
        USER/Component/Algorithm/alg_AHRS.c
        USER/Component/Algorithm/alg_pid.c

        USER/Component/Driver/drv_uart.c
        USER/Component/Driver/drv_can.c
        USER/Component/Driver/fifo.c
        USER/Component/Driver/drv_gpio.c
        USER/Component/Driver/drv_timer.c
        USER/Component/Driver/drv_i2c.c

        USER/Device/dvc_led.c
        USER/Device/dvc_buzzer.c
        USER/Device/dvc_laser.c
        USER/Device/dvc_app_gpio.c
        USER/Device/dvc_DJmotor.c
        USER/Device/dvc_DMmotor.c
        USER/Device/Dvc_dt7.c
        USER/Device/dvc_heater.c
        USER/Device/dvc_soft_timer.c
        USER/Device/dvc_BMI088.c
        USER/Device/dvc_IST8310.c

        USER/Module/mdl_Chassis.c
        USER/Module/mdl_Gimbal.c
        USER/Module/motor.c
        USER/Module/mdl_IMU.c

        USER/Task/task_Shoot.c
        USER/Task/task_Communicate.c
        USER/Task/task_Start.c
        USER/Task/task_SoftwareTimer_Check.c
        USER/Task/task_Detect.c
        USER/Task/task_IMU_Update.c
        USER/Task/task_Console.c
        USER/Task/task_Chassis_ctrl.c
        USER/Task/task_Gimbal_ctrl.c


)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        USER/Component/Algorithm
        USER/Device
        USER/Component/Driver
        USER/Task
        USER/Module
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
