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
set(CMAKE_PROJECT_NAME Model_F4)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        MY_DRIVE/MY_DDS/ad9959.c
        MY_DRIVE/MY_DDS_FUN/9959_scan.h
        MY_DRIVE/MY_DDS_FUN/9959_scan.c
        MY_DRIVE/MY_FFT/DFT.c
        MY_DRIVE/MY_FFT/Phase.c
        MY_DRIVE/MY_FFT/WindowFunction.c
        MY_DRIVE/MY_Uart/ZPN_Uart.c
        MY_DRIVE/MY_ADF4002/adf4002.c
        MY_DRIVE/MY_MaCalculate/Modulation.h
        MY_DRIVE/MY_MaCalculate/Modulation.c
        MY_DRIVE/MY_MaCalculate/ModRecognizer.c
        MY_DRIVE/MY_Uart/ZPN_Hmi.c
        MY_DRIVE/MY_Uart/ZPN_Hmi_Pack.c
        MY_DRIVE/MY_Uart/ZPN_Hmi.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB DSP_SOURCES
    Drivers/CMSIS/DSP/Source/TransformFunctions/TransformFunctions.c
    Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c
    Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
    Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.c
    Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c
)
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
####################AD9959##################
    MY_DRIVE/MY_DDS/ad9959.c
    MY_DRIVE/MY_DDS_FUN/9959_scan.c
####################FFT&DFT#################
    MY_DRIVE/MY_FFT/DFT.c
    MY_DRIVE/MY_FFT/Phase.c
    MY_DRIVE/MY_FFT/WindowFunction.c
####################UART####################
    MY_DRIVE/MY_Uart/ZPN_Uart.c
    MY_DRIVE/MY_Uart/ZPN_Hmi.c
    MY_DRIVE/MY_Uart/ZPN_Hmi_Pack.c
    ${DSP_SOURCES}
###################ADF4002##################
    MY_DRIVE/MY_ADF4002/adf4002.c
####################MaCalculate#############
    MY_DRIVE/MY_MaCalculate/Modulation.c
    MY_DRIVE/MY_MaCalculate/ModRecognizer.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    MY_DRIVE/MY_DDS
    MY_DRIVE/MY_DDS_FUN
    MY_DRIVE/MY_FFT
    MY_DRIVE/MY_Uart
    MY_DRIVE/MY_ADF4002
    MY_DRIVE/MY_MaCalculate
    Drivers/CMSIS/DSP/Include
    Drivers/CMSIS/DSP/PrivateInclude

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM4
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
