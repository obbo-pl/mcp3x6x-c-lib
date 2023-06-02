# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
#set(PICO_SDK_PATH "C:\pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# include(pico_extras_import_optional.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(mcp3x6x-pico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

#AUX_SOURCE_DIRECTORY(. SRC_LIST)
#FILE(GLOB SRC_LIST "./*.c")
#ADD_EXECUTABLE(mcp3x6x-pico ${SRC_FILE})
add_executable(mcp3x6x-pico 
        mcp3x6x-example.c 
        spi_mcp3x6x.c 
        spi_bus_hal.c
        )

# Add executable. Default name is the project name, version 0.1
pico_set_program_name(mcp3x6x-pico "mcp3x6x-pico")
pico_set_program_version(mcp3x6x-pico "1.0")

# Add the standard library to the build
target_link_libraries(mcp3x6x-pico
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(mcp3x6x-pico PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Add any user requested libraries
target_link_libraries(mcp3x6x-pico
        hardware_spi
        )

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

