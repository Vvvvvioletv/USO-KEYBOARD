# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(FAMILY "rp2040")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "D:/QianRuShi/pico-sdk")
set(CMAKE_BUILD_TYPE Release)
if(CMAKE_BUILD_TYPE MATCHES Release)
  add_compile_options(-O3)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s")
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(MULTITHREADED_BUILD 8)

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
# include(pico_sdk_import.cmake)

include(D:/QianRuShi/pico-sdk/lib/tinyusb/hw/bsp/family_support.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(uso C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(uso
  uso.c
  # usb_descriptors.c
)
add_subdirectory(ssd1306)
add_subdirectory(usb)

pico_generate_pio_header(uso ${CMAKE_CURRENT_LIST_DIR}/ssd1306/spi.pio)

pico_set_program_name(uso "uso")
pico_set_program_version(uso "0.1")

pico_enable_stdio_uart(uso 0)
pico_enable_stdio_usb(uso 1)

# Add the standard library to the build
target_link_libraries(uso
  pico_stdlib
  hardware_spi
  hardware_clocks
  hardware_dma
  hardware_adc
  pico_multicore
  pico_sync
  pico_util
  tinyusb_device
  tinyusb_board
  hardware_pio
)

# Add the standard include files to the build
target_include_directories(uso PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
  ${CMAKE_SOURCE_DIR}/include
  ssd1306
  usb
)

# Add any user requested libraries
target_link_libraries(uso 
  ssd1306
  usb
)

pico_add_extra_outputs(uso)

