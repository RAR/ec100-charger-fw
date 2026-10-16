# oem/CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)
project(oem C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CPU_FLAGS -mcpu=cortex-m23 -mthumb)
# The image trailer's version, from OEM_FW_VERSION (0xMMmm) so it can't drift from IR 0.
file(STRINGS src/app/modbus_map.h OEM_FW_LINE REGEX "OEM_FW_VERSION = 0x")
string(REGEX REPLACE ".*0x([0-9A-Fa-f][0-9A-Fa-f])([0-9A-Fa-f][0-9A-Fa-f]).*" "\\1;\\2" OEM_FW_HEX "${OEM_FW_LINE}")
list(GET OEM_FW_HEX 0 OEM_FW_MAJOR_HEX)
list(GET OEM_FW_HEX 1 OEM_FW_MINOR_HEX)
math(EXPR OEM_FW_MAJOR "0x${OEM_FW_MAJOR_HEX}")
math(EXPR OEM_FW_MINOR "0x${OEM_FW_MINOR_HEX}")
set(OEM_VERSION ${OEM_FW_MAJOR}.${OEM_FW_MINOR})

add_compile_options(${CPU_FLAGS} -Os -g -ffunction-sections -fdata-sections -Wall -Wextra -Werror
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics;-fno-use-cxa-atexit>")

set(HW_SOURCES src/startup.c src/hw/clock.cpp src/hw/slcd.cpp)

# Phase-2 drivers, built on their own so each one cross-compiles before the charger app links it.
add_library(oem_drivers STATIC src/hw/flash.cpp src/hw/can.cpp src/hw/adc.cpp src/hw/rs485.cpp)
target_include_directories(oem_drivers PRIVATE src)

function(oem_app name)
  add_executable(${name}.elf ${ARGN})
  target_include_directories(${name}.elf PRIVATE src ${CMAKE_CURRENT_SOURCE_DIR}/../esphome/components)
  target_link_options(${name}.elf PRIVATE ${CPU_FLAGS} -T${CMAKE_CURRENT_SOURCE_DIR}/ld/app.ld -nostartfiles
    --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=${name}.map)
  add_custom_command(TARGET ${name}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${name}.elf ${name}.bin
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/../tools/oem_image.py ${name}.bin ${name}.dat --version ${OEM_VERSION})
endfunction()

oem_app(charger ${HW_SOURCES} src/main.cpp)
target_link_libraries(charger.elf PRIVATE oem_drivers)
