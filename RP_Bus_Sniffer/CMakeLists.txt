# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(RP_Bus_Sniffer C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

# 构建时间戳：每次 build 重新生成一个只有一行的 C 文件，链接进固件后
# 开机从串口打印。这样无论改的是哪个源文件，打印的时间都是“这一次构建”，
# 不会像 __DATE__/__TIME__ 那样停在某个 .c 上一次编译的时刻。
#
# 用 add_custom_target 而不是 add_custom_command(OUTPUT ...)：没有输出
# 文件的 custom target 每次构建都会执行，而 Ninja 会把“无输入的 OUTPUT”
# 当成已是最新，时间戳就永远停在上一次。
add_custom_target(build_stamp
        COMMAND ${CMAKE_COMMAND}
                -DOUT=${CMAKE_CURRENT_BINARY_DIR}/build_stamp.c
                -P ${CMAKE_CURRENT_LIST_DIR}/cmake/gen_build_stamp.cmake
        BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/build_stamp.c
        COMMENT "生成固件构建时间戳")

# FatFs R0.15：Pico SDK 的 tinyusb 子模块里带了一份完整源码，
# 直接编进来用，不再单独拉第三方库。diskio 在 drivers/sdcard/sdcard.c 里实现。
if(NOT PICO_SDK_PATH)
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
set(FATFS_DIR ${PICO_SDK_PATH}/lib/tinyusb/lib/fatfs/source)
add_library(fatfs STATIC
        ${FATFS_DIR}/ff.c
        ${FATFS_DIR}/ffunicode.c)
target_include_directories(fatfs PUBLIC ${FATFS_DIR})

# 源码组织：应用入口在根目录，每个驱动一个子目录放在 drivers/ 下
add_executable(RP_Bus_Sniffer
        RP_Bus_Sniffer.c              # Core 0：时钟、UI、mailbox
        drivers/sniffer/sniffer.c     # Core 1：PIO + DMA 抓包引擎
        drivers/st7789/st7789.c       # 板载 1.14" ST7789 LCD
        drivers/sdcard/sdcard.c       # TF 卡（SPI 模式）+ FatFs diskio
        ${CMAKE_CURRENT_BINARY_DIR}/build_stamp.c   # 构建时间戳（自动生成）
        )

# 保证 build_stamp 先跑，生成的 build_stamp.c 在编译之前就已是最新
add_dependencies(RP_Bus_Sniffer build_stamp)

pico_set_program_name(RP_Bus_Sniffer "RP_Bus_Sniffer")
pico_set_program_version(RP_Bus_Sniffer "0.1")

# Generate PIO header（生成到 build/，直接 #include "spi_sniffer.pio.h"）
pico_generate_pio_header(RP_Bus_Sniffer
        ${CMAKE_CURRENT_LIST_DIR}/drivers/sniffer/spi_sniffer.pio)
# TF 卡的 SPI 主机（PIO1）
pico_generate_pio_header(RP_Bus_Sniffer
        ${CMAKE_CURRENT_LIST_DIR}/drivers/sdcard/sd_spi.pio)

# Core 0 通过 UART0(USB 转串口) 和 USB CDC 输出调试日志
pico_enable_stdio_uart(RP_Bus_Sniffer 1)
pico_enable_stdio_usb(RP_Bus_Sniffer 1)

# Add the standard library to the build
target_link_libraries(RP_Bus_Sniffer
        pico_stdlib
        pico_multicore
        hardware_spi
        hardware_pio
        hardware_dma
        hardware_clocks
        hardware_gpio
        hardware_sync
        fatfs
        )

# Add the standard include files to the build
target_include_directories(RP_Bus_Sniffer PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}                   # board.h
        ${CMAKE_CURRENT_LIST_DIR}/drivers/sniffer
        ${CMAKE_CURRENT_LIST_DIR}/drivers/st7789
        ${CMAKE_CURRENT_LIST_DIR}/drivers/sdcard
)

pico_add_extra_outputs(RP_Bus_Sniffer)
