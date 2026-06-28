cmake_minimum_required(VERSION 3.20.0)

set(BOARD rpi_pico2/rp2350a/m33)

set(DTC_OVERLAY_FILE "${CMAKE_SOURCE_DIR}/boards/ctl.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ctl)

target_sources(app PRIVATE
    src/main.c
)
