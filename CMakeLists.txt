cmake_minimum_required(VERSION 3.20.0)

# Pick up the local board definition under boards/arm/nice_nano/
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(irtransmitter)

target_sources(app PRIVATE
    src/main.c
    src/ir.c
    src/ir_rx.c
    src/crashlog.c
)

target_include_directories(app PRIVATE src)
