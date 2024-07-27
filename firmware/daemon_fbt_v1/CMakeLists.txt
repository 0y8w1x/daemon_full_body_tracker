cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(daemon_fbt_v1)

zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/flash)

FILE(GLOB sources src/*.c)
target_sources(app PRIVATE ${sources} ${flash_srcs})
