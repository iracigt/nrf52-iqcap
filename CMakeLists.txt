cmake_minimum_required(VERSION 3.20)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf_iqcap)

target_sources(app PRIVATE
  src/main.c
  src/rfx.c
)

zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
