
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sdtool)

FILE(GLOB_RECURSE app_sources src/*.c*)
target_sources(app PRIVATE ${app_sources})

set(APP_BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_custom_target(copy_fw ALL
  DEPENDS ${logical_target_for_zephyr_elf}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${APP_BIN_DIR}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:${logical_target_for_zephyr_elf}>
          ${APP_BIN_DIR}/sdtool.elf
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_BINARY_DIR}/zephyr/zephyr.bin
          ${APP_BIN_DIR}/sdtool.bin
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_BINARY_DIR}/zephyr/zephyr.hex
          ${APP_BIN_DIR}/sdtool.hex
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_BINARY_DIR}/zephyr/zephyr.uf2
          ${APP_BIN_DIR}/sdtool.uf2
)
