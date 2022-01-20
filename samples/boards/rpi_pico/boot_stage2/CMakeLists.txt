#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.20.0)

# This is not a Zephyr sample, but a minimal stage2 bootloader.
# However, we reuse the Zephyr build infrastructure for Zephyr modules,
# board processing, toolchain and other related tools
# List of Zephyr CMake modules required.
set(zephyr_cmake_modules extensions west root zephyr_module boards shields arch
    build_configuration host-tools generic_toolchain kconfig soc target_toolchain
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE} COMPONENTS ${zephyr_cmake_modules})
project(rpi_boot2)

enable_language(C CXX ASM)

# Test the toolchain.
zephyr_check_compiler_flag(C "" toolchain_is_ok)
assert(toolchain_is_ok "The toolchain is unable to build a dummy C file. See CMakeError.log.")

# Load flash and friends.
add_subdirectory(${ZEPHYR_BASE}/cmake/flash flash)

set(rp2_common_dir ${ZEPHYR_HAL_RPI_PICO_MODULE_DIR}/src/rp2_common)
set(rp2040_dir ${ZEPHYR_HAL_RPI_PICO_MODULE_DIR}/src/rp2040)
set(common_dir ${ZEPHYR_HAL_RPI_PICO_MODULE_DIR}/src/common)
set(boot_stage_dir ${rp2_common_dir}/boot_stage2)

# Compile the second stage bootloader

add_executable(boot_stage2)

target_sources_ifdef(CONFIG_RP2_FLASH_W25Q080 boot_stage2 PRIVATE
      ${boot_stage_dir}/boot2_w25q080.S)
target_sources_ifdef(CONFIG_RP2_FLASH_GENERIC_03H boot_stage2 PRIVATE
      ${boot_stage_dir}/boot2_generic_03h.S)
target_sources_ifdef(CONFIG_RP2_FLASH_IS25LP080 boot_stage2 PRIVATE
      ${boot_stage_dir}/boot2_is25lp080.S)
target_sources_ifdef(CONFIG_RP2_FLASH_W25X10CL boot_stage2 PRIVATE
      ${boot_stage_dir}/boot2_w25x10cl.S)

target_include_directories(boot_stage2 PUBLIC
  ${ZEPHYR_BASE}/modules/hal_rpi_pico
  ${boot_stage_dir}/asminclude
  ${rp2_common_dir}/pico_platform/include
  ${rp2040_dir}/hardware_regs/include
  ${common_dir}/pico_base/include
  )

target_link_options(boot_stage2 PRIVATE
  "-nostartfiles"
  "--specs=nosys.specs"
  "LINKER:--script=${boot_stage_dir}/boot_stage2.ld"
  )

# Generates a binary file from the compiled bootloader
add_custom_target(bootloader_bin ALL DEPENDS boot_stage2.bin)
add_custom_command(OUTPUT boot_stage2.bin
  DEPENDS boot_stage2
  COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:boot_stage2> boot_stage2.bin
  )

# @yonsh I assume you still will need to pad checksum, and then convert the
# final result to a hex or bin file before flashing, correct ?
# Checksums the binary, pads it, and generates an assembly file
#add_custom_target(boot_stage2_asm ALL DEPENDS boot_stage2.S)
#add_custom_command(OUTPUT boot_stage2.S
#  DEPENDS boot_stage2.bin
#  COMMAND ${Python3_EXECUTABLE} ${boot_stage_dir}/pad_checksum
#  -s 0xffffffff boot_stage2.bin boot_stage2.S)
