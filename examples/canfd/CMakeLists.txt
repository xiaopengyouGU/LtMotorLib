#===========================================================
# LTM_FOC canfd 例程（独立构建）
# 板级映射来自 boards/<board>/board.cmake：HAL SDK、工具链、机型参数、链接脚本
# SDK 静态库用 --whole-archive 链接，确保 IRQ 向量表和所有 ISR 完整拉取
#===========================================================
cmake_minimum_required(VERSION 3.20)

get_filename_component(LTM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
if(NOT LTM_FOC_BOARD)
    set(LTM_FOC_BOARD "ra6m4")
endif()
include(${LTM_ROOT}/boards/${LTM_FOC_BOARD}/board.cmake)
get_filename_component(LTM_BOARD_LINKER_DIR "${LTM_BOARD_LINKER}" DIRECTORY)

set(CMAKE_TOOLCHAIN_FILE ${LTM_BOARD_TOOLCHAIN})

project(ltm_foc_canfd VERSION 1.0.0 LANGUAGES C ASM)

set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})

add_compile_definitions(${LTM_BOARD_DEFINES})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${LTM_HAL_SDK_DIR}/bin          # ltm_hal/ltm_hal.h（已编译 SDK 的交付头）
    ${LTM_ROOT}/LTM_FOC/bin         # ltm_foc/lt_motor.h
)

set(EXAMPLE_SOURCES
    src/main.c
    src/protocol/ltm_commut.c
    src/protocol/protocol.c
)

add_executable(${PROJECT_NAME}.elf ${EXAMPLE_SOURCES})

target_link_options(${PROJECT_NAME}.elf PRIVATE
    -L ${LTM_BOARD_LINKER_DIR}
    -T ${LTM_BOARD_LINKER}
    -Wl,--gc-sections
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
)

target_link_libraries(${PROJECT_NAME}.elf
    -Wl,--whole-archive
        ${LTM_ROOT}/LTM_FOC/bin/libLTM_FOC.a
        ${LTM_HAL_SDK_DIR}/bin/libLTM_HAL.a
    -Wl,--no-whole-archive
    m
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary --remove-section=__option_setting_* $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_DIR}/${PROJECT_NAME}.bin
    COMMAND ${SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
)