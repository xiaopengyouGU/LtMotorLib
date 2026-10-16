#===========================================================
# LTM_HAL 简单开环例程（独立构建）
# 依赖：board.cmake 指定的 HAL 包（${LTM_HAL_SDK_DIR}/bin 下的 libLTM_HAL.a + ltm_hal.h）
# 注意：SDK 静态库用 --whole-archive 链接，确保 IRQ 向量表（g_vector_table）
#       和所有 ISR 从归档中完整拉取（否则向量表缺失，中断无法启动）
# App 固定链接运行区 0x02008000（配合 BootLoader 运行区+备份区升级模型）
# bin 排除 __option_setting_*（芯片配置段），只含 App 固件，可直接 OTA
# 构建：cmake -G "MinGW Makefiles" -B build -DCMAKE_BUILD_TYPE=Release .
#       cmake --build build
#===========================================================
cmake_minimum_required(VERSION 3.20)

# 板级映射（HAL SDK 包、链接脚本、J-Link 器件名）：boards/<board>/board.cmake
get_filename_component(LTM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
if(NOT LTM_FOC_BOARD)
    set(LTM_FOC_BOARD "ra8t2")
endif()
include(${LTM_ROOT}/boards/${LTM_FOC_BOARD}/board.cmake)
get_filename_component(LTM_BOARD_LINKER_DIR "${LTM_BOARD_LINKER}" DIRECTORY)

# 公共组件源码：components/control、components/math、components/protocol
set(LTM_COMPONENTS ${LTM_ROOT}/components)

set(CMAKE_TOOLCHAIN_FILE ${LTM_HAL_SDK_DIR}/cmake/toolchain.cmake)   # HAL 包自带的工具链，与 SDK 库同源

project(ltm_hal_canfd VERSION 1.0.0 LANGUAGES C ASM)

set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})

add_compile_definitions(${LTM_BOARD_DEFINES} ${LTM_BOARD_EXAMPLE_DEFINES})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${LTM_HAL_SDK_DIR}/bin            # ltm_hal/ltm_hal.h（SDK 交付头）
    ${LTM_COMPONENTS}                # 公共组件源码
)

set(EXAMPLE_SOURCES
    src/main.c
    ${LTM_COMPONENTS}/protocol/ltm_commut.c
    ${LTM_COMPONENTS}/protocol/protocol.c
    ${LTM_COMPONENTS}/control/foc/lt_foc.c
    ${LTM_COMPONENTS}/math/basic/lt_math.c
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

# SDK 静态库：--whole-archive 强制全量链接（修复 IRQ 向量表缺失）
target_link_libraries(${PROJECT_NAME}.elf
    -Wl,--whole-archive ${LTM_HAL_SDK_DIR}/bin/libLTM_HAL.a -Wl,--no-whole-archive
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary --remove-section=__option_setting_* $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_DIR}/${PROJECT_NAME}.bin
    COMMAND ${SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
)
