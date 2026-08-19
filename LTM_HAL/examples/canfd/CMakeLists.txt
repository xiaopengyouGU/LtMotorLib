#===========================================================
# LTM_HAL 简单开环例程（独立构建）
# 依赖：lib/libLTM_HAL.a + lib/ltm_hal/ltm_hal.h（对接不同平台只需替换 lib/ 内容）
# 注意：SDK 静态库用 --whole-archive 链接，确保 IRQ 向量表（g_vector_table）
#       和所有 ISR 从归档中完整拉取（否则向量表缺失，中断无法启动）
# App 固定链接运行区 0x02008000（配合 BootLoader 运行区+备份区升级模型）
# bin 排除 __option_setting_*（芯片配置段），只含 App 固件，可直接 OTA
# 构建：cmake -G "MinGW Makefiles" -B build -DCMAKE_BUILD_TYPE=Release .
#       cmake --build build
#===========================================================
cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../ltm_hal_ra8t2_CPU0/cmake/toolchain.cmake)

project(ltm_hal_canfd VERSION 1.0.0 LANGUAGES C ASM)

set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})

add_compile_definitions(
    _RENESAS_RA_
    _RA_CORE=CPU0
    _RA_ORDINAL=1
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

set(EXAMPLE_SOURCES
    src/main.c
    src/protocol/ltm_commut.c
    src/protocol/protocol.c
    src/control/foc/lt_foc.c
    src/math/basic/lt_math.c
)

add_executable(${PROJECT_NAME}.elf ${EXAMPLE_SOURCES})

target_link_options(${PROJECT_NAME}.elf PRIVATE
    -L ${CMAKE_CURRENT_SOURCE_DIR}/linker
    -T ${CMAKE_CURRENT_SOURCE_DIR}/linker/fsp.ld
    -Wl,--gc-sections
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
)

# SDK 静态库：--whole-archive 强制全量链接（修复 IRQ 向量表缺失）
target_link_libraries(${PROJECT_NAME}.elf
    -Wl,--whole-archive ${CMAKE_CURRENT_SOURCE_DIR}/lib/libLTM_HAL.a -Wl,--no-whole-archive
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary --remove-section=__option_setting_* $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_DIR}/${PROJECT_NAME}.bin
    COMMAND ${SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
)
