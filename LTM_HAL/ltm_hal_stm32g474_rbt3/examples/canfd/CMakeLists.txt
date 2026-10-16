#===========================================================
# LTM_HAL 简单开环例程（独立构建）
# 依赖：lib/libLTM_HAL.a + lib/ltm_hal/ltm_hal.h（对接不同平台只需替换 lib/ 内容）
# 注意：SDK 静态库用 --whole-archive 链接，确保 IRQ 向量表（default_isr_*）和
#       所有 ISR 包装器从归档中完整拉取（否则向量表缺失，中断无法启动）
# App 链接 config/stm32g474rbt3_flash.ld：Flash 0x08000000 起 128KB；SRAM 96KB + CCM 32KB（栈放 CCM）
# 构建：cmake -G "MinGW Makefiles" -B build -DCMAKE_BUILD_TYPE=Release .
#       cmake --build build
#===========================================================
cmake_minimum_required(VERSION 3.20)

# 公共组件源码：components/control、components/math、components/protocol
get_filename_component(LTM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(LTM_COMPONENTS ${LTM_ROOT}/components)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/toolchain.cmake)

project(ltm_hal_canfd VERSION 1.0.0 LANGUAGES C ASM)

set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
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
    -T ${CMAKE_CURRENT_SOURCE_DIR}/../../config/stm32g474rbt3_flash.ld
    -nostartfiles
    -static
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
)

# SDK 静态库：--whole-archive 强制全量链接（修复 IRQ 向量表缺失）
target_link_libraries(${PROJECT_NAME}.elf
    -Wl,--whole-archive ${CMAKE_CURRENT_SOURCE_DIR}/lib/libLTM_HAL.a -Wl,--no-whole-archive
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_DIR}/${PROJECT_NAME}.bin
    COMMAND ${SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
)
