#===========================================================
# LTM_HAL —— RA8T2/CPU0 静态库
# 对外交付：libLTM_HAL.a + ltm_hal/ltm_hal.h（bin/ 目录）
# 版本化：bin/v<version>/ 下留档每次发布
# 构建：python script.py -s -v x.y.z
# 注意：源码用 glob 全量纳入 drivers（显式裁剪会使弱引用向量表中的 ISR 不被拉取，中断起不来）
#===========================================================
cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain.cmake)

project(LTM_HAL VERSION 1.0.0 LANGUAGES C ASM)

# 版本号：命令行可用 -DLTM_HAL_VERSION=x.y.z 覆盖（script.py -v 一键生成不同版本）
set(LTM_HAL_VERSION "${PROJECT_VERSION}" CACHE STRING "LTM_HAL 版本号")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_compile_definitions(
    _RENESAS_RA_
    _RA_CORE=CPU0
    _RA_ORDINAL=1
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/hardware
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/hardware/ra_gen
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/hardware/ra/fsp/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/hardware/ra/fsp/inc/api
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/hardware/ra/fsp/inc/instances
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/hardware/ra_cfg/fsp_cfg
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/hardware/ra_cfg/fsp_cfg/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/hardware/ra/arm/CMSIS_6/CMSIS/Core/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/hardware/ra/fsp/src/bsp/cmsis/Device/RENESAS/Include
)

# ========== 库源码：glob 全量纳入 drivers + ltm_hal（main 归应用工程，不进库） ==========
file(GLOB_RECURSE SDK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components/ltm_hal/*.c
)
list(FILTER SDK_SOURCES EXCLUDE REGEX ".*/ra_gen/main\\.c$")

add_library(${PROJECT_NAME} STATIC ${SDK_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# 版本号编译宏：LTM_HAL_VERSION_STRING 展开为 "x.y.z"
target_compile_definitions(${PROJECT_NAME} PUBLIC LTM_HAL_VERSION_STRING="${LTM_HAL_VERSION}")

# Release：-O2，不带 -g（发布库完全隔离）；Debug：内部开发用 -g -O2
set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_C_FLAGS_DEBUG "-g -O2")

# 发布物：最新版 → bin/，版本留档 → bin/v<version>/
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${BIN_DIR}/ltm_hal
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:${PROJECT_NAME}> ${BIN_DIR}/libLTM_HAL.a
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/src/components/ltm_hal/ltm_hal.h
            ${BIN_DIR}/ltm_hal/ltm_hal.h
    COMMAND ${CMAKE_COMMAND} -E make_directory ${BIN_DIR}/v${LTM_HAL_VERSION}/ltm_hal
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:${PROJECT_NAME}> ${BIN_DIR}/v${LTM_HAL_VERSION}/libLTM_HAL.a
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/src/components/ltm_hal/ltm_hal.h
            ${BIN_DIR}/v${LTM_HAL_VERSION}/ltm_hal/ltm_hal.h
    COMMENT "LTM_HAL release artifacts -> ${BIN_DIR}/ + v${LTM_HAL_VERSION}/"
)