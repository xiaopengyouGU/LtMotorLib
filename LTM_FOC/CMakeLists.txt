#===========================================================
# LTM_FOC —— 胶水层 SDK：FOC 控制逻辑 + 状态机 + 任务调度
# 依赖：LTM_HAL（ltm_hal/ltm_hal.h）  LTM_CTRL（ltm_ctrl/lt_*.h）
# 对外交付：libLTM_FOC.a（已内含 LTM_CTRL 算法）+ ltm_foc/lt_motor.h（自包含单头）
# 平台：RA6M4（Cortex-M33，ARM，工具链见 cmake/toolchain.cmake）
# 一键：python script.py -s / -c
#===========================================================
cmake_minimum_required(VERSION 3.20)

get_filename_component(LTM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

# 板级映射：HAL SDK 目录 / 工具链 / 链接脚本的唯一出处。必须在 project() 之前选定
# 工具链——否则换板后仍按固定档位编指令集（M33 的代码在 M4F 上非法）。
if(NOT LTM_BOARD)
    message(FATAL_ERROR "未指定板级配置：-DLTM_BOARD=<board>（现有：ra6m4、stm32g474）")
endif()
set(LTM_BOARD_CMAKE "${LTM_ROOT}/boards/${LTM_BOARD}/board.cmake")
if(NOT EXISTS "${LTM_BOARD_CMAKE}")
    message(FATAL_ERROR "缺少板级映射 ${LTM_BOARD_CMAKE}（需定义 LTM_HAL_SDK_DIR / LTM_BOARD_TOOLCHAIN）")
endif()
include("${LTM_BOARD_CMAKE}")
set(CMAKE_TOOLCHAIN_FILE ${LTM_BOARD_TOOLCHAIN})

project(LTM_FOC VERSION 1.0.0 LANGUAGES C)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(LTMFOC_VERSION "${PROJECT_VERSION}" CACHE STRING "LTM_FOC 版本号")

# 机型参数：板级 user_param_def.h（路径由 board.cmake 给出）
set(LTM_FOC_PARAM_H "${LTM_BOARD_PARAM_H}")
if(NOT EXISTS "${LTM_FOC_PARAM_H}")
    message(FATAL_ERROR "找不到板级参数文件：${LTM_FOC_PARAM_H}")
endif()
message(STATUS "LTM_FOC 板级: ${LTM_BOARD}")

# LTM_CTRL 以子项目方式并入：同一构建图、同一工具链，改 CTRL 源码即触发重建
add_subdirectory(${LTM_ROOT}/LTM_CTRL ${CMAKE_BINARY_DIR}/ltm_ctrl)

file(GLOB_RECURSE LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
add_library(${PROJECT_NAME} STATIC ${LIB_SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE ltm_ctrl_objects)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${LTM_HAL_SDK_DIR}/bin
)

target_compile_definitions(${PROJECT_NAME} PUBLIC LTMFOC_VERSION_STRING="${LTMFOC_VERSION}")
target_compile_definitions(${PROJECT_NAME} PRIVATE LTM_FOC_PARAM_H="${LTM_FOC_PARAM_H}")

set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_C_FLAGS_DEBUG "-g -O2")

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:${PROJECT_NAME}> ${BIN_DIR}/libLTM_FOC.a
    COMMAND ${CMAKE_COMMAND}
        -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}/src
        -DOUT_DIR=${BIN_DIR}
        -P ${LTM_ROOT}/cmake/ltm_foc.cmake
    COMMENT "LTM_FOC: 交付单头 -> bin/ltm_foc/"
)
