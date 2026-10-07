# LTM_CTRL：纯 C 电机控制算法库
# 顶层构建：libLTM_CTRL.a + 聚合头 -> bin/<platform>/
# 作为子项目（add_subdirectory）：只提供 ltm_ctrl_objects，不做交付

cmake_minimum_required(VERSION 3.20)

project(LTM_CTRL VERSION 1.0.1 LANGUAGES C)

set(LTM_CTRL_TOP_LEVEL OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(LTM_CTRL_TOP_LEVEL ON)
endif()

# 产物目录名：arm / riscv / pc（仅顶层构建需要）
if(LTM_CTRL_TOP_LEVEL AND NOT DEFINED LTM_CTRL_PLATFORM)
    if(NOT CMAKE_CROSSCOMPILING)
        set(_LTM_PLATFORM_DEFAULT "pc")
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "riscv")
        set(_LTM_PLATFORM_DEFAULT "riscv")
    else()
        set(_LTM_PLATFORM_DEFAULT "arm")
    endif()
    set(LTM_CTRL_PLATFORM "${_LTM_PLATFORM_DEFAULT}" CACHE STRING "产物目录名：arm / riscv / pc")
    unset(_LTM_PLATFORM_DEFAULT)
endif()

set(SRC_DIR        ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PUBLIC_HEADERS lt_math.h lt_control.h lt_analysis.h)

set(LTMCTRL_VERSION "${PROJECT_VERSION}" CACHE STRING "LTM_CTRL 版本号")

file(GLOB_RECURSE LIB_SOURCES
    ${SRC_DIR}/math/*.c
    ${SRC_DIR}/control/*.c
    ${SRC_DIR}/analysis/*.c
)
if(NOT LIB_SOURCES)
    message(FATAL_ERROR "没找到源文件：${SRC_DIR}/{math,control,analysis}")
endif()

# 聚合头按交付布局 ltm_ctrl/lt_*.h 摆进构建目录，树内下游与吃 bin/ 时写法一致
foreach(h ${PUBLIC_HEADERS})
    configure_file(${SRC_DIR}/${h} ${CMAKE_CURRENT_BINARY_DIR}/include/ltm_ctrl/${h} COPYONLY)
endforeach()

# 对象库：顶层用它归档成 libLTM_CTRL.a；LTM_FOC 直接并进自己的静态库，
# 这样 CTRL 源码一改，FOC 的归档就跟着重建，不需要另外的合并步骤。
add_library(ltm_ctrl_objects OBJECT ${LIB_SOURCES})
target_include_directories(ltm_ctrl_objects
    PUBLIC  ${CMAKE_CURRENT_BINARY_DIR}/include
    PRIVATE ${SRC_DIR}
)
target_compile_definitions(ltm_ctrl_objects PUBLIC LTMCTRL_VERSION_STRING="${LTMCTRL_VERSION}")

set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_C_FLAGS_DEBUG "-g -O2")

if(LTM_CTRL_TOP_LEVEL)
    set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)
    get_filename_component(LTM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

    add_library(LTM_CTRL STATIC $<TARGET_OBJECTS:ltm_ctrl_objects>)
    set_target_properties(LTM_CTRL PROPERTIES OUTPUT_NAME LTM_CTRL)

    add_custom_target(LTM_CTRL_deliver ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory ${BIN_DIR}/${LTM_CTRL_PLATFORM}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:LTM_CTRL> ${BIN_DIR}/${LTM_CTRL_PLATFORM}/libLTM_CTRL.a
        COMMAND ${CMAKE_COMMAND} -DSRC_DIR=${SRC_DIR} -DOUT_DIR=${BIN_DIR}/${LTM_CTRL_PLATFORM}
                -P ${LTM_ROOT}/cmake/ltm_ctrl.cmake
        DEPENDS LTM_CTRL
        COMMENT "LTM_CTRL(${LTM_CTRL_PLATFORM}) release -> bin/"
    )
endif()