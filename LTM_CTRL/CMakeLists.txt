# LTM_CTRL：纯 C 电机控制算法库
# 用法：cmake -B <build_dir> [-DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_arm.cmake|cmake/toolchain_riscv.cmake] -DCMAKE_BUILD_TYPE=Release .

cmake_minimum_required(VERSION 3.20)

project(LTM_CTRL VERSION 1.0.1 LANGUAGES C)

# 产物目录名：arm / riscv / pc
if(NOT DEFINED LTM_CTRL_PLATFORM)
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
set(BIN_DIR        ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(PUBLIC_HEADERS lt_math.h lt_control.h lt_analysis.h)

set(LTMCTRL_VERSION "${PROJECT_VERSION}" CACHE STRING "LTM_CTRL 版本号")

# 构建目录本身就是 build/<platform>/，静态库默认就落在里面

file(GLOB_RECURSE LIB_SOURCES
    ${SRC_DIR}/math/*.c
    ${SRC_DIR}/control/*.c
    ${SRC_DIR}/analysis/*.c
)
if(NOT LIB_SOURCES)
    message(FATAL_ERROR "没找到源文件：${SRC_DIR}/{math,control,analysis}")
endif()

add_library(${PROJECT_NAME} STATIC ${LIB_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# 聚合头按交付布局 ltm_ctrl/lt_*.h 摆进构建目录，树内下游与吃 bin/ 时写法一致
foreach(h ${PUBLIC_HEADERS})
    configure_file(${SRC_DIR}/${h} ${CMAKE_CURRENT_BINARY_DIR}/include/ltm_ctrl/${h} COPYONLY)
endforeach()

target_include_directories(${PROJECT_NAME}
    PUBLIC  ${CMAKE_CURRENT_BINARY_DIR}/include
    PRIVATE ${SRC_DIR}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC LTMCTRL_VERSION_STRING="${LTMCTRL_VERSION}")

set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_C_FLAGS_DEBUG "-g -O2")

add_custom_target(${PROJECT_NAME}_deliver ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory ${BIN_DIR}/${LTM_CTRL_PLATFORM}/ltm_ctrl
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:${PROJECT_NAME}> ${BIN_DIR}/${LTM_CTRL_PLATFORM}/libLTM_CTRL.a
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SRC_DIR}/lt_math.h     ${BIN_DIR}/${LTM_CTRL_PLATFORM}/ltm_ctrl/lt_math.h
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SRC_DIR}/lt_control.h  ${BIN_DIR}/${LTM_CTRL_PLATFORM}/ltm_ctrl/lt_control.h
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SRC_DIR}/lt_analysis.h ${BIN_DIR}/${LTM_CTRL_PLATFORM}/ltm_ctrl/lt_analysis.h
    DEPENDS ${PROJECT_NAME}
    COMMENT "LTM_CTRL(${LTM_CTRL_PLATFORM}) release -> bin/"
)