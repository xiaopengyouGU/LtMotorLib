# PC（本机 gcc）工具链：不交叉编译，只固定编译器
# 用法：-DCMAKE_TOOLCHAIN_FILE=cmake/toolchain.cmake
# gcc 不在 PATH 时：-DLTM_PC_TOOLCHAIN_DIR=D:/xxx/bin

set(LTM_PC_TOOLCHAIN_DIR "" CACHE PATH "本机 gcc 所在目录；留空则从 PATH 查找")
if(LTM_PC_TOOLCHAIN_DIR)
    set(_LTM_PC_FIND PATHS "${LTM_PC_TOOLCHAIN_DIR}")
else()
    set(_LTM_PC_FIND)
endif()

find_program(CMAKE_C_COMPILER   NAMES gcc    ${_LTM_PC_FIND})
find_program(CMAKE_CXX_COMPILER NAMES g++    ${_LTM_PC_FIND})
find_program(CMAKE_AR           NAMES ar     ${_LTM_PC_FIND})
find_program(CMAKE_RANLIB       NAMES ranlib ${_LTM_PC_FIND})

if(NOT CMAKE_C_COMPILER)
    message(FATAL_ERROR "找不到 gcc：加进 PATH，或用 -DLTM_PC_TOOLCHAIN_DIR=D:/xxx/bin")
endif()

message(STATUS "LTM_CTRL PC: ${CMAKE_C_COMPILER}")