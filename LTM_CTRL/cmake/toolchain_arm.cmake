# ARM (arm-none-eabi-gcc) 交叉编译工具链：只约定指令集与浮点 ABI，不管芯片型号
# 用法：-DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_arm.cmake
# 编译器不在 PATH 时：-DLTM_ARM_TOOLCHAIN_DIR=D:/xxx/bin

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(LTM_ARM_TOOLCHAIN_DIR "" CACHE PATH "ARM 工具链 bin 目录；留空则从 PATH 查找")
if(LTM_ARM_TOOLCHAIN_DIR)
    set(_LTM_ARM_FIND PATHS "${LTM_ARM_TOOLCHAIN_DIR}")
else()
    set(_LTM_ARM_FIND)
endif()

find_program(CMAKE_C_COMPILER   NAMES arm-none-eabi-gcc     ${_LTM_ARM_FIND})
find_program(CMAKE_CXX_COMPILER NAMES arm-none-eabi-g++     ${_LTM_ARM_FIND})
find_program(CMAKE_ASM_COMPILER NAMES arm-none-eabi-gcc     ${_LTM_ARM_FIND})
find_program(CMAKE_AR           NAMES arm-none-eabi-ar      ${_LTM_ARM_FIND})
find_program(CMAKE_OBJCOPY      NAMES arm-none-eabi-objcopy ${_LTM_ARM_FIND})
find_program(CMAKE_OBJDUMP      NAMES arm-none-eabi-objdump ${_LTM_ARM_FIND})
find_program(SIZE               NAMES arm-none-eabi-size    ${_LTM_ARM_FIND})

if(NOT CMAKE_C_COMPILER)
    message(FATAL_ERROR "找不到 arm-none-eabi-gcc：加进 PATH，或用 -DLTM_ARM_TOOLCHAIN_DIR=D:/xxx/bin")
endif()

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# M4/M7 同一套：ARMv7E-M + 单精度硬浮点
set(LTM_ARM_ARCH_FLAGS "-mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
    CACHE STRING "指令集与浮点 ABI")
set(ARCH_FLAGS "${LTM_ARM_ARCH_FLAGS}")

set(WARNING_FLAGS "-Wall -Wextra -Wunused -Wuninitialized -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal")
set(COMMON_FLAGS "${ARCH_FLAGS} ${WARNING_FLAGS} -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -MMD -MP")

set(CMAKE_C_FLAGS_INIT   "${COMMON_FLAGS} -std=c99")
set(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -std=c++11")
set(CMAKE_ASM_FLAGS_INIT "${ARCH_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${ARCH_FLAGS} -Wl,--gc-sections -Wl,--print-memory-usage")

if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 12.2)
    add_compile_options(
        --param=min-pagesize=0
        -Wno-format-truncation
        -Wno-stringop-overflow
    )
endif()

message(STATUS "LTM_CTRL ARM: ${ARCH_FLAGS}")