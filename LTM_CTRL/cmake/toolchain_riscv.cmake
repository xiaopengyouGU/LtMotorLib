# RISC-V (riscv32-unknown-elf-gcc) 交叉编译工具链：只约定 ISA / ABI / 寻址模型
# 用法：-DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_riscv.cmake
# 编译器不在 PATH 时：-DLTM_RISCV_TOOLCHAIN_DIR=D:/xxx/bin

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv)

set(LTM_RISCV_TOOLCHAIN_DIR "" CACHE PATH "RISC-V 工具链 bin 目录；留空则从 PATH 查找")
if(LTM_RISCV_TOOLCHAIN_DIR)
    set(_LTM_RISCV_FIND PATHS "${LTM_RISCV_TOOLCHAIN_DIR}")
else()
    set(_LTM_RISCV_FIND)
endif()

find_program(CMAKE_C_COMPILER   NAMES riscv32-unknown-elf-gcc     ${_LTM_RISCV_FIND})
find_program(CMAKE_CXX_COMPILER NAMES riscv32-unknown-elf-g++     ${_LTM_RISCV_FIND})
find_program(CMAKE_ASM_COMPILER NAMES riscv32-unknown-elf-gcc     ${_LTM_RISCV_FIND})
find_program(CMAKE_AR           NAMES riscv32-unknown-elf-ar      ${_LTM_RISCV_FIND})
find_program(CMAKE_OBJCOPY      NAMES riscv32-unknown-elf-objcopy ${_LTM_RISCV_FIND})
find_program(CMAKE_OBJDUMP      NAMES riscv32-unknown-elf-objdump ${_LTM_RISCV_FIND})
find_program(SIZE               NAMES riscv32-unknown-elf-size    ${_LTM_RISCV_FIND})

if(NOT CMAKE_C_COMPILER)
    message(FATAL_ERROR "找不到 riscv32-unknown-elf-gcc：加进 PATH，或用 -DLTM_RISCV_TOOLCHAIN_DIR=D:/xxx/bin")
endif()

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# zicsr/zifencei 需显式声明（GCC 12+ 不隐含）；优化级别由工程侧 CMAKE_C_FLAGS_<CONFIG> 决定
set(LTM_RISCV_ARCH_FLAGS "-march=rv32imafc_zicsr_zifencei -mabi=ilp32f -mcmodel=medany -msmall-data-limit=8"
    CACHE STRING "ISA / ABI / 寻址模型")
set(ARCH_FLAGS "${LTM_RISCV_ARCH_FLAGS}")
set(COMMON_FLAGS "${ARCH_FLAGS} -ffunction-sections -fdata-sections -fmessage-length=0 -fsigned-char -MMD -MP")

set(CMAKE_C_FLAGS_INIT   "${COMMON_FLAGS} -std=gnu99")
set(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")

message(STATUS "LTM_CTRL RISC-V: ${ARCH_FLAGS}")