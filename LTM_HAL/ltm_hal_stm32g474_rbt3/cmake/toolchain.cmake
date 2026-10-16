#===========================================================
# LTM_HAL - STM32G474xx ARM 交叉编译工具链
#===========================================================

cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 指定交叉编译工具链
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)  # 修正此行
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)

# 对于FPU的定义，使用 add_compile_options 而不是 add_definitions
add_compile_options(-DARM_MATH_CM4)
# 添加必要的编译定义
add_compile_definitions(
    STM32G474xx
    USE_HAL_DRIVER
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

# 硬件浮点支持（STM32G4支持双精度浮点）
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)

# 软件浮点（备选）
# add_compile_options(-mfloat-abi=soft)

# 核心编译选项
add_compile_options(
    -mcpu=cortex-m4
    -mthumb
    -mthumb-interwork
    -ffunction-sections
    -fdata-sections
    -fno-common
    -fmessage-length=0
)

# NDEBUG：禁 assert，裁掉 newlib fiprintf/_vfiprintf_r 链（省 ~7KB）
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_C_FLAGS_DEBUG "-g -O2")
# 警告选项
add_compile_options(-Wall -Wextra)
