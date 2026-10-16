#===========================================================
# LTM_HAL —— STM32G474xx (静态库)
# 对外交付：libLTM_HAL.a + ltm_hal/ltm_hal.h（bin/ 目录）
# 硬件层：STM32 G4 HAL库驱动（已内置到 config，平台自包含）
# 构建：python script.py -s -v x.y.z
#===========================================================
cmake_minimum_required(VERSION 3.20)

# ========== 全局编译宏定义 ==========
add_compile_definitions(
    STM32G474RBT3
    USE_FULL_LL_DRIVER
)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain.cmake)

project(LTM_HAL VERSION 1.0.0 LANGUAGES C ASM)

set(LTM_HAL_VERSION "${PROJECT_VERSION}" CACHE STRING "LTM_HAL 版本号")

# STM32G4 库路径
set(CMSIS_DIR          ${CMAKE_CURRENT_SOURCE_DIR}/config/CMSIS)
set(HAL_DIR            ${CMAKE_CURRENT_SOURCE_DIR}/config/STM32G4xx_HAL_Driver)
set(HAL_DIR_SRC        ${HAL_DIR}/Src)
set(CMSIS_STARTUP_DIR  ${CMSIS_DIR}/Device/ST/STM32G4xx/Source/Templates/gcc)
set(CMSIS_SYSTEM_DIR   ${CMSIS_DIR}/Device/ST/STM32G4xx/Source/Templates)
set(LINKER_SCRIPT      ${CMAKE_CURRENT_SOURCE_DIR}/config/stm32g474rbt3_flash.ld)

# 输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# ========== 头文件路径 ==========
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers
    ${CMSIS_DIR}/Include
    ${CMSIS_DIR}/Device/ST/STM32G4xx/Include
    ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy
)

# ========== STM32G4 HAL 驱动源文件 ==========
set(STM32G4_SOURCES
    # CMSIS 启动文件
    ${CMSIS_STARTUP_DIR}/startup_stm32g474xx.s

    # 系统文件
    ${CMSIS_SYSTEM_DIR}/system_stm32g4xx.c

    # HAL 驱动核心
    ${HAL_DIR_SRC}/stm32g4xx_hal.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_cortex.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_dma.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_dma_ex.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_exti.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_flash.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_flash_ex.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_gpio.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_pwr.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_pwr_ex.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_rcc.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_rcc_ex.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_spi.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_tim.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_tim_ex.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_uart.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_uart_ex.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_adc.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_adc_ex.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_comp.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_opamp.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_opamp_ex.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_hrtim.c
    ${HAL_DIR_SRC}/stm32g4xx_hal_fdcan.c

    # LL 驱动源文件（drv_spi.c 使用 LL_SPI_Init/LL_DMA_Init，非内联，需随库编译）
    ${HAL_DIR_SRC}/stm32g4xx_ll_spi.c
    ${HAL_DIR_SRC}/stm32g4xx_ll_dma.c
    ${HAL_DIR_SRC}/stm32g4xx_ll_rcc.c
)


# ========== LTM_HAL 源：驱动  ==========
file(GLOB_RECURSE SDK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components/ltm_hal/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/startup/*.s
)

add_library(${PROJECT_NAME} STATIC ${SDK_SOURCES} ${STM32G4_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_compile_definitions(${PROJECT_NAME} PUBLIC LTM_HAL_VERSION_STRING="${LTM_HAL_VERSION}")

# ========== 发布产物 -> bin/（最新版 + 版本存档） ==========
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
    COMMENT "LTM_HAL(STM32G474RBT3) release artifacts -> ${BIN_DIR}/"
)
