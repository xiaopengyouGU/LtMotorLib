# boards/stm32g474/board.cmake —— 板级映射（STM32G474RBT3）
# HAL 是已经编好的底层驱动 SDK。
get_filename_component(LTM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set(LTM_FOC_BOARD        "stm32g474"                                        CACHE STRING   "板级配置")
set(LTM_HAL_SDK_DIR      "${LTM_ROOT}/LTM_HAL/ltm_hal_stm32g474_rbt3"       CACHE PATH     "该板使用的 LTM_HAL SDK 包")
set(LTM_BOARD_TOOLCHAIN  "${LTM_ROOT}/cmake/toolchain_arm_m4.cmake"         CACHE FILEPATH "该板的工具链（Cortex-M4F）")
set(LTM_BOARD_PARAM_H    "${CMAKE_CURRENT_LIST_DIR}/user_param_def.h"       CACHE FILEPATH "机型参数（编 FOC 库用）")
set(LTM_BOARD_LINKER     "${LTM_HAL_SDK_DIR}/config/stm32g474rbt3_flash.ld" CACHE FILEPATH "链接脚本")
set(LTM_BOARD_DEFINES    STM32G474RBT3 STM32G474xx USE_HAL_DRIVER           CACHE STRING   "该板的 MCU 编译定义")
set(LTM_BOARD_JLINK_DEVICE "STM32G474RB"                                   CACHE STRING   "J-Link device (flash)")
