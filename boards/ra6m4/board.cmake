# boards/ra6m4/board.cmake —— 板级映射
# HAL 是已经编好的 底层驱动 SDK。 
get_filename_component(LTM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set(LTM_FOC_BOARD        "ra6m4"                                          CACHE STRING   "板级配置")
set(LTM_HAL_SDK_DIR      "${LTM_ROOT}/LTM_HAL/ltm_hal_ra6m4"              CACHE PATH     "该板使用的 LTM_HAL SDK 包")
set(LTM_BOARD_TOOLCHAIN  "${LTM_ROOT}/cmake/toolchain_arm.cmake"          CACHE FILEPATH "该板的工具链")
set(LTM_BOARD_PARAM_H    "${CMAKE_CURRENT_LIST_DIR}/user_param_def.h"     CACHE FILEPATH "机型参数（编 FOC 库用）")
set(LTM_BOARD_LINKER     "${CMAKE_CURRENT_LIST_DIR}/linker/fsp.ld"        CACHE FILEPATH "链接脚本")
set(LTM_BOARD_DEFINES  _RENESAS_RA_ _RA_CORE=CM33 _RA_ORDINAL=1 CACHE STRING "该板的 MCU 编译定义")
