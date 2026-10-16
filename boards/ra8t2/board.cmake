# boards/ra8t2/board.cmake —— 板级映射
# HAL 是已经编好的底层驱动 SDK。
get_filename_component(LTM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set(LTM_FOC_BOARD        "ra8t2"                                     CACHE STRING   "板级配置")
set(LTM_HAL_SDK_DIR      "${LTM_ROOT}/LTM_HAL/ltm_hal_ra8t2_CPU0"    CACHE PATH     "该板使用的 LTM_HAL SDK 包")
set(LTM_BOARD_TOOLCHAIN  "${LTM_HAL_SDK_DIR}/cmake/toolchain.cmake"  CACHE FILEPATH "该板的工具链")
set(LTM_BOARD_LINKER     "${CMAKE_CURRENT_LIST_DIR}/linker/fsp.ld"   CACHE FILEPATH "链接脚本")
set(LTM_BOARD_DEFINES  _RENESAS_RA_ _RA_CORE=CPU0 _RA_ORDINAL=1      CACHE STRING   "该板的 MCU 编译定义")
set(LTM_BOARD_JLINK_DEVICE "R7KA8T2LF_CPU0"                          CACHE STRING   "J-Link device (flash)")
set(LTM_BOARD_JLINK_SPEED  1000                                      CACHE STRING   "J-Link SWD 时钟 (kHz)")
set(LTM_BOARD_EXAMPLE_DEFINES POLE_PAIRS=10 ENCODER_ZERO_OFFSET=203609.0f  CACHE STRING "例程机型参数（覆盖 POLE_PAIRS 等）")
# 注：该板暂无 LTM_FOC 机型参数（boards/ra8t2/user_param_def.h），仅供 LTM_HAL 例程使用
