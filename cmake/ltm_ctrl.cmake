# LTM_CTRL 对外聚合头生成
# 用法：cmake -DSRC_DIR=<项目>/src -DOUT_DIR=<项目>/bin/<platform> -P cmake/ltm_ctrl.cmake
# 产出：<OUT_DIR>/ltm_ctrl/{lt_math.h, lt_control.h, lt_analysis.h}

set(LTM_CTRL_PUBLIC_HEADERS lt_math.h lt_control.h lt_analysis.h)

if(NOT DEFINED SRC_DIR OR NOT DEFINED OUT_DIR)
    message(FATAL_ERROR "ltm_ctrl.cmake: 需要 SRC_DIR 与 OUT_DIR")
endif()

file(MAKE_DIRECTORY "${OUT_DIR}/ltm_ctrl")
foreach(h ${LTM_CTRL_PUBLIC_HEADERS})
    if(NOT EXISTS "${SRC_DIR}/${h}")
        message(FATAL_ERROR "ltm_ctrl.cmake: 缺文件 ${SRC_DIR}/${h}")
    endif()
    configure_file("${SRC_DIR}/${h}" "${OUT_DIR}/ltm_ctrl/${h}" COPYONLY)
endforeach()
message(STATUS "ltm_ctrl: 聚合头 -> ${OUT_DIR}/ltm_ctrl/")