# LTM_FOC 交付物生成
#   1) 自包含单头：<SRC_DIR>/common/lt_motor_types.h + <SRC_DIR>/motor/lt_motor.h
#                  -> <OUT_DIR>/ltm_foc/lt_motor.h
# 用法：
#   cmake -DSRC_DIR=<项目>/src -DOUT_DIR=<项目>/bin -P cmake/ltm_foc.cmake

if(NOT DEFINED SRC_DIR OR NOT DEFINED OUT_DIR)
    message(FATAL_ERROR "ltm_foc.cmake: 需要 SRC_DIR 与 OUT_DIR")
endif()

set(TYPES_H "${SRC_DIR}/common/lt_motor_types.h")
set(MOTOR_H "${SRC_DIR}/motor/lt_motor.h")
set(OUT_H   "${OUT_DIR}/ltm_foc/lt_motor.h")

file(READ "${TYPES_H}" TYPES_TEXT)
file(READ "${MOTOR_H}" MOTOR_TEXT)

# 1) 提取类型段正文：跳过 guard 首部
string(FIND "${TYPES_TEXT}" "#define LT_MOTOR_TYPES_H" DEF_POS)
if(DEF_POS LESS 0)
    message(FATAL_ERROR "ltm_foc.cmake: 未找到 #define LT_MOTOR_TYPES_H")
endif()
math(EXPR BODY0 "${DEF_POS} + 23")
string(SUBSTRING "${TYPES_TEXT}" ${BODY0} -1 TYPES_TAIL)
string(FIND "${TYPES_TAIL}" "\n" NL1)
if(NL1 LESS 0)
    message(FATAL_ERROR "ltm_foc.cmake: guard 后无换行")
endif()
math(EXPR BODY_START "${BODY0} + ${NL1} + 1")
string(SUBSTRING "${TYPES_TEXT}" ${BODY_START} -1 TYPES_BODY)

# 2) 截掉结尾 #endif
string(FIND "${TYPES_BODY}" "#endif" END_POS)
if(END_POS LESS 0)
    message(FATAL_ERROR "ltm_foc.cmake: 类型段缺 #endif")
endif()
string(SUBSTRING "${TYPES_BODY}" 0 ${END_POS} TYPES_BODY)
string(REGEX REPLACE "[ \t\r\n]+$" "" TYPES_BODY "${TYPES_BODY}")

# 3) 删掉 API 头里的内部 include
string(REGEX REPLACE "#include \"common/lt_motor_types.h\"[^\r\n]*\r?\n" "" MOTOR_TEXT "${MOTOR_TEXT}")

# 4) 在 API 段锚点前插入类型段
string(FIND "${MOTOR_TEXT}" "/* API 接口返回值" API_POS)
if(API_POS LESS 0)
    message(FATAL_ERROR "ltm_foc.cmake: 未找到 API 段锚点")
endif()
string(SUBSTRING "${MOTOR_TEXT}" 0 ${API_POS} HEAD_PART)
string(REGEX REPLACE "[ \t\r\n]+$" "" HEAD_PART "${HEAD_PART}")
string(REGEX REPLACE "^[ \t\r\n]+" "" TYPES_BODY "${TYPES_BODY}")
string(SUBSTRING "${MOTOR_TEXT}" ${API_POS} -1 TAIL_PART)

file(MAKE_DIRECTORY "${OUT_DIR}/ltm_foc")
file(WRITE "${OUT_H}" "${HEAD_PART}\n\n${TYPES_BODY}\n\n${TAIL_PART}")
message(STATUS "ltm_foc: 自包含单头 -> ${OUT_H}")
