# LTM_CTRL PC 仿真与主机侧压测（只在本机编译，不参与交叉构建）
cmake_minimum_required(VERSION 3.20)

project(ltm_ctrl_example LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)

set(LTM_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src)

file(GLOB_RECURSE LTM_SOURCES
    ${LTM_SRC}/math/*.c ${LTM_SRC}/control/*.c ${LTM_SRC}/analysis/*.c)

add_library(ltm_ctrl_host STATIC ${LTM_SOURCES})
target_include_directories(ltm_ctrl_host
    PUBLIC  ${CMAKE_CURRENT_BINARY_DIR}/include
    PRIVATE ${LTM_SRC})
foreach(h lt_math.h lt_control.h lt_analysis.h)
    configure_file(${LTM_SRC}/${h} ${CMAKE_CURRENT_BINARY_DIR}/include/ltm_ctrl/${h} COPYONLY)
endforeach()

add_executable(sim main.c)
target_link_libraries(sim PRIVATE ltm_ctrl_host m)

# 压测把 lt_speed 单独编进来，这样能打开 M 法通道与 PLL 对比
add_executable(speed_stress
    stress/lt_speed_stress.c
    ${LTM_SRC}/control/speed/lt_speed.c
    ${LTM_SRC}/math/basic/lt_math.c)
target_include_directories(speed_stress PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/include ${LTM_SRC})
target_compile_definitions(speed_stress PRIVATE LT_SPEED_USE_ADAP_M=1)
target_link_libraries(speed_stress PRIVATE m)