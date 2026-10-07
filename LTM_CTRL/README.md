# LTM_CTRL 电机控制算法库

LTM_CTRL 采用 纯 C 实现，无动态内存分配、无文件与控制台 I/O、无硬件依赖；同一套源码可交叉编译到 ARM 与 RISC-V，也可在本机编译用于仿真与测试。

当前版本 1.0.1。

## 目录结构

```
LTM_CTRL/
├─ CMakeLists.txt              顶层构建
├─ script.py                   一键构建脚本
├─ cmake/
│   ├─ toolchain_arm.cmake     ARM 交叉工具链
│   ├─ toolchain_riscv.cmake   RISC-V 交叉工具链
│   └─ toolchain.cmake         本机（PC）工具链
├─ src/
│   ├─ lt_math.h               对外聚合头
│   ├─ lt_control.h
│   ├─ lt_analysis.h
│   ├─ math/                   基础数学、相干解调、插值、最小二乘
│   ├─ control/                纯定点：FOC、PID、PDOB、S 曲线、测速、死区补偿、陷波
│   └─ analysis/               参数校准、辨识、扫频测量、激励信号
├─ examples/                   PC 仿真示例
├─ build/<platform>/           构建目录，`-c` 清理
└─ bin/<platform>/             交付产物
```

## 模块


| 模块             | 内容                                                       | 说明                                           |
| ---------------- | ---------------------------------------------------------- | ---------------------------------------------- |
| math/basic       | `lt_math` `lt_q15` `lt_q24`                                | 三角/开方/统计函数，Q15、Q24 定点运算          |
| math/demod       | `lt_demod`                                                 | 相干解调，输出幅值与相位                       |
| math/interp      | `lt_interp`                                                | 一维线性、双线性、三次 Hermite 插值            |
| math/lsq         | `lt_lsq`                                                   | 最小二乘线性回归，最多 3 个自变量              |
| control/foc      | `lt_foc`                                                   | 定点 FOC：Clark/Park 变换与三相占空比计算      |
| control/pid      | `lt_pid`                                                   | 增量式 PI/PD/PID，支持 Q15/Q24，静态实例池     |
| control/pdob     | `lt_pdob`                                                  | 速度环 DOB+P、电流环无差拍 DPCC/ESO、弱磁      |
| control/scurve   | `lt_scurve`                                                | 5 段 S 型/T 型曲线规划，静态实例池             |
| control/speed    | `lt_speed`                                                 | 锁相环测速（count/s），可选用自适应 M 法       |
| control/deadzone | `lt_deadzone`                                              | 电流极性法死区补偿                             |
| control/filter   | `lt_notch`                                                 | 三级陷波滤波器，机械谐振抑制                   |
| analysis/calib   | `lt_calib_R` `lt_calib_L` `lt_calib_pp` `lt_calib_encoder` | 电阻、电感、极对数与编码器方向、编码器零点校准 |
| analysis/ident   | `lt_cogging` `lt_friction` `lt_flux` `lt_inertia`          | 齿槽转矩、摩擦、磁链、惯量辨识                 |
| analysis/meas    | `lt_meas`                                                  | 环路扫频测量                                   |
| analysis/excit   | `lt_excit`                                                 | 阶跃、方波、三角波、正弦激励                   |

对外只提供三个聚合头 `lt_math.h`、`lt_control.h`、`lt_analysis.h`，模块内部头文件不参与交付。控制主链以 Q15/Q24 定点为主，`analysis` 模块以 float 为主。

## 环境


| 用途   | 工具链                          | 已验证版本 |
| ------ | ------------------------------- | ---------- |
| ARM    | `arm-none-eabi-gcc`             | 14.2.1     |
| RISC-V | `riscv32-unknown-elf-gcc`       | 13.2.0     |
| PC     | MinGW`gcc`（x86_64）            | 11.2.0     |
| 构建   | CMake（≥ 3.20）+`mingw32-make` | 4.1.4      |

`script.py` 固定使用 `MinGW Makefiles` 生成器，编译器都从 `PATH` 查找。工具链装在非默认位置时，用 CMake 变量指定其 `bin` 目录，变量名为 `LTM_ARM_TOOLCHAIN_DIR`、`LTM_RISCV_TOOLCHAIN_DIR`、`LTM_PC_TOOLCHAIN_DIR`，留空即从 `PATH` 查找：

```powershell
cmake -B build/arm -S . -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_arm.cmake `
      -DLTM_ARM_TOOLCHAIN_DIR=D:/path/to/bin
```

## 工具链文件


| 文件                          | 目标       | 关键选项                                                       |
| ----------------------------- | ---------- | -------------------------------------------------------------- |
| `cmake/toolchain_arm.cmake`   | ARMv7E-M   | `-mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard`   |
| `cmake/toolchain_riscv.cmake` | RV32IMAFDC | `-march=rv32imafc_zicsr_zifencei -mabi=ilp32f -mcmodel=medany` |
| `cmake/toolchain.cmake`       | 本机 gcc   | 不交叉编译，仅固定编译器                                       |

ARM 侧取 M4/M7 的公共基线：M7 的 FPv5 为 FPv4 超集，同一份 `.a` 可在 M4 与 M7 上运行；Cortex-M33/M85 亦可运行，但不使用 MVE 与低开销循环指令。若需针对单一目标重新出库，可通过 `-DLTM_ARM_ARCH_FLAGS=...` 替换指令集选项。

## 构建脚本

```
python script.py -s              构建 arm、riscv、pc 三个平台
python script.py -s arm          只构建指定平台，可给多个
python script.py -e              构建并运行 PC 闭环仿真
python script.py -t              构建并运行 lt_speed 压测
python script.py -b              -s 全部 + PC 仿真
python script.py -c              删除 build/ 与 examples/build
```

产物布局，三个平台结构一致：

```
bin/
├─ arm/
│   ├─ libLTM_CTRL.a
│   └─ ltm_ctrl/
│       ├─ lt_math.h
│       ├─ lt_control.h
│       └─ lt_analysis.h
├─ riscv/   （同上）
└─ pc/      （同上）
```

## 集成到固件工程

- 头文件：把 `ltm_ctrl` 加入 include 路径，代码中写 `#include "ltm_ctrl/lt_control.h"`。
- 库：链接 `libLTM_CTRL.a`。
- 编译选项必须与库一致，否则 ABI 不匹配：

  - ARM：`-mfloat-abi=hard -mfpu=fpv4-sp-d16`
  - RISC-V：`-mabi=ilp32f`
