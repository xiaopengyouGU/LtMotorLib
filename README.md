# LtMotorLib

面向 PMSM/BLDC 的电机控制软件库。分层设计，底层驱动、控制胶水与算法模块彼此解耦；同一套控制代码按平台与板级配置编译，既能产出固件用的预编译库，也能在 PC 上跑闭环仿真与回归测试。

许可：MIT。

## 架构

```
应用层（用户自行实现）
   │  #include "ltm_foc/lt_motor.h"
LTM_FOC    控制胶水：状态机 + 三环级联调度 + 电机对象 API（内含 LTM_CTRL）
   │  #include "ltm_hal/ltm_hal.h"
LTM_HAL    底层驱动：把 MCU 外设抽象为电机所需的接口
   │
目标硬件
```

| 层 | 定位 | 交付物 |
| --- | --- | --- |
| LTM_CTRL | 纯算法：FOC、PID、PDOB、S 曲线规划、测速、陷波、参数校准与辨识 | `libLTM_CTRL.a` + 三个聚合头 |
| LTM_FOC | 控制胶水：三环调度、状态机、模式管理、机型参数 | `libLTM_FOC.a`（内含 CTRL）+ `lt_motor.h` |
| LTM_HAL | 底层驱动：PWM / ADC / 编码器 / 定时 / 通信 / Flash 的抽象接口 | 每个平台一对 `ltm_hal.h` + `libLTM_HAL.a` |

依赖方向单向：`应用 → LTM_FOC → LTM_HAL`；LTM_CTRL 只被 LTM_FOC 使用，不直接面向应用。

## 交付模型

- **应用层用户**：`libLTM_FOC.a` + `libLTM_HAL.a` + `lt_motor.h` + `ltm_hal.h` + 链接脚本；应用层由用户自行编写。
- **自研胶水层的内部用户**：`libLTM_CTRL.a` + 三个聚合头，配合自己的调度实现。
- LTM_CTRL 的静态库在 LTM_FOC 构建时以对象形式并入，用户无需关心其内部模块划分。
- 本仓库当前不跟踪预编译库（`.a`），需要时从源码构建产生。

## 目录结构

```
cmake/       共享构建层：三份工具链（arm / riscv / pc）+ 对外头生成脚本
boards/      板级映射：<board>/{board.cmake, user_param_def.h, linker/}
LTM_CTRL/    算法库：平台无关，可在 PC 上编译、仿真与压测
LTM_FOC/     控制胶水：构建时并入 LTM_CTRL，输出单头与合并库
LTM_HAL/     平台驱动包：一个平台一份，随该平台一起交付
examples/    例程：basic（串口承载 LTM 协议）、canfd（CAN-FD）
BootLoader/  IAP / UDS 在线升级：A/B 分区，升级失败可回滚
python/      上位机辅助脚本（伯德图、SVPWM 对比等）
```

## 平台与板级

- **平台驱动包**：`LTM_HAL/ltm_hal_ra6m4`、`LTM_HAL/ltm_hal_ra8t2_CPU0`，各自提供该平台的 `ltm_hal.h` 与 `libLTM_HAL.a`；接口函数跨平台同名同义，平台相关常量以宏的形式随该平台的头文件给出。
- **板级映射**：`boards/<board>/board.cmake` 指定该板使用哪个 HAL 包、哪套工具链、哪个链接脚本与哪些 MCU 编译定义；机型参数在 `boards/<board>/user_param_def.h`。
- 移植一个新平台：新增一份 HAL 平台包 + 在 `boards/` 下新增一份映射，控制代码无需改动。

## 构建

```bash
# 1) 算法库（可选，三个平台）
python LTM_CTRL/script.py -s              # arm / riscv / pc
python LTM_CTRL/script.py -e              # PC 闭环仿真
python LTM_CTRL/script.py -t              # lt_speed 主机侧压测

# 2) 控制胶水库（按板级参数编译）
python LTM_FOC/script.py -s --board ra6m4

# 3) 例程（链接 SDK 交付库与板级链接脚本）
python examples/basic/script.py -b
python examples/canfd/script.py -b
```

工具链不在 `PATH` 时，用 `-DLTM_ARM_TOOLCHAIN_DIR`、`-DLTM_RISCV_TOOLCHAIN_DIR`、`-DLTM_PC_TOOLCHAIN_DIR` 指定其 `bin` 目录。板级参数未指定时构建会直接报错，不会静默采用默认值。

## 设计约定

- **零动态内存**：模块状态存放在内部静态实例池，调用方通过 `idx` 访问，实例数量由 `LT_*_MAX_INSTANCES` 在编译期确定。
- **定点为主**：控制主链使用 Q15 / Q24 定点，`analysis` 模块使用 float；热路径避免除法与浮点。
- **单位写死在接口上**：count/s、Q15 标幺、RPM、0.1℃ 等单位在函数注释中明确，不做隐式换算。
- **三环严格级联**：位置环 → 速度环 → 电流环；规划与重计算放在主循环，中断内只做轻量步进。
- **测量与控制通道分离**：测速的 M 法通道用于观测，控制反馈统一走锁相环输出。

## 许可

MIT License，见 [LICENSE](LICENSE)。仓库内 `LTM_HAL` 的 FSP / CMSIS 等第三方组件保留其原始许可。