# LTM_FOC 胶水层 + LTM_CTRL 开环示例（CAN-FD 承载 LTM 协议）

基于 LTM_FOC 胶水层的标准开环示例：LTM 协议通讯（CAN-FD）+ 开环 SVPWM + 10 条曲线上报。
应用层只写 `main.c` + `protocol/`，电机控制与算法全部走胶水层 API，与 LTM_FOC SDK 对接的最简样板。

## 分层结构

| 层 | 产物 | 职责 |
| :--- | :--- | :--- |
| LTM_HAL | `lib/libLTM_HAL.a` + `lib/ltm_hal/ltm_hal.h` | 外设驱动：PWM/ADC/编码器/CAN-FD/UART/LED |
| LTM_CTRL | （已合并进 libLTM_FOC.a） | 控制算法：FOC/PID/测速/死区补偿等 |
| LTM_FOC | `lib/libLTM_FOC.a` + `lib/ltm_foc/lt_motor.h` | 胶水层：状态机 + 25kHz 三环调度 + 一键初始化 |
| App | `src/main.c` + `src/protocol/` | 协议解析、命令分发、曲线上报 |

> `libLTM_FOC.a` 在 SDK 构建期已把 `LTM_CTRL` 合并进去，应用侧只需链接 `libLTM_FOC.a + libLTM_HAL.a`，
> 不会出现重复定义。换平台时只需替换 `lib/` 目录内容。

## 快速开始

```bash
python script.py -b    # 构建（Release）
python script.py -R    # 构建 + J-Link 烧录
```

产物：`bin/ltm_foc_canfd.bin`（可经 IAP/UDS 直接 OTA）。

## 开环控制流

1. `lt_motor_init()` 一键初始化：LTM_HAL + 电流零点校准 + 状态机 + 25kHz 三环回调绑定；
2. 上位机 `Data_Target` 下发占空比（%）→ `lt_motor_set(Mode_Open_Loop, value)`；
3. `Data_CMD_Start` → `lt_motor_enable()` + `lt_motor_run()`，电流环中断内完成 SVPWM 输出；
4. `Data_CMD_Stop` → `lt_motor_stop()` + `lt_motor_disable()`；
5. 周期上报 10 条曲线：A/B/C 电流、母线电压、位置、速度、Id/Iq、驱动/电机温度。

## 通讯约定

- `0x100`：下行 LTM 协议帧（上位机 → 电机，载荷 = LTM 帧字节流，>64B 分片）；
- `0x101`：上行 LTM 协议帧（电机 → 上位机）；
- `0x7F0`：升级触发，App 收到后软复位进 BootLoader（6s 升级窗口）。

## 关键工程记录

- App 固定链接运行区 `0x02008000`（224KB），配合 BootLoader 运行区 + 备份区升级模型；
- RAM 顶部 4 字节保留为 BootLoader 跳转标记（`0x220E9FFC`，`BL_JUMP_MAGIC`），链接脚本已预留；
- BootLoader 跳转后恢复全局中断由 HAL SDK 内部完成（`ltm_hal_init` 处理），应用无需关心；
- CAN-FD RX FIFO Payload Size 必须配 `RFPLS=7`（64B），否则长帧被硬件截断（详见 LTM_HAL 记录）；
- 浮点打印用 `LT_FTOAT3`（`src/util/lt_str.h`），固定 3 位小数，不链接标准库浮点 printf，省数 KB。

## 内存占用（Release，-O2）

| text | data | bss | dec | Flash 占用 | RAM 占用 |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 37,124 | 460 | 22,946 | 60,530 | 37,560 B / 224 KB（16.4%） | 23,410 B / 936 KB（2.4%） |

## 目录结构

```
canfd/
├── lib/                  # SDK 产物（换平台只改这里）
│   ├── libLTM_HAL.a
│   ├── libLTM_FOC.a      # 已内含 LTM_CTRL
│   ├── ltm_hal/ltm_hal.h
│   └── ltm_foc/lt_motor.h
├── linker/               # 链接脚本（App 运行区 0x02008000）
├── src/
│   ├── main.c            # 应用层：协议 + 命令分发 + 上报
│   ├── protocol/         # LTM 协议（ltm_commut + protocol）
│   └── util/lt_str.h     # 简易浮点转字符串
├── CMakeLists.txt
└── script.py             # 一键构建 / 烧录
```
