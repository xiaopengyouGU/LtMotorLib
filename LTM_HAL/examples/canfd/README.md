# LTM_HAL RA8T2 开环例程

基于 `libLTM_HAL.a` 的简单开环 FOC 例程：LTM 协议通信 + 开环 SVPWM + 5 条曲线上报。
固定链接到 **运行区 0x02008000（224KB）**，配合 BootLoader 运行区 + 备份区升级模型，
生成的 `ltm_hal_xxx.bin` 可直接通过 IAP/UDS 烧录。

## 构建

SDK 产物已随工程放在 `lib/`（`lib/libLTM_HAL.a` + `lib/ltm_hal/ltm_hal.h`），
对接不同平台只需替换 `lib/` 下的内容。

```bash
python script.py            # 构建示例（cmake 手动构建亦可）
```

## 修改记录

### 1. 链接脚本：App 固定链接运行区

- `linker/memory_regions.ld`：`FLASH_START = 0x02008000`、`FLASH_LENGTH = 0x38000`（224KB）
- 与 BootLoader 分区模型对齐：Boot 32KB（0x02000000）→ 运行区 224KB（0x02008000）
- `.bin` 由 objcopy 生成并排除 `__option_setting_*`（芯片配置段不进固件）

### 2. 干掉标准库浮点打印（省 ~9.3KB）

- `CMakeLists.txt`：移除 `-u _printf_float`，不再链接标准库浮点 printf
- `src/math/basic/lt_math.h`：新增 `LT_FTOAT3` 宏——纯整数格式化（`%ld`）实现
  浮点转字符串，固定 3 位小数，带四舍五入与进位处理，不依赖浮点格式化代码
- `src/main.c`：全部 `%.1f / %.4f` 调用改为 `%s` + `LT_FTOAT3`

### 3. 堆栈降配（省 28KB RAM）

SDK ? `ltm_hal_ra8t2_boot/src/drivers/hardware/ra_cfg/fsp_cfg/bsp/bsp_cfg.h`：


| 配置项                     | 修改前         | 修改后             | 说明                               |
| -------------------------- | -------------- | ------------------ | ---------------------------------- |
| `BSP_CFG_STACK_MAIN_BYTES` | 32KB（0x8000） | **16KB（0x4000）** | 裸机 + FOC 中断足够                |
| `BSP_CFG_HEAP_BYTES`       | 16KB（0x4000） | **4KB（0x1000）**  | example 零动态分配，4KB 为保险余量 |

> 注意：堆栈大小编译进 libLTM_HAL.a，改完需**重编库**（根目录 `cmake --build build`）
> 再重编 example，`bin/libLTM_HAL.a` 会同步更新。

### 4. BootLoader 跳转标记（修复跳转后无响应）

**背景**：BootLoader `port_sys_jump()` 跳转前 `__disable_irq()`，软跳转**不复位 PRIMASK**，
App 从 Reset_Handler 启动后全局中断仍是关闭状态 → UART/CAN-FD 收不到数据 → 响应错误。

**方案**：RAM 顶部保留 4 字节做跳转标记，两端约定：

- BootLoader `port_sys_jump()` 跳转前写入 `BL_JUMP_MAGIC`（"JUMP"）；
- BootLoader 跳转前同时 **Close 掉自己打开的外设**（`R_SCI_B_UART_Close` + `R_CANFD_Close`），
  让 App 从干净的硬件状态重新初始化；
- App `main()` 最先调用 `bl_commut_init()`：检测到标记 → 恢复全局中断（`cpsie i`）→ 清标记；
- 两端链接脚本 `RAM_LENGTH` 均减 4（`0xEA000` → `0xE9FFC`），顶部字留给标记，永不与栈/堆冲突。

标记地址：`0x220E9FFC`（`BL_JUMP_FLAG_ADDR`）。

### 5. App 主动进 BootLoader（强行烧录入口）

App 运行中可随时软复位到 BootLoader（复位后 6s 升级窗口内开始烧录）：

- **串口**：收到 `Data_CMD_Reset`（上位机"复位"按钮）→ 软复位；
- **CAN-FD**：收到 ID `0x7F0`（与 BootLoader 升级 ID 一致）→ 软复位；
- 软复位实现：`app_system_reset()` 直写 `SCB->AIRCR`（VECTKEY+SYSRESETREQ），
  不依赖 CMSIS 头文件（example 编译路径未包含）。

> 工作流：上位机先选好固件、界面停在升级页 → 触发复位 → 6s 窗口内点"开始升级"。
> 窗口长度由 BootLoader `BL_IDLE_TIMEOUT_MS` 决定，调试期可临时调大。

### 6. CAN-FD RX FIFO Payload 64B（RFPLS=7）

RA8T2 的 CAN-FD RX FIFO Payload Size 配 8B 时，硬件对超过 8 字节的帧只保留 8 字节，长数据被截断——BootLoader UDS 升级 13B 的 0x34 请求即因此 size=0 失败。本工程与 BootLoader 的全部 `hal_data.c` 已将 `rx_fifo_config` 的 `RFPLS` 统一改为 7（64B）。该文件标注 generated，用 FSP 重新生成会覆盖，需重新修改。

## 内存占用演进


| 版本                  | text   | data | bss        | dec        |
| --------------------- | ------ | ---- | ---------- | ---------- |
| 初始（含浮点 printf） | 41,792 | 696  | 51,384     | 93,872     |
| 去浮点 printf         | 32,596 | 332  | 51,396     | 84,324     |
| 堆栈降配后            | 32,588 | 332  | **22,724** | **55,644** |
| 跳转恢复后            | 32,676 | 332  | 22,724     | 55,732     |

RAM（data + bss）从 52KB 降至 23KB，Flash 占用从 41.8KB 降至 32.6KB。

## 注意事项

- 库用 `--whole-archive` 全量链接，确保 IRQ 向量表与全部 ISR 被拉取（否则中断无效）。
- `bsp_cfg.h` 标注"generated"，若用 FSP 重新生成配置会覆盖堆栈/堆改动，需重新修改。
- ITCM/DTCM 未启用（0%）；FOC 控制环如需极致实时性，可将 ISR/控制环代码放入 ITCM。
