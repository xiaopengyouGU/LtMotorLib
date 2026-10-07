# RA6M4 三相互补 PWM 死区实现说明（GPT6/7/8）

适用对象：`src/drivers/bsp/pwm/pwm.c`
硬件：RA6M4，U=GPT6(P600/P601)、V=GPT7(P303/P304)、W=GPT8(P106/P107)
模式：三角波对称 PWM（`TIMER_MODE_TRIANGLE_WAVE_SYMMETRIC_PWM`，GTCR.MD=4）
载波：20kHz，GTPR=2475，PCLKD=99MHz

## 1 实现要点

- `GTDTCR.TDE = 0`，不使用 GPT 的硬件负相/死区功能。
- 死区由 A/B 两路比较值错开产生：

      GTCCRA = center + DEAD/2
      GTCCRB = center - DEAD/2
      DEAD   = PWM_DEAD_TIME_COUNTS = 50  （≈505ns，与原 RASC 的 Dead Time 同值）

- 比较值必须在计数器启动前预置；运行中只更新缓冲寄存器。
- `center` 限幅在 `[DEAD/2+1, GTPR-1-DEAD/2]`，保证两路比较值都严格落在 (0, GTPR) 内且互不相等。

## 2 硬件行为

RASC 为每个通道生成的配置相同：

| 寄存器 | 值 | 含义 |
| --- | --- | --- |
| GTCR | 0x00040001 | MD=4 三角波对称模式；CST=1 |
| GTIOR | 0x01530103 | GTIOCnA：初电平低、比较匹配翻转、输出使能；GTIOCnB：初电平高、比较匹配翻转、输出使能 |
| GTBER | 0x00150000 | GTCCRA/GTCCRB/GTPR 均为单缓冲（FSP 写入的 CCRSWT 位读回为 0） |
| GTPR | 0x9AB = 2475 | 半周期计数值 |
| GTDTCR | 0（由应用代码清除） | 关闭负相/死区功能 |

- GTIOCnA 在 GTCCRA 命中时翻转，GTIOCnB 在 GTCCRB 命中时翻转；三角波一个周期内各命中两次（上计数、下计数各一次）。
- 占空比关系：GTIOCnA 高电平宽度 = 2×(GTPR−GTCCRA) 计数，GTIOCnB 高电平宽度 = 2×GTCCRB 计数。
- 两路初始电平相反，因此 `GTCCRA == GTCCRB` 时两路自然互补、死区为 0。

## 3 失效机理（本次问题的根因）

翻转式输出的相位是**运行状态**，不是配置项。三个通道的 GPT 寄存器完全一致时，仍可能只有某一相处于“反半周期”状态，该故障无法通过比对配置定位。

已验证机理：

1. 计数器启动后修改比较值。新值落在当前计数之后时，本周期会多一次命中翻转；直接写 `GTCCRA/GTCCRB`（绕过缓冲寄存器）时更易发生。“先 Start 再设置占空比”的初始化顺序即属此类。实测依据：本次故障中 U 相与 V/W 的 GPT 寄存器完全一致，差异仅在于比较值的写入时序。
2. 比较值等于 0 或 GTPR。此时一周期只命中一次，输出周期与相位均退化。0% 占空比（`pwm_set_dutys(0,0,0)`）算出的比较值恰为 GTPR，校零阶段每次上电必然触发。

未单独复现、列为待确认项：

3. `GTCCRA == GTCCRB`（FSP 三相模块 `R_GPT_THREE_PHASE_DutyCycleSet()` 的默认行为）。直接后果是死区恒为 0；是否本身会引发相位翻转尚无独立验证——本次观测到"两路同相"的样本中同时存在第 1 条的干扰。即便如此，仍应保持两路比较值不等，理由见第 5 节。

## 4 RASC/FSP 配置为何无法覆盖

- 三相模块接口没有死区参数，其占空比写入固定使 A/B 两路比较值相等。
- RASC 中 GPT 的 "Dead Time" 对应 `GTDVU/GTDTCR.TDE`（硬件负相）。该功能在三角波对称模式下不成立：实测 TDE=1、GTDVU=50 时 GTIOCnB 输出为"同相、滞后 50 计数"，两路占空比相同，并非反相。
- 因此死区只能在应用代码中由比较值偏移实现，输出相位也必须由应用代码保证。
- FSP 源码与配置头中未见上述约束的说明（`r_gpt.c` 中仅有一行 `Enable Dead-time nagative-phase waveform` 注释，拼写为原文）。

## 5 实现规则（修改本模块时必须保持）

1. `pwm_init()` 在 `R_GPT_THREE_PHASE_Open()` 之后清除 `GTDTCR`（TDE=0）。重新生成 RASC 后该代码必须保留，否则 FSP 会依据 "Dead Time=50" 重新置位 TDE。
2. 启动前用 `pwm_preload_phase()` 同时写 GTCCRA/GTCCRB 及其缓冲寄存器，预置完成后再调用 `R_GPT_THREE_PHASE_Start()`。
3. 运行中用 `pwm_write_phase()` 仅写 `GTCCR[2]`(0x54) / `GTCCR[3]`(0x58)，由硬件在比较匹配点整体生效，避免丢翻转。
4. 保持 `center` 限幅，禁止出现 `GTCCRA == GTCCRB`、比较值为 0 或为 GTPR。
5. 方向固定为 `GTCCRA` 取大、`GTCCRB` 取小：两条边沿各留 DEAD 计数的"两路同时关断"空隙。取反则形成"两路同时导通"的重叠。

## 6 验证方法与实测数据

示波器：每一对引脚应为 20kHz 互补波形，翻转处留约 505ns 空隙。

J-Link 回读（固定占空比 0.16/0.37/0.82 测试工况）：

| 相 | 引脚（GTIOCnA / GTIOCnB） | GTCCRA | GTCCRB | GTIOCnA 高电平区间 |
| --- | --- | --- | --- | --- |
| U | P601 / P600 | 2104 | 2054 | (2104, 2475] |
| V | P304 / P303 | 1584 | 1534 | (1584, 2475] |
| W | P107 / P106 | 470 | 420 | (470, 2475] |

判定标准：

- 每对引脚在 `(GTCCRB, GTCCRA)` 区间内两路同为低电平，即死区。
- GTIOCnA 占空比等于指令值，GTIOCnB 为其互补。若某相 GTIOCnA 占空比表现为 (1−指令值)，说明该相发生了第 3 节的相位翻转，需复位后重新初始化。

## 7 已知遗留

- 0% 占空比经限幅后输出约 2 计数（≈20ns）的最小脉冲，而不是完全停止开关。若校零阶段要求桥臂完全静止，应在 0% 时关闭输出（OAE/OBE=0，引脚停在 OADFLT/OBDFLT：A 低、B 高，即全下桥导通）。

## 8 修订记录

- 2026-09-28：首次定位并修复。根因见第 3 节；验证固件与源码见 `backup/LTM_HAL/ltm_hal_ra6m4/dtime_ok_20260928_214713`。