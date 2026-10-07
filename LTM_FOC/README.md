# LTM_FOC（HPM6E8Y Joint 关节板）

胶水层 SDK：LTM_HAL（底层驱动）+ LTM_CTRL（控制算法，已合并进 `libLTM_FOC.a`）+ LTM_FOC（状态机 + 三环调度 + 电机 API）。

- 平台：HPM6E8Y（RISC-V rv32imafc，200MHz PWM，浮点硬件）
- 电机：14 对极、100W 关节电机，MT6701 14bit 绝对值编码器（16384 CPR）
- 频率分配：电流环 20kHz / 速度环 2kHz / 位置环 1kHz（三环级联分频，同一 ADC 中断内串行执行）
- App 固定链接运行区 `0x02008000`，配合 BootLoader 运行区 + 备份区升级模型

---

## 三环整定全流程（从电流环开始）

### 1. 电流环（最顺的一环）

带宽整定法（零极点对消）：**Kp = Ls·ωc，Ki = Rs·ωc**

- Ls = 305µH，Rs = 0.33Ω，ωc = 2π×500Hz，500Hz 闭环验证通过
- 输出归一化系数实时按母线电压计算：`1/(Udc×0.5)`，母线异常兜底 24V
- 死区补偿（电流极性法）+ 一拍延迟补偿（电压晚 1.5 拍生效）

电流环是系统地基，后续所有问题都不在它身上。

### 2. 测速方案（自适应 M 法 + PLL 双通道）

14bit 编码器是硬约束。两条测速链并行，用途分离：

| 通道 | 实现 | 特性 | 用途 |
| :--- | :--- | :--- | :--- |
| 自适应 M 法 | 窗口 18→12→9→6→3→1 拍自适应切换 + 低通 | 测量精度高、可信 | 上报显示（`info.speed`） |
| PLL 测速 | 位置/速度闭环估计器（αβ 域） | 输出平滑、无窗口量化跳变 | 控制反馈（速度环） |

PLL 参数整定轨迹：178Hz（噪声大）→ 100Hz（震）→ 50Hz（平滑但积分+反馈滞后慢速极限环，等一会失稳）→ **150Hz** → 120Hz → **90Hz 定稿**。

曾试过"高速 M 法 / 低速 PLL"切换（150/180RPM 滞回），M 法在切换点会抖，最终**反馈统一用 PLL**，M 法只做测量。

### 3. 速度环（最难的一环）

模型整定：

- K_plant = kt·9.5493 / J ≈ 1.30e4 RPM/(A·s)（J=1.2e-4 kg·m² 估算，kt=0.164 N·m/A）
- **Kp = 2ζωn / K_plant，Ki = ωn² / K_plant**
- 定稿：ωn≈7Hz、ζ≈2.0 过阻尼 → **Kp=0.0136、Ki=0.15**

实测带宽天花板 **~7Hz**：10Hz 起振崩环——14bit+2kHz 反馈链的物理极限（M 法窗口滞后、PLL 反馈相位、量化噪声共同决定）。

**摩擦前馈**是速度模式能跑的关键（设备静摩擦大、动摩擦小）：

- 静摩擦突破电流：0.14A（Te=0.13 起不来、0.14 基本能转）
- 粘性系数：0.0002 A/RPM
- 前馈死区：±10RPM（防零速抖振）

根治粘滑极限环（PI 憋力矩 → 突破静摩擦 → 过冲 → 反向的循环）。

### 4. 齿槽补偿（全链路）

14 对极齿槽大，Q 轴电流波动明显。补偿链路：

1. 上位机发 `CC=1` 武装，`Start` 后内部自动进入 15RPM 恒速辨识（PLL 测速门控采样）
2. 采满 512 点自动停机，表留在 RAM，由 JLink 固化到 Flash `0x80024000`
3. 固化镜像：`[magic "COGG"][size=512][table×512 float][crc32]`（2060B，CRC 覆盖连续的 size+table 2052B）
4. 上电 `ident_task_load()` 校验魔数/大小/CRC 后自动重建表
5. 速度环按绝对位置查表前馈，方向用实测速度判别（死区防零速抖振）

低速取表（10RPM 门控）后效果明显。JLink 烧录时 App 区域不覆盖表扇区。

### 5. 位置环

- 指令：`P=xx` 绝对机械角度（°），运行中可改目标
- 梯形规划：LTM_CTRL `lt_scurve`（type=0 T 形），Unit=0.1°，速度上限 450RPM、斜率 600RPM/s，短距离自动降级三角形
- 位置环 PD：**Kp=4 RPM/°、Kd=0.1**（8 超调；Kd 10/5/1 均啸叫——增量式 D 对 14bit 量化二阶差分太敏感）
- 到位不干预：位置环连续跟踪，规划自然减速到位，误差归零输出自然趋 0

### 6. lt_scurve 排坑记录（三个真 bug）

1. **T 形减速段被限幅清零**：`update()` 里 `phase≤2` 的负加速度被强制归零，导致 T 形/三角形规划全程不减速、全速冲过目标——**定位全乱的根因**。修复：T 形按 ±limit 双向限幅（S 形减速段在 phase3~4，不受影响）。
2. **pos/vel 整数截断**：int32 累积导致减速末端停在 target 前约 1°，规划完成瞬间 target 跳变 → 速度尖峰 + 定位偏差。修复：`pos`/`vel` 改 float 内部累积。
3. **主循环 start 与中断 update 竞态**：`lt_scurve_start`（含软件 sqrt）在主循环执行，中断内 `update` 可能读到半更新状态 → 偶发规划错乱。修复：模块内部 `ready` 就绪标志 + `pos_hold` 旧值保持——`start` 入口清 0、算完置 1，`update` 未就绪返回旧位置不步进。

另：`sqrtf` → `lt_sqrt`（快速逆平方根，无 libm 依赖，例程链接不需要 `-lm`）。

---

## 当前关键参数

| 项目 | 参数 |
| :--- | :--- |
| 电流环 | 20kHz，ωc=500Hz，Kp=Ls·ωc / Ki=Rs·ωc |
| PLL 测速 | ωn≈90Hz：Kp=792、Ki=160（2kHz 更新） |
| 速度环 | 2kHz，ωn≈7Hz、ζ≈2：Kp=0.0136、Ki=0.15 |
| 位置环 | 1kHz，Kp=4、Kd=0.1（增量式） |
| 梯形规划 | 450RPM 上限，600RPM/s，T 形/三角形自动降级 |
| 摩擦前馈 | Fc=0.14A、B=0.0002 A/RPM、死区 ±10RPM |
| 齿槽表 | Flash 0x80024000，512×float，上电自动加载 |
| 到位 | 无锁定干预，位置环连续跟踪自然收敛 |

---

## 架构原则

- 三环严格级联：**位置环 → 速度环 → 电流环**，位置环不碰电流环
- 规划/重计算放主循环（`lt_motor_run` 完成初次规划），中断只做轻量步进
- 测量通道与控制反馈通道分离：显示用 M 法、控制用 PLL
- 电机参数（Rs/Ls/ψf）集中在 `tasks_param_def.h`

---

## 指令约定（basic 例程，串口承载 LTM 协议）

| 指令 | 含义 |
| :--- | :--- |
| `Te=xx` | 力矩模式，目标 Q 轴电流（A） |
| `V=xx` | 斜坡速度模式，目标转速（RPM） |
| `P=xx` | 位置模式，绝对角度（°） |
| `CC=1` | 齿槽辨识武装（Start 后自动辨识→停机） |
| `Start` / `Stop` | 使能运行 / 停机失能 |
| `Reset` | 软复位回 BootLoader（6s 升级窗口） |

模式切换仅允许在 Idle（停机失能）状态，运行中切换会被拒绝。

---

## 构建与烧录

```bash
# 1. 构建 LTM_CTRL（hpm6e8y）
cmake --build D:/Develop/LtMotorLib/LTM_CTRL/build_hpm

# 2. 构建 LTM_FOC SDK（自动 merge LTM_CTRL 到 bin/libLTM_FOC.a）
python script.py -s

# 3. 同步库到头文件到 basic 例程
copy bin/libLTM_FOC.a examples/basic/lib/
copy bin/ltm_foc/lt_motor.h examples/basic/lib/ltm_foc/

# 4. 链接 basic 例程
cmake --build examples/basic/build

# 5. J-Link 烧录
JLink.exe tmp_jlink/flash_joint_basic.jlink
```

> 修改 LTM_CTRL 后，LTM_FOC 需 touch 任一源文件触发重新 merge。

## 目录结构

```
ltm_foc_hpm6e8y_Joint/
├─ src/ltm_foc/
│  ├─ common/tasks_param_def.h   # 控制参数集中定义
│  ├─ motor/lt_motor.c           # 电机 API（init/enable/run/set/cmd）
│  ├─ tasks/control_tasks.c      # 三环调度 + 位置规划 + 到位处理
│  ├─ tasks/calib_task.c         # 电角度零点自校准
│  ├─ tasks/ident_task.c         # 齿槽辨识 + 固化表加载
│  └─ schedule/lt_fsm.c          # 状态机
├─ examples/basic/               # 串口 basic 例程（Te/V/P/CC 指令）
├─ examples/canfd/               # CAN-FD 例程
├─ bin/                          # libLTM_FOC.a + 合并的 LTM_CTRL
└─ script.py                     # SDK 一键构建
```
