# LTM_HAL 开环例程（canfd）

LTM_HAL 各平台共用的一份开环 FOC 例程，LTM 协议经 CAN-FD 承载：
开环 SVPWM + 10 条曲线上报（A/B/C、α/β、母线电压、机械角度、Id/Iq、驱动板温度）。
板级（HAL 包、工具链、链接脚本、机型参数）全部来自 `boards/<board>/board.cmake`。

## 构建与烧录

```bash
python script.py -b --board ra6m4           # 构建（默认板级 ra6m4）
python script.py -R --board ra6m4           # 构建 + J-Link 烧录
python LTM_HAL/script.py -s --hal ra6m4     # 编 SDK 时顺带编本平台例程
```

- 依赖已编好的 SDK：`${LTM_HAL_SDK_DIR}/bin/libLTM_HAL.a`（由 `LTM_HAL/script.py -s` 产出）；
- 公共源码在仓库根 `components/`（control / math / protocol），例程不自备副本。

## LTM-over-CANFD

| CAN-FD ID | 方向 | 用途 |
| :--- | :--- | :--- |
| `0x100` | 收 | LTM 协议帧载荷（>64B 分片），直接进协议环形缓冲 |
| `0x101` | 发 | LTM 协议帧输出（同理分片） |
| `0x7F0` | 收 | 升级触发：收到即软复位进 BootLoader 升级窗口 |

## 机型参数

`POLE_PAIRS`（极对数）与 `ENCODER_ZERO_OFFSET`（电角度零点，count；0 = 上电预定位实测）
由板级 `LTM_BOARD_EXAMPLE_DEFINES` 覆盖，见 `boards/<board>/board.cmake`。
