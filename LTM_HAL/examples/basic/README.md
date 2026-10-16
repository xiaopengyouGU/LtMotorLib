# LTM_HAL 开环例程（basic）

LTM_HAL 各平台共用的一份开环 FOC 例程：LTM 协议通信（UART）+ 开环 SVPWM + 7 条曲线上报。
板级（HAL 包、工具链、链接脚本、机型参数）全部来自 `boards/<board>/board.cmake`。

## 构建与烧录

```bash
python script.py -b --board ra6m4           # 构建（默认板级 ra6m4）
python script.py -R --board ra6m4           # 构建 + J-Link 烧录
python LTM_HAL/script.py -s --hal ra6m4     # 编 SDK 时顺带编本平台例程
```

- 依赖已编好的 SDK：`${LTM_HAL_SDK_DIR}/bin/libLTM_HAL.a`（由 `LTM_HAL/script.py -s` 产出）；
- 公共源码在仓库根 `components/`（control / math / protocol），例程不自备副本；
- 烧录用器件名与 SWD 时钟取自板级 `LTM_BOARD_JLINK_DEVICE` / `LTM_BOARD_JLINK_SPEED`。

## 机型参数

`POLE_PAIRS`（极对数）与 `ENCODER_ZERO_OFFSET`（电角度零点，count；0 = 上电预定位实测）
由板级 `LTM_BOARD_EXAMPLE_DEFINES` 覆盖，见 `boards/<board>/board.cmake`。

## 上电流程

1. `ltm_hal_init()` + LTM 协议通道初始化；
2. 三相 50% 等占空比下做电流零点校准（`ltm_adc_calibrate_zero`，校准完成前电流读数为 0）；
3. 停机偏置自检：打印三相平均电流；
4. 电角度零点：板级给了 `ENCODER_ZERO_OFFSET` 就用固定值，否则 D 轴强拖一次实测编码器零点；
5. 主循环：协议命令 → 编码器电角度 → 开环 SVPWM → 7 条曲线上报。
