# 双路红外巡线传感器校准

## 硬件

- QST 鸿蒙小车
- TCRT5000 ×2
- LM393 双路比较器
- SR1/SR2：两路 LM393 比较阈值调节电阻

传感器链路为 `TCRT5000 -> LM393 -> GPIO`。老师已确认，巡线前必须通过
SR1/SR2 调整两路比较器阈值。

## 正式赛道与数字极性

正式赛道为白底黑线。实机确认：

- 白底 = `0`
- 黑线 = `1`

## 实机物理映射

当前 Windows UDP 校准报文使用历史标签 `CAL L/R`，其字母**不代表物理左右**。

- 物理左探头压黑线：`CAL L=0 R=1`
- 物理右探头压黑线：`CAL L=1 R=0`

因此：

- `CAL R` = 物理左探头
- `CAL L` = 物理右探头

代码已在 `LineSensorRead()` 中统一为物理命名：

- `physical_left` = GPIO13
- `physical_right` = GPIO14

校准 UDP 输出仍保留 `CAL L=<physical_right> R=<physical_left>`，以兼容
当前 Windows 监视器和本次实测记录。

## 校准目标状态

以下 `00/01/10/11` 按 **CAL 输出顺序 L,R** 记录，而不是物理左右：

| 物理状态 | CAL 输出 |
| --- | --- |
| 两边白 | `00` |
| 左黑、右白 | `01` |
| 左白、右黑 | `10` |
| 两边黑 | `11` |

## 本次故障与结论

- 正常车高下，白底和黑线最初都读为 `0`。
- 架空和手遮挡可以读到 `1`。
- raw 采样统计表明问题不是 debounce 或采样频率。
- 最终由老师确认需要调整 SR1/SR2。
- 调节后已得到明确的单侧黑线识别结果。

## 重新校准流程

1. 保持整车正常四轮着地；不要翘车、垫高或改变传感器安装高度。
2. 使用正式白底黑线赛道。
3. 在 `src/task_car_control.c` 启用：
   ```c
   #define LINE_SENSOR_CALIBRATION_MODE 1
   ```
4. 通过 Windows UDP 监视器观察 `CAL L=<0|1> R=<0|1>`。
5. 分别细调 SR1/SR2，直到白底、单侧黑线和双黑的四种状态稳定。
6. 验收：`00` / `01` / `10` / `11`。
7. 完成后关闭 `LINE_SENSOR_CALIBRATION_MODE`；正式运行时诊断模式默认关闭。

## 诊断能力保留

- `LINE_SENSOR_CALIBRATION_MODE`：约每 100ms 输出 raw `CAL`，电机始终 STOP，
  不执行 debounce 或巡线控制。
- `LINE_SENSOR_ANALYSIS_MODE`：约每 10ms raw 采样、每 2 秒输出一次
  `SENSORSTAT` 的 0/1 数量与跳变统计，电机始终 STOP。

Windows UDP 监视器应支持以下前缀：

- `CAL `
- `SENSORSTAT `
- `LINE `
- `STAT `
