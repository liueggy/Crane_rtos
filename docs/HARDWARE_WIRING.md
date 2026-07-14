# Crane_rtos 硬件接线基线

更新时间：2026-07-14。以 `Crane_rtos.ioc` 和 `Core/Inc/main.h` 为准。

## 主控与电源

- 主控：STM32F103ZET6。
- 电池：24V，系统总电压不得超过竞赛规定的 36V。
- 直流减速电机为 12V，必须由 24V 转 12V DC-DC 供电，不能直接接 24V。
- 舵机使用独立 5V/6V 电源；STM32、L298N、TB6600、K230、编码器和舵机电源必须共地。

## 四路直流减速电机与 L298N

L298N 的 EN1 至 EN4 只作使能，保持高电平或使用跳帽。速度和方向采用 IN1/IN2 双 PWM。

| 电机 | IN1 PWM | IN2 PWM | 编码器 A | 编码器 B |
| --- | --- | --- | --- | --- |
| M1 | PA6 / TIM3_CH1 | PA7 / TIM3_CH2 | PD10 | PD4 |
| M2 | PB0 / TIM3_CH3 | PB1 / TIM3_CH4 | PD13 | PD5 |
| M3 | PC6 / TIM8_CH1 | PC7 / TIM8_CH2 | PD14 | PD6 |
| M4 | PC8 / TIM8_CH3 | PC9 / TIM8_CH4 | PD15 | PD7 |

```text
正转：IN1=PWM，IN2=0
反转：IN1=0，IN2=PWM
停止：IN1=0，IN2=0
```

TIM3/TIM8：`PSC=71`、`ARR=99`、PWM 频率 10kHz、命令范围 0 至 99。

编码器线色：红/白为电机端，黑=GND，蓝=3.3V，黄=A 相，绿=B 相。输出轴每圈计数 `374` 仅为暂定值，必须实测校准。

## X/Z 步进轴与 TB6600

| 轴 | PUL | DIR | ENA |
| --- | --- | --- | --- |
| X 轴 | PE9 / TIM1_CH1 | PD2 | PD11 |
| Z 轴 | PE11 / TIM1_CH2 | PD3 | PD12 |

TB6600 的 ENA 低电平有效。Z 轴当前实机参数：4 细分、800 脉冲/圈、TIM1 `PSC=71`、`ARR=499`、脉冲频率约 2kHz。

## 舵机、OLED 与 K230

| 模块 | 信号 | 配置 |
| --- | --- | --- |
| 舵机 1 | PA0 / TIM2_CH1 | 50Hz，270 度，默认 90 度 |
| 舵机 2 | PA1 / TIM2_CH2 | 50Hz，270 度 |
| OLED | PB6/PB7 / I2C1 | SSD1306，地址 0x3C |
| K230 TX -> STM32 RX | PA3 / USART2_RX | 115200 8N1，DMA RX |
| K230 RX <- STM32 TX | PA2 / USART2_TX | 115200 8N1，DMA TX |

K230 与 STM32 交叉连接 TX/RX，并共地。当前 STM32 调试命令为 `PING`、`GET`、`A1/A2 <0..270>`、`S1/S2 <us>`、`AUTO START`、`AUTO ABORT`。

## 按键、急停与对射光电限位

| 功能 | 引脚 | 电平与中断 |
| --- | --- | --- |
| K0 | PE4 | 低有效，下降沿 EXTI4，内部上拉 |
| K1 | PE3 | 低有效，下降沿 EXTI3，内部上拉 |
| 急停 | PE7 | 低有效，下降沿 EXTI9_5，内部上拉 |
| X 最小端 | PE0 | 光电遮挡高有效，上升沿 EXTI0，内部下拉 |
| X 最大端 | PE1 | 光电遮挡高有效，上升沿 EXTI1，内部下拉 |
| Z 上端 | PE2 | 光电遮挡高有效，上升沿 EXTI2，内部下拉 |
| Z 下端 | PC5 | 光电遮挡高有效，上升沿 EXTI5，内部下拉 |

对射光电的模块逻辑：无遮挡 `DO=低`，遮挡 `DO=高`。只有模块使用 3.3V 供电或确认 DO 高电平不超过 3.3V 时，DO 才能直连 STM32；5V/12V/24V DO 必须先进行电平转换。

## 未占用引脚

PC0 至 PC4 为旧电机方向引脚预留，当前不接 L298N，也不参与电机控制。PC5 已改作 `LIMIT_Z_MIN`。

