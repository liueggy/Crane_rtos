# Crane_rtos

基于 STM32F103ZET6、FreeRTOS、CubeMX 和 CMake 的起重式散料货物搬运机器人控制工程。

项目面向物流技术创意赛运输豆子任务，目标是让机器人完成自主识别、取料、绕障运输、准确放料和归位。当前工程已经完成硬件控制框架和分模块调试基础，完整视觉搬运流程仍需结合实机继续开发。

## 1. 项目状态

当前已经具备：

- 双 TB6600 步进轴的启停、方向和使能控制框架；Z 轴已完成实机调试。
- 四路 L298N 直流电机双 PWM 驱动结构；M1 已完成正反转、换挡和编码器测速验证。
- M1 单电机 PI 控速调试页面。
- 两路 270 度舵机 PWM 和 USART2 调试命令。
- K230 USART2 DMA 收发框架。
- OLED 中文调试界面。
- 急停、四路对射光电限位和 EXTI 事件框架。
- FreeRTOS 任务框架、路线表和比赛状态机骨架。

当前尚未完成：

- M2 至 M4 的完整实机验证和四轮同步标定。
- X/Z 轴双次回零与指定位置闭环。
- K230 视觉结果协议、多帧确认和任务表生成。
- 底盘路线到点、绕障和自动运输实机联调。
- 夹爪/料斗取料、锁料、倾倒动作确认。

自动状态机目前默认停在 `VISION_SCAN`，不会在硬件未接齐时自动驱动机器人跑路线。

## 2. 工程结构

```text
Crane_rtos/
├── Core/Inc/                 头文件和 CubeMX 生成接口
├── Core/Src/                 应用代码与 CubeMX 生成代码
├── Drivers/                  STM32 HAL/CMSIS 驱动
├── Middlewares/              FreeRTOS
├── cmake/                    ARM GCC 和 CubeMX CMake 配置
├── Crane_rtos.ioc            CubeMX 配置文件
├── CMakeLists.txt            应用源文件和构建入口
├── CMakePresets.json         Debug/Release 预设
├── STM32F103XX_FLASH.ld      STM32F103xE 链接脚本
└── docs/项目配置与开发进度.md  唯一项目配置与进度文档
```

业务代码按模块拆分，`main.c` 主要保留外设初始化、HAL 回调转发和 FreeRTOS 任务入口：

| 模块 | 文件 | 职责 |
| --- | --- | --- |
| 舵机 | `servo_control.c/.h` | 角度、脉宽和限幅 |
| 步进轴 | `stepper_axis.c/.h` | X/Z 轴使能、方向和停止 |
| 底盘 | `chassis_motion.c/.h` | 四轮调速、手动调试和路线段接口 |
| 安全 | `safety_manager.c/.h` | 急停、限位和故障停机 |
| 输入 | `input_manager.c/.h` | 按键消抖和输入事件 |
| K230 | `k230_link.c/.h` | USART2 DMA、命令和视觉接口 |
| 路线 | `robot_routes.c/.h` | 取货、绕障、放料、归位路线表 |
| 状态机 | `robot_controller.c/.h` | 比赛流程控制 |
| 状态/参数 | `app_state.c/.h`、`app_config.c/.h` | 运行状态和统一参数 |

## 3. 开发环境

### 必需工具

| 工具 | 用途 | 要求 |
| --- | --- | --- |
| Git | 克隆与版本管理 | 能执行 `git --version` |
| CMake | 生成构建文件 | 3.22 或更高 |
| Ninja | CMake 构建生成器 | 在 `PATH` 中 |
| GNU Arm Embedded GCC | 编译 STM32 | `arm-none-eabi-gcc` 在 `PATH` 中 |
| STM32CubeMX | 打开/修改 `.ioc` 并生成代码 | 安装任意可用版本，建议与项目生成版本相近 |
| STM32CubeProgrammer | 下载固件 | 仅烧录时需要 |
| ST-LINK 驱动 | SWD 下载/调试 | 仅连接开发板时需要 |

当前工程的 CMake 工具链没有写死 GCC 安装目录，使用以下命令查找工具：

```powershell
arm-none-eabi-gcc --version
cmake --version
ninja --version
```

如果 `arm-none-eabi-gcc` 找不到，需要把 GNU Arm Embedded Toolchain 的 `bin` 目录加入当前用户或系统 `PATH`，然后重新打开终端。

Windows 下可使用 STM32CubeCLT 自带的 GCC。不同电脑的安装目录可能不同，README 不要求固定为某个盘符。

## 4. 克隆后首次构建

在仓库根目录执行：

```powershell
git clone <仓库地址>
cd Crane_rtos
cmake --preset Debug
cmake --build --preset Debug
```

或者使用已经生成的构建目录：

```powershell
cmake --build build\Debug
```

成功后主要产物为：

```text
build/Debug/Crane_rtos.elf
```

Release 构建：

```powershell
cmake --preset Release
cmake --build --preset Release
```

如果首次构建报 `arm-none-eabi-gcc is not recognized`，这是本机工具链 PATH 问题，不是项目源码问题。若报 `ninja is not recognized`，安装 Ninja 并加入 PATH，或在 CMake 中改用本机可用的生成器。

## 5. CubeMX 配置与重新生成

CubeMX 工程文件为：

```text
Crane_rtos.ioc
```

打开后修改引脚、定时器、DMA、FreeRTOS 或 NVIC 配置，再点击 **Generate Code**。生成完成后必须重新构建：

```powershell
cmake --build --preset Debug
```

生成后至少检查：

- `Core/Inc/main.h` 的标签与预期一致。
- `Core/Src/main.c` 中 `MX_TIM8_Init()` 存在。
- TIM3/TIM8 四通道 PWM 存在。
- USART2 DMA RX/TX 和空闲线接收存在。
- `PC5 / LIMIT_Z_MIN` 为 `GPIO_MODE_IT_RISING + GPIO_PULLDOWN`。
- `PE0/PE1/PE2` 同样为高有效限位输入。
- `EXTI9_5_IRQHandler` 同时转发 `LIMIT_Z_MIN_Pin` 和 `ESTOP_IN_Pin`。

CubeMX 可能重写生成区域，因此手写业务应放在 `USER CODE` 区域或独立 `.c/.h` 文件。修改 `.ioc` 后不要只修改生成的 `main.c`，否则下次生成可能丢失配置。

## 6. 烧录与调试

项目当前没有把 STM32CubeProgrammer 或 ST-LINK 的绝对路径写入 CMake，也没有固定烧录脚本。不同电脑只需要在本机安装工具并替换命令路径。

推荐使用 SWD：

| ST-LINK | STM32 |
| --- | --- |
| SWDIO | PA13 |
| SWCLK | PA14 |
| GND | GND |
| 3.3V | 目标板 3.3V 检测/供电，按调试器要求连接 |
| NRST | NRST，可选但建议连接 |

生成 Intel HEX 或 BIN：

```powershell
arm-none-eabi-objcopy -O ihex build\Debug\Crane_rtos.elf build\Debug\Crane_rtos.hex
arm-none-eabi-objcopy -O binary build\Debug\Crane_rtos.elf build\Debug\Crane_rtos.bin
```

使用 STM32CubeProgrammer CLI 烧录时，命令格式通常为：

```powershell
STM32_Programmer_CLI.exe -c port=SWD -w build\Debug\Crane_rtos.elf -v -rst
```

如果命令找不到，请使用本机 STM32CubeProgrammer 安装目录中的 `STM32_Programmer_CLI.exe` 的绝对路径。也可以使用 STM32CubeProgrammer 图形界面选择 `Crane_rtos.elf`、连接方式 `ST-LINK/SWD` 后下载。

烧录前确认：目标板供电正常、ST-LINK 电压与目标板一致、所有电机驱动器处于安全停止状态。

## 7. 硬件基线

### 直流电机与编码器

| 电机 | IN1 PWM | IN2 PWM | 编码器 A | 编码器 B |
| --- | --- | --- | --- | --- |
| M1 | PA6 / TIM3_CH1 | PA7 / TIM3_CH2 | PD10 | PD4 |
| M2 | PB0 / TIM3_CH3 | PB1 / TIM3_CH4 | PD13 | PD5 |
| M3 | PC6 / TIM8_CH1 | PC7 / TIM8_CH2 | PD14 | PD6 |
| M4 | PC8 / TIM8_CH3 | PC9 / TIM8_CH4 | PD15 | PD7 |

L298N 的 EN1 至 EN4 只保持使能，不承担 PWM。正转为 `IN1=PWM, IN2=0`，反转为 `IN1=0, IN2=PWM`。

编码器线色资料：黑线 GND、蓝线 3.3V、黄线 A 相、绿线 B 相。输出轴每圈计数 `374` 目前是暂定参数，闭环使用前必须实测。

### 步进轴

| 轴 | PUL | DIR | ENA |
| --- | --- | --- | --- |
| X | PE9 / TIM1_CH1 | PD2 | PD11 |
| Z | PE11 / TIM1_CH2 | PD3 | PD12 |

TB6600 ENA 低电平有效。Z 轴当前使用 4 细分、800 脉冲/圈、TIM1 `PSC=71`、`ARR=499`、约 2kHz。

### 舵机、OLED、K230

- 舵机 1：PA0 / TIM2_CH1；舵机 2：PA1 / TIM2_CH2；50Hz；独立 5V/6V 供电。
- OLED：PB6/PB7 / I2C1，SSD1306，常用地址 0x3C。
- K230：PA2 为 USART2_TX，PA3 为 USART2_RX，115200 8N1，DMA RX/TX，双方 TX/RX 交叉并共地。

### 按键、急停、对射光电

- K0：PE4，低有效下降沿；K1：PE3，低有效下降沿；急停：PE7，低有效下降沿。
- X 最小/最大：PE0/PE1；Z 上/下：PE2/PC5。
- 光电模块无遮挡 `DO=低`、遮挡 `DO=高`，四路限位使用内部下拉和上升沿 EXTI。
- 光电模块只有在 DO 高电平不超过 3.3V 时才能直接连接 STM32；高于 3.3V 必须做电平转换。

## 8. FreeRTOS 任务

当前任务来自 `Crane_rtos.ioc`：

| 任务 | 入口 | 优先级 | 栈配置 |
| --- | --- | ---: | ---: |
| APP_CTRL | `StartAppCtrlTask` | 24 | 256 words |
| DC_MOTOR | `StartDcMotorTask` | 32 | 256 words |
| STEPPER | `StartStepperTask` | 32 | 256 words |
| K230_RX | `StartK230RxTask` | 24 | 256 words |
| INPUT_EVT | `StartInputEvtTask` | 32 | 256 words |
| OLED | `StartOledTask` | 8 | 256 words |

FreeRTOS 动态堆当前为 12288 字节。限位和急停的硬件中断只做快速转发，消抖和业务处理在任务/安全模块中完成。

## 9. K230 运行文件

K230 Python 程序不参与 STM32 CMake 构建。K230 端需要单独准备：

```text
/sdcard/number/main.py
/sdcard/number/deploy_config.json
/sdcard/number/<kmodel文件>
```

模型路径、类别、输入尺寸和阈值由 `deploy_config.json` 决定。K230 的串口编号属于 K230 自己的硬件定义，不等同于 STM32 的 USART 编号；当前物理连接对应 STM32 USART2。

## 10. 新开发者建议流程

1. 克隆仓库并确认 Git、CMake、Ninja、ARM GCC 都能在命令行执行。
2. 使用 `cmake --preset Debug` 和 `cmake --build --preset Debug` 编译。
3. 打开 `Crane_rtos.ioc`，确认生成配置和引脚标签，再生成代码并重新编译。
4. 不接电机时先烧录，确认 OLED、按键、串口和限位输入。
5. 单独验证 Z 轴、舵机、M1，再逐步接入 M2 至 M4。
6. 校准编码器每圈计数、极性、底盘直行和光电限位位置。
7. 最后接入 K230 视觉结果、路线参数和自动比赛状态机。
