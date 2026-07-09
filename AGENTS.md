# AGENTS.md

本文件是 Crane_rtos 仓库内 Codex/Agent 的本地协作约定。若上层系统指令与本文冲突，以上层系统指令为准；若无冲突，修改本仓库代码、配置、文档和提交信息时遵守本文。

## 提交信息语言

除非用户明确要求其他语言，所有 commit message 必须使用中文，包括：

- 第一行意图标题
- 正文说明
- trailer 的值

为了兼容 git trailer 生态，trailer key 保持英文，trailer value 使用中文。

推荐格式：

```text
<中文意图标题：说明为什么改，而不是只说改了什么>

<中文正文：说明背景、约束、方案取舍和影响。>

Constraint: <中文约束说明>
Rejected: <中文替代方案> | <中文拒绝原因>
Confidence: <低|中|高>
Scope-risk: <窄|中|宽>
Directive: <中文后续维护提醒>
Tested: <中文验证内容>
Not-tested: <中文未验证风险>
```

示例：

```text
建立硬件引脚基线

在重构 FreeRTOS 任务模型前，先保存当前 CubeMX 生成的硬件配置状态。
这次提交保留 OLED、K230 串口、双步进、双舵机、四路直流电机 PWM、
编码器 GPIO/EXTI 输入、限位和状态输出的工作基线。

Constraint: 用户要求先建立远程可回退点
Constraint: CubeMX .ioc 与生成代码必须保持一致
Rejected: 纳入 .omo 缓存 | 这是本地索引/工具状态，不属于项目源码
Confidence: 高
Scope-risk: 中
Directive: 后续修改引脚后必须从 Crane_rtos.ioc 重新生成，并谨慎同步 USER CODE 区域
Tested: cmake --build build\Debug
Not-tested: 尚未在 STM32 实板运行验证
```

## CubeMX 与代码同步

修改 CubeMX 相关配置时，优先修改 `Crane_rtos.ioc`，再让 CubeMX 生成代码，最后同步 `USER CODE` 区域中的业务逻辑。

涉及引脚、外设、FreeRTOS 任务、NVIC、时钟、DMA、串口、定时器等配置时，必须同时检查：

- `Crane_rtos.ioc`
- `Core/Inc/main.h`
- `Core/Src/main.c`
- `Core/Src/stm32f1xx_it.c`
- `Core/Src/stm32f1xx_hal_msp.c`
- `cmake/stm32cubemx/CMakeLists.txt`

每次配置或代码变更后，至少运行：

```powershell
cmake --build build\Debug
```

## 当前工程方向

本项目面向物流技术创意赛散料豆子搬运机器人。代码结构应优先服务于：

- `APP_CTRL`：总状态机和比赛流程
- `DC_MOTOR`：四轮直流电机控制
- `STEPPER`：双步进轴控制
- `K230_RX`：K230 视觉结果接收
- `INPUT_EVT`：按键、限位、急停事件
- `OLED`：调试显示

新增功能时尽量保持 `.ioc`、任务命名、入口函数和业务模块边界一致。
