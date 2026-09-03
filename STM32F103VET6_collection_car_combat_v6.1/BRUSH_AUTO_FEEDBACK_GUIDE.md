# 前刷自动解卡与引脚配置

当前启动标记：`FW=v6.1+AUTO_UNJAM`。这是原 v6.1 工程的反馈修订，工程名及 ELF 仍为 `collection_car_combat_v6_1`。完整源代码可直接 Clean/Build，不必先用 CubeMX 重新生成代码。

## 1. 接线与已实现的配置

| 模块信号 | STM32 引脚 | 软件配置 |
|---|---|---|
| 前刷编码器 A | PB12 | EXTI12，上升沿+下降沿，上拉 |
| 前刷编码器 B | PB15 | EXTI15，上升沿+下降沿，上拉 |
| 电流传感器模拟 OUT（可选） | PA4 | 模拟输入，ADC1 通道 4 |
| 编码器/电流传感器 GND | GND | 与主板共地 |

前刷功率控制沿用 PB0/PB1 方向及 PC8/TIM8_CH3 PWM。反馈接的是前刷轴编码器，不能把车轮编码器替代接入。编码器供电按其规格，A/B 使用与主板兼容的逻辑电平；PA4 只接 0～3.3V 的传感器信号，不能接电机功率线。供电电压与信号电压并不是同一概念。

PB13/PB14 的 LED、PB6/PB7 与 PA0/PA1 的车轮编码器、蓝牙、视觉串口和后舵机引脚保留原配置。

PB12/PB15 采用外部中断软件正交解码，A/B 的两个边沿都计数（四倍频），不占用另一个定时器。非法的双比特同时跳变只计为 ENCERR，不当作转动；中断丢失也可能导致计数误差，因此高速编码器需实机检查错误率和计数准确性。

ADC1 由 `brush_feedback_hw.c` 独立初始化：PCLK2/6=12MHz、单通道、239.5周期采样、软件启动连续转换，主循环每 10ms 读取一次已完成结果。ADC1 当前由该模块独占，未添加缺失的 HAL ADC 驱动文件。

## 2. CubeMX 中应看到什么

项目 IOC 已同步以下配置：

1. PB12 选择 `GPIO_EXTI12`；GPIO 模式 `External Interrupt Mode with Rising/Falling edge trigger detection`，Pull-up。
2. PB15 选择 `GPIO_EXTI15`，同样双边沿及 Pull-up。
3. PA4 选择 `GPIO_Analog`，No pull。这里 ADC1 通道 4 的转换由反馈模块初始化，不需要额外生成另一个 ADC1 驱动。
4. NVIC 勾选 `EXTI line[15:10] interrupts`，抢占优先级 2、子优先级 0。

中断入口在 `stm32f1xx_it.c` 的 `EXTI15_10_IRQHandler()`，分别调用两个引脚的 HAL EXTI 处理；`HAL_GPIO_EXTI_Callback()` 在 `brush_feedback_hw.c` 读取 A/B 并解码。入口声明已加入 `stm32f1xx_it.h`。

如果重新生成代码，要保留 `brush_feedback*.c/.h`，检查初始化调用及中断入口仍在，并确认 ADC1 没有被另一个初始化函数重复配置。本轮核对了 IOC 文本与运行时引脚的一致性，没有在 CubeMX GUI 中执行代码再生成。

## 3. 第一次启用：不需要先知道编码器型号的标定方法

自动功能默认未启用，`BRUSH_COUNTS_PER_REV=0` 表示未知，不能猜测转速或准确反转圈数。

分别发送下面的单字符，每步等反馈再进行下一步，不能一次发送整串 `HNTKE`：

1. 发送 H：停轮、停刷并进入 HOLD。等刷轴完全静止。
2. 发送 N：把当前编码器计数设为诊断/标定零点。
3. 在刷轴做标记，手动沿同一方向转动实际刷轴一整圈，回到原标记。若减速器不能被手动带动，不要强扭，应根据编码器规格和传动比填写配置或在脱开传动后按传动关系标定。
4. 发送 T：程序把本次净计数的绝对值作为一圈 CPR。至少 4 个计数且过程中没有非法跳变才接受；T 只做标定，不启动电机。
5. 发送 V：查看 `CPR=...` 和 `ENCERR=0`。把 CPR 记下来。若手动转了多圈、没有转满一圈或打滑，程序无法判断，标定会不准确。
6. 发送 K：停顿后恢复前刷正转，车轮保持停车。
7. 等待约 1 秒后发 V，确认 `RPM_X10` 为正且合理。若正转时为负，互换 A/B 信号线，或把配置中的 `BRUSH_ENCODER_SIGN` 改为 -1 后重新编译。
8. 发送 E：校准值有效、前刷当前正转且最近测速达到阈值、无编码器错误时，返回 `AUTO_UNJAM ARMED`。随后可使用 A/C/D 开始任务。

T 的标定只保存在 RAM，断电恢复为头文件配置。若已知正确 CPR，也可直接填写配置后构建，跳过 H/N/T。CPR 必须是一圈实际刷轴对应的四倍频计数；厂家 PPR 如果仅指电机轴单相每圈脉冲，通常需要结合传动比和四倍频换算，但厂家已经标为四倍频 CPR 时不能重复乘 4。

## 4. 如何改配置并让它上电自动启用

修改 `Core/Inc/brush_feedback_config.h`：

```c
#define BRUSH_COUNTS_PER_REV 0U       /* 用实测 CPR 替换 0 */
#define BRUSH_ENCODER_SIGN 1          /* 正转应显示正 RPM */
#define BRUSH_AUTO_ENABLE_ON_BOOT 0U   /* 校准并验证后可改为 1 */
```

先填实测 CPR 再改上电启用位，不能在 CPR=0 时启用。编译期会拒绝这种无效配置。保持上电启用为 0 时，每次上电发送 E 即可；A/C/D 不关闭已启用的监测，S/X/0/O/H 会关闭它。

更换引脚也修改这个头文件中的以下定义，并同步 IOC：

```c
#define BRUSH_ENCODER_A_PORT GPIOB
#define BRUSH_ENCODER_A_PIN GPIO_PIN_12
#define BRUSH_ENCODER_B_PORT GPIOB
#define BRUSH_ENCODER_B_PIN GPIO_PIN_15
#define BRUSH_ENCODER_GPIO_CLOCK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BRUSH_ENCODER_IRQn EXTI15_10_IRQn
#define BRUSH_CURRENT_PORT GPIOA
#define BRUSH_CURRENT_PIN GPIO_PIN_4
#define BRUSH_CURRENT_GPIO_CLOCK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BRUSH_CURRENT_ADC_CHANNEL 4U
```

- A/B 必须使用不同 EXTI 线号，不能用 PA12 和 PB12 组成一对。不同 GPIO 端口需启用各自的 GPIO 时钟。
- 保持两个编码器输入都在线号 10～15 时，可以沿用当前 IRQ 组；改到其他组必须同步中断入口/头文件/NVIC。如果两个引脚分属不同组，要分别建立并启用两个 IRQ，当前单组初始化不能只改一个宏就覆盖两个组。
- 修改电流引脚必须同时改 ADC 通道号。例如 PA4→PA5，需要 `GPIO_PIN_5` 及通道 `5U`，IOC 改为 PA5 模拟输入并释放 PA4。通道号是 ADC 通道索引，不是 GPIO 位掩码。
- 不要只改 IOC 或只改头文件；两处要一致。默认接线已完成配置，无需修改。

## 5. 自动触发与两圈控制

配置的初始判据为：前刷正转命令有效，排除启动后的 1000ms，转速低于 5rpm 持续 600ms。前刷此前必须曾被测到正常正转，才将低速作为可以解卡的事件。上电自动启用却从未检测到转动时，会报 NO_FEEDBACK 并保持停刷，需要排查传感器或初始卡滞。

自动解卡使用原有 J 流程：停刷 150ms，反转同时后退，反向净计数达到 `2×CPR` 时停刷，再停顿 150ms 恢复正转。软件是在达到计数目标时撤销 PWM，机械惯性及处理延迟会带来实际停转角误差；它不等于高精度位置伺服。

反转期间 800ms 没有新的反向计数进展，或 6s 仍未完成两圈，或出现非法跳变，则停轮停刷并关闭自动触发，保持 HOLD。反向计数必须方向正确，向前转或来回抖动不会简单累加为两圈。较慢的机构需要按实测调整这些限制。

后退仍以任一车轮累计轮程 80mm 或 600ms 为上限；车轮停车后前刷可继续完成两圈。当前没有后向测距，不能根据后方障碍物决定是否允许倒车。

连续自动尝试最多两次。两次正常结束后如果仍再次判定卡滞，会保持 HOLD；只有正常正转持续约 10s 才恢复尝试额度。反馈故障、反转超时和解卡期间再次过流会直接 HOLD。编码器断线和机械堵转不能仅靠转速完全区分，有限重试及 HOLD 用于限制这种不确定性。

J 仍可手动触发：CPR 有效时同样数两圈，不要求先发 E；CPR 未校准时沿用旧版定时反转。正常结束保留原任务位置、已收集记录、软件载货及对抗赛计时；D 等新帧，A/C 重扫。自动事件发生在卸货阶段时停刷并取消运动，等待人工处理，不在卸货过程中强行反转后退。

## 6. PA4 电流功能怎样启用

不接电流传感器时保持 `BRUSH_CURRENT_ENABLE=0`，只靠编码器工作。PA4 仍会给出 ADC/MV 原始诊断读数，但未连接的读数没有电流意义，MA=-1 表示未配置。ADC_OK=1 只表示最近完成过 ADC 转换，不证明电流传感器已经正确接线。

要使用电流判据，在头文件中根据模块和实测填写：

```c
#define BRUSH_CURRENT_ENABLE 1U
#define BRUSH_CURRENT_ZERO_MV ...     /* 零电流时 PA4 的实际电压，单位 mV */
#define BRUSH_CURRENT_MV_PER_AMP ...   /* PA4 电压变化量 / 电流变化量，mV/A */
#define BRUSH_CURRENT_STALL_MA ...    /* 根据正常负载与异常样本确定，单位 mA */
```

上面省略号必须替换成数值，不能原样编译。若加了分压或放大，换算必须使用最终到达 PA4 的电压；默认 1650mV 是占位值，不是对具体传感器的假设。输入按 0～3.3V 设计。

电流判据独立于低速判据：过了启动宽限期，电流持续超过阈值 150ms 可触发解卡。启用后 ADC 数据超过 300ms 没更新会保持停刷。它属于任务级软件保护，不能替代驱动器硬件限流和快速短路保护。

## 7. 指令和状态

| 指令 | 功能 |
|---|---|
| H | 随时停轮停刷、关闭自动触发，进入标定/故障 HOLD；接收队列中有停止优先级 |
| N | 仅 HOLD：将当前计数/错误数设为诊断及标定基线 |
| T | 仅 HOLD：把从 N 开始手动一圈的净计数设为 CPR，存入 RAM |
| K | HOLD 中停顿后恢复前刷正转，保持车轮停车；不会自动重新启用保护 |
| E | 校准和当前反馈满足条件时启用自动解卡 |
| O | 关闭自动触发；如果正在解卡则同时停轮停刷并 HOLD |
| J | 手动执行一次解卡，重复发送不延长动作 |
| S/X/0 | 关闭自动触发并停止驾驶；如果正在解卡则同时停刷，普通运行时仍保持前刷正转 |
| V | 原车辆状态，以及反馈和解卡状态；HOLD/解卡期间只回简短相关状态 |

```text
BRUSH_FB COUNT=120 CPR=40 RPM_X10=600 ENCERR=0 ADC=2048 MV=1650 MA=-1 ADC_OK=1 AUTO=1 EVENT=NONE
REV_CONTROL=ENCODER REV_COUNT=80 AUTO_CYCLE=1 ATTEMPTS=1
```

示例 CPR=40 仅用于解释：RPM_X10=600 是 60rpm，REV_COUNT=80 对应两圈。实际值以你标定的 CPR 为准。COUNT 为 N 以来的有符号净计数；ENCERR 为 N 以来的非法跳变数。

事件可能为 LOW_RPM、HIGH_CURRENT、NO_FEEDBACK、ENCODER_ERRORS、ENCODER_SIGN、ADC_LOST。遇到 HOLD 先看事件和接线，再决定 J 重试或 K 恢复；修复后重新发 E。

## 8. 模块职责和验证范围

- `brush_feedback_config.h`：引脚、CPR、方向、启动策略和判据参数。
- `brush_feedback.c`：四倍频解码、转速/电流换算、标定及持续异常判断，独立于 STM32 寄存器。
- `brush_feedback_hw.c`：GPIO/EXTI 初始化、A/B 采样、ADC1 初始化及轮询。
- `main.c`：蓝牙指令、自动事件到解卡状态机的衔接、重试额度、反向计数目标和停止控制。
- `stm32f1xx_it.c/.h`：共享 EXTI15_10 入口。

已通过宿主反馈模块测试（含电流启用/关闭）、真实 main.c 函数联动回归，以及硬件相关 C 文件语法检查。原有视觉/规划/卸货回归仍通过。原超声测距已修正为先减原始 DWT 周期再换算，测试覆盖回绕和无回波超时，防止主循环长时间阻塞而影响堵转监测。

未执行 ARM 链接构建、CubeMX GUI 再生成或实车试验。需要验证真实编码器信号速率、方向、CPR、带载转速/电流和实际刹停角度。

硬件初始化参考：[STM32F103xC/D/E 数据手册](https://www.st.com/resource/en/datasheet/stm32f103ze.pdf)、[ST 官方 ADC 驱动](https://github.com/STMicroelectronics/stm32f1xx-hal-driver/blob/master/Src/stm32f1xx_hal_adc.c)。
