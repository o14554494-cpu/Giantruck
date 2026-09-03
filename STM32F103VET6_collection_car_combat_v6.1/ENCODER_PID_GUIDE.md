# 编码器测速与电机 PID 使用说明

## 当前引脚分配

| 功能 | STM32 引脚 | 外设 |
|---|---|---|
| 左电机 PWM | PA6 | TIM3_CH1 |
| 右电机 PWM | PA7 | TIM3_CH2 |
| 左编码器 A/B | PB6 / PB7 | TIM4_CH1 / CH2 |
| 右编码器 A/B | PA0 / PA1 | TIM5_CH1 / CH2 |
| 电机方向 | PC0～PC3 | GPIO |
| 串口 | PC10 / PC11 | UART4，115200 |

PB6 已分配给左编码器，因此卸货舵机不能再使用 PB6。

## 上电前必须确认

1. 编码器输出电平必须适合 STM32 的 3.3 V 输入；不确定时使用电平转换。
2. 编码器与 STM32 必须共地。
3. 先让车轮悬空测试，确认测速方向和电机方向正确。
4. 电机电源不要直接从 STM32 开发板取电。

## 第一步：只测试测速

`main.c` 中默认：

```c
#define MOTOR_CLOSED_LOOP_ENABLE 0U
```

此时电机仍按原来的 PWM 开环运行，但 TIM4、TIM5 会统计编码器并计算 RPM。
通过 UART4 发送 `V`，返回示例：

```text
RPM L=48 R=45 PWM L=40 R=40 PID=OFF
```

如果车轮正转时 RPM 为负，把对应参数从 `1.0f` 改为 `-1.0f`：

```c
#define LEFT_ENCODER_SIGN   1.0f
#define RIGHT_ENCODER_SIGN  1.0f
```

如果 RPM 数值明显不正确，修改：

```c
#define ENCODER_COUNTS_PER_WHEEL_REV 1560.0f
```

这里需要填写车轮转一整圈时，定时器实际累计的计数值。使用 TI12 四倍频时，它通常等于编码器标称脉冲数乘减速比再乘 4，但最终以实测为准。

## 第二步：启用 PID

测速确认正确以后改为：

```c
#define MOTOR_CLOSED_LOOP_ENABLE 1U
```

再填写空载时车轮大致最高转速：

```c
#define MOTOR_MAX_RPM 120.0f
```

初始 PID 参数：

```c
#define MOTOR_PID_KP 0.45f
#define MOTOR_PID_KI 1.20f
#define MOTOR_PID_KD 0.00f
```

调参顺序：

1. 先令 `KI=0`、`KD=0`，逐渐增大 `KP`，直到速度响应较快但不持续振荡。
2. 再小幅增加 `KI`，消除长期速度误差。
3. 通常小车轮速控制暂时不需要 `KD`；只有超调明显时再少量增加。

## 串口命令

| 命令 | 功能 |
|---|---|
| F | 前进 |
| B | 后退 |
| L | 原地左转 |
| R | 原地右转 |
| S | 停止 |
| 0～9 | 速度设为 0%～90% |
| A | 启动自动扫描、规划和收集 |
| X | 停止自动模式并清空地图 |
| M | 打印目标地图 |
| P | 打印当前规划路线 |
| V | 查询左右轮 RPM、PWM、PID、自动状态、位姿和协议状态 |

手动运动命令超过 1 秒没有刷新时，程序仍会自动停车；自动模式由 STM32 状态机持续控制，不受该超时影响。视觉联调、通信协议与路径规划说明见 `VISION_CONTROL_PLANNER_GUIDE.md`。
