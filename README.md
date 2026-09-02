# Giantruck collection car

当前上传版本：`STM32F103VET6_collection_car_combat_v5.3`

这是 STM32F103VET6 控制端工程，使用 STM32CubeIDE 打开工程目录中的 `.project` 文件。

## 当前功能

- UART4 接收 OpenMV 视觉目标帧
- USART3 接收蓝牙单字符控制命令并返回状态
- 双轮电机 PWM、编码器和可选闭环 PID
- 视觉目标地图、滚动路径规划和对抗赛策略
- HC-SR04 前向安全停车与避障重规划
- 自动收集、倒车到卸货区、后舱门 MG995 舵机扫动卸货
- 初始扫描速度调整为 18%
- 新增有效物块时 PB8 蜂鸣器短响，并通过蓝牙返回 `TARGET FOUND`

## 重要硬件配置

- MCU：STM32F103VET6
- OpenMV 视觉串口：UART4，PC10/PC11，115200 8N1
- 蓝牙串口：USART3，PB10/PB11，115200 8N1
- 舱门舵机：TIM1_CH1，PA8
- 蜂鸣器默认：PB8，高电平有效

蜂鸣器引脚和有效电平需要按实际主板原理图确认。若实际蜂鸣器不是 PB8，请修改 `Core/Src/main.c` 顶部的 `BUZZER_PORT`、`BUZZER_PIN` 和电平宏。

## 验证

`tests/` 中的主机测试可在 macOS/Linux 上运行：

```sh
make -C tests test
```

详细修改记录见 `STM32F103VET6_collection_car_combat_v5.3/V5.3_CHANGELOG.md`。
