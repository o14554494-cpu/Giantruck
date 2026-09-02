# Giantruck collection car

当前版本：**v6 前刷常转、后舵机卸货版**。

前刷在初始化完成后以 50% PWM 持续正转，后舵机单独执行卸货。视觉失联只停车轮；蓝牙 S/X 也不关闭前刷。完整变更见 [Changelog](CHANGELOG.md) 和 [v6 前刷修改说明](STM32F103VET6_collection_car_combat_v5.3/V6_BRUSH_CHANGELOG.md)。

这是 STM32F103VET6 控制端工程，工程内部仍沿用收到的 v6 包中的 `STM32F103VET6_collection_car_combat_v5.3` 名称。使用 STM32CubeIDE 导入工程目录并重新构建烧录。

## 当前功能

- UART4 接收 OpenMV 视觉目标帧
- USART3 接收蓝牙单字符控制命令并返回状态
- 双轮电机 PWM、编码器和可选闭环 PID
- 视觉目标地图、滚动路径规划和对抗赛策略
- HC-SR04 前向安全停车与避障重规划
- 前刷独立持续正转收集，倒车到卸货区后由后舱门 MG995 舵机扫动卸货
- 车轮非零档位映射到 60%～100%；保留用户 v6 新增的 80%、200ms 启动补偿实现及其已知限制（见 Changelog）
- 新增有效物块时 PB8 蜂鸣器短响，并通过蓝牙返回 `TARGET FOUND`

## 重要硬件配置

- MCU：STM32F103VET6
- OpenMV 视觉串口：UART4，PC10/PC11，115200 8N1
- 蓝牙串口：USART3，PB10/PB11，115200 8N1
- 前刷：PB0/PB1 控制方向，PC8/TIM8_CH3 输出 PWM，默认 50%
- 舱门舵机：TIM1_CH1，PA8
- 蜂鸣器默认：PB8，高电平有效

蜂鸣器引脚和有效电平需要按实际主板原理图确认。若实际蜂鸣器不是 PB8，请修改 `Core/Src/main.c` 顶部的 `BUZZER_PORT`、`BUZZER_PIN` 和电平宏。

## 验证

`tests/` 中的主机测试可在 macOS/Linux 上运行：

```sh
make -C STM32F103VET6_collection_car_combat_v5.3/tests test
```

软件测试覆盖控制决策和模拟输出，尚未完成本版 ARM 构建或实车验证。
