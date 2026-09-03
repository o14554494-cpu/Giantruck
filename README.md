# Giantruck collection car

当前版本：**v6.1 + AUTO_UNJAM 前刷反馈与自动解卡修订**。

已接入 PB12/PB15 前刷 A/B 编码器、PA4 可选模拟电流反馈。先校准每圈计数，再发 E 启用自动解卡；默认不在未知编码器参数下自动后退。完整接线、CubeMX 配置和 H/N/T/K/E 校准流程见 [自动解卡与引脚配置](STM32F103VET6_collection_car_combat_v6.1/BRUSH_AUTO_FEEDBACK_GUIDE.md)。

蓝牙 `A`=技术赛收集，`C`=对抗赛，`D`=单帧视觉直接跟随，`S`=退出任务并停车，`V`=查看状态和原始目标质量。目标提示已由蜂鸣改为蓝牙 `SEEN ... Q=...`。使用说明见 [调试模式与质量诊断](STM32F103VET6_collection_car_combat_v6.1/DEBUG_MODE_GUIDE.md)。

前刷正常以 50% PWM 持续正转，后舵机单独执行卸货。J 仍可手动解卡；校准后采用反向编码器计数到两圈，未校准时 J 保留定时方式。E 启用持续低速/可选过流触发，O 关闭；S/X/0 同时关闭自动触发，解卡中还会停刷并保持 HOLD。H 可随时停轮停刷，K 从 HOLD 恢复正转。完整变更见 [Changelog](CHANGELOG.md)。

这是 STM32F103VET6 控制端工程，当前目录为 `STM32F103VET6_collection_car_combat_v6.1`，CubeIDE 工程名为 `collection_car_combat_v6_1`，CubeMX 配置为 `collection_car_combat_v6_1.ioc`。

升级时在 STM32CubeIDE 中导入这个新工程，Clean 后重新 Build，并新建或检查烧录配置，确认程序路径指向新工程的 `Debug/collection_car_combat_v6_1.elf`（Release 构建则使用 Release 目录）。启动蓝牙提示中的 `FW=v6.1+AUTO_UNJAM` 可用于核对运行版本。

`V5.3_CHANGELOG.md`、`V5.3_FIXES_20260902.md` 和 `V6_BRUSH_CHANGELOG.md` 是历史版本记录，故意保留原版本号；不代表当前固件版本。请以本 README 和根目录 CHANGELOG 为准。

## 当前功能

- UART4 接收 OpenMV 视觉目标帧
- USART3 接收蓝牙单字符控制命令并返回状态
- 双轮电机 PWM、编码器和可选闭环 PID
- 视觉目标地图、滚动路径规划和对抗赛策略
- HC-SR04 前向安全停车与避障重规划
- 前刷独立持续正转收集，倒车到卸货区后由后舱门 MG995 舵机扫动卸货
- 车轮非零档位映射到 60%～100%；保留用户 v6 新增的 80%、200ms 启动补偿实现及其已知限制（见 Changelog）
- 蓝牙输出视觉目标数量、原始质量和过滤原因；蜂鸣器静音
- D 调试模式单帧跟随一个目标，无质量/确认次数门槛，不规划路线
- J/自动解卡：停刷缓冲 → 反转并后退 → 停刷缓冲 → 正转；后退有轮程及超时限制，校准后的前刷以反向计数控制两圈
- 前刷低速持续确认、启动宽限期、最多两次连续自动尝试、反馈异常与反转无进展停刷；电流保护需配置后才启用

## 重要硬件配置

- MCU：STM32F103VET6
- OpenMV 视觉串口：UART4，PC10/PC11，115200 8N1
- 蓝牙串口：USART3，PB10/PB11，115200 8N1
- 前刷：PB0/PB1 控制方向，PC8/TIM8_CH3 输出 PWM，默认 50%
- 前刷编码器：PB12=A、PB15=B，双边沿 EXTI 软件正交解码；电流模拟输出 PA4=ADC1_IN4（0～3.3V）
- 舱门舵机：TIM1_CH1，PA8
- 原蜂鸣器 PB8 保持无效电平，目标提示改走蓝牙

配置集中在 `Core/Inc/brush_feedback_config.h`。每圈计数默认 0，电流保护默认关闭；模块初始化和原始反馈已实现。H/解卡异常后的 HOLD 一直停刷，K 恢复后可用 E 重新启用。普通 S/X/0 仍保留前刷正转，但会关闭自动触发。

## 验证

`tests/` 中的主机测试可在 macOS/Linux 上运行：

```sh
make -C STM32F103VET6_collection_car_combat_v6.1/tests test
```

软件测试覆盖控制决策和模拟输出，尚未完成本版 ARM 构建或实车验证。
