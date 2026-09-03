# Giantruck collection car

当前版本：**v6.1 + UNJAM 前刷反转解卡修订**。

蓝牙 `A`=技术赛收集，`C`=对抗赛，`D`=单帧视觉直接跟随，`S`=退出任务并停车，`V`=查看状态和原始目标质量。目标提示已由蜂鸣改为蓝牙 `SEEN ... Q=...`。使用说明见 [调试模式与质量诊断](STM32F103VET6_collection_car_combat_v6.1/DEBUG_MODE_GUIDE.md)。

前刷正常以 50% PWM 持续正转，后舵机单独执行卸货。新增 `J` 手动触发前刷反转并短距离后退；中途 `S/X/0` 会同时停轮、停刷并保持 HOLD，`J` 重试、`K` 恢复正转。解卡以外的 S/X 仍只停轮。操作、参数及“两圈”的标定限制见 [前刷解卡说明](STM32F103VET6_collection_car_combat_v6.1/BRUSH_RECOVERY_GUIDE.md)；完整变更见 [Changelog](CHANGELOG.md)。

这是 STM32F103VET6 控制端工程，当前目录为 `STM32F103VET6_collection_car_combat_v6.1`，CubeIDE 工程名为 `collection_car_combat_v6_1`，CubeMX 配置为 `collection_car_combat_v6_1.ioc`。

升级时在 STM32CubeIDE 中导入这个新工程，Clean 后重新 Build，并新建或检查烧录配置，确认程序路径指向新工程的 `Debug/collection_car_combat_v6_1.elf`（Release 构建则使用 Release 目录）。不要继续烧录旧 v5 工程的 ELF。启动蓝牙提示中的 `FW=v6.1+UNJAM` 可用于核对运行版本。

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
- J 解卡：停刷缓冲 → 反转并后退 → 停刷缓冲 → 正转；后退用车轮编码器和时间限制，前刷两圈为定时估算，不具备自动堵转检测

## 重要硬件配置

- MCU：STM32F103VET6
- OpenMV 视觉串口：UART4，PC10/PC11，115200 8N1
- 蓝牙串口：USART3，PB10/PB11，115200 8N1
- 前刷：PB0/PB1 控制方向，PC8/TIM8_CH3 输出 PWM，默认 50%
- 舱门舵机：TIM1_CH1，PA8
- 原蜂鸣器 PB8 保持无效电平，目标提示改走蓝牙

前刷常转有解卡例外：中断解卡后的 HOLD 会一直停刷，需 J 重试或 K 恢复；退出 HOLD 前拒绝普通驾驶与模式切换指令。

## 验证

`tests/` 中的主机测试可在 macOS/Linux 上运行：

```sh
make -C STM32F103VET6_collection_car_combat_v6.1/tests test
```

软件测试覆盖控制决策和模拟输出，尚未完成本版 ARM 构建或实车验证。
