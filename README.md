# Giantruck · 个人赛固件

本分支为 `personal-v7.6.3`，面向个人技术挑战赛的自动收集与黑区卸货。有效的 STM32CubeIDE 工程位于 [`STM32F103VET6_collection_car_v7.6.3_personal_strict_imu_frames/`](STM32F103VET6_collection_car_v7.6.3_personal_strict_imu_frames/)，其 README 和 [`PERSONAL_PERIODIC_IMU_GUIDE.md`](STM32F103VET6_collection_car_v7.6.3_personal_strict_imu_frames/PERSONAL_PERIODIC_IMU_GUIDE.md) 以实际代码为准。

> `Core/Src/main.c` 是固件启动入口，必须保留。仓库中的 `STM32F103VET6_collection_car_combat_v6.1/` 是旧版历史存档，不属于个人赛 v7.6.3 的烧录工程。

## 一句话流程

```text
上电静止采集10帧严格 IMU 初始零点
  → 自动启动 Greedy
  → 6×60°扫描 → 锁定一个目标 → 视觉横向对准并收集
  → 拾取/解卡后刷新短期直线参考并重扫
  → 空扫描时按个人赛返回流程寻找黑区
  → 黑色占比达到阈值 → 对齐、倒车、舵机开舱门并抖动卸货
```

## 运行与控制

- 上电后程序不等待蓝牙，先确认车体静止并锁定 10 帧 `START_RAW`，随后自动进入 Greedy。
- Greedy 采用“看一个、捡一个、再扫描”的相机相对控制，不依赖全场最短路径；目标进入视觉盲区后按编码器多走一段距离再结束拾取动作。
- 个人赛固件通过 `COMPETITION_GREEDY_ONLY=1` 限制蓝牙为安全/观察用途：`V` 查看状态，`S` 停轮，`X/0` 停止并清理任务，`H` 保持刷子故障状态；其他旧模式命令被明确忽略。
- 前刷在自动收集时独立持续正转；后舱门舵机只在卸货状态动作。

## IMU、视觉与容错

- 初始参考只在上电静止阶段建立，整场不被周期校准覆盖，用于最终返回和卸货方向。
- 短期 `LOCAL_RAW` 参考每 20 s 到期，但只在下一次安全扫描停车边界停止约 300 ms、采集 6 帧稳定数据后更新；拾取和解卡完成后也会进入同一刷新流程。
- `imu_navigation.c` 对过期、跳变过大、角速度异常或与编码器不一致的帧进行拒绝；恢复需要连续 5 帧一致，失败则保留旧参考。
- 前刷反馈支持脉冲/RPM监测和自动解卡，最多尝试 5 次，仍失败进入 `JAM_HOLD`。
- 黑区由 OpenMV 视觉协议提供黑色占比，STM32 负责阈值确认、运动和卸货时序；蓝墙不是个人赛主流程的避障触发源。

## 个人赛黑区卸货

空扫描找不到物块后，程序进入原有的黑区返回序列：先对齐上电初始方向，IMU 闭环逆时针 90°，使用车尾超声倒车到约 180 mm，再逆时针 90°并低速直行寻找黑色区域。黑色占比达到 3%（连续 3 帧）开始跟随，达到 40%（连续 3 帧）确认到达；随后预退 80 mm、对齐初始方向、逆时针 30°，再倒车至车尾超声 ≤120 mm、编码器约 300 mm或 10 s 兜底，最后由舵机执行“内转 90°→外转 180°”并抖动车体卸货。

详细步骤和参数见工程目录中的 `COVERAGE_REAR_UNLOAD_GUIDE.md`、`GREEDY_IMU_GUIDE.md` 与 `PERSONAL_PERIODIC_IMU_GUIDE.md`。

## 接口与构建

| 模块 | 配置 |
| --- | --- |
| OpenMV | UART4，PC10/PC11，115200 8N1 |
| 蓝牙 | USART3，PB10/PB11，115200 8N1 |
| JY901S | USART1：PA10 ← TX、PA9 → RX，9600 |
| 车轮/前刷编码器、车尾超声、舵机 | 沿用本工程 `.ioc` 中的定时器、EXTI、PWM和GPIO配置 |

在 STM32CubeIDE 中导入工程目录的 `.project`，执行 `Clean Project` → `Build Project` 后通过 ST-Link 烧录。主机测试：

```sh
make -C STM32F103VET6_collection_car_v7.6.3_personal_strict_imu_frames/tests test
```

本分支不提交 `Debug/`、`Release/` 或预编译固件；烧录前应先检查 OpenMV 与 JY901S 的串口交叉接线和共地。
