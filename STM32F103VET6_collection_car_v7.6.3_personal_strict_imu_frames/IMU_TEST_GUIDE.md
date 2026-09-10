# JY901S 停车测试（蓝牙I）：FW=v6.2+REVERSE_IMU_D_RETURN

蓝牙I用于独立停车诊断IMU。蓝牙D已经改为直接执行RETURN_ORIGIN、Coverage和卸货。当前Q使用通过本页测试的新鲜YAW辅助扫描、倒车航向保持、Coverage和卸货调头；A/C仍保持原控制方式。导航参数和降级逻辑见 `GREEDY_IMU_GUIDE.md`。

## 接线和烧录

断电接线，JY901S VCC→3.3V，GND→GND，TX→PA10（USART1_RX），RX→PA9（USART1_TX）。必须共地、TX/RX 交叉连接。PA8 后舵机、UART4 PC10/PC11 摄像头、USART3 PB10/PB11 蓝牙均保留。

IMU 接收配置为 9600 波特率、8 数据位、无校验、1 停止位。若传感器曾改为其他波特率，需要将 Core/Inc/jy901.h 的 JY901_UART_BAUD 改成相同值后重新编译。本版没有自动波特率识别，也不向传感器写入设置。蓝牙仍为原来的 115200，不要改成 9600。

STM32CubeIDE 导入完整工程，Refresh → Clean → Build，确认 `Core/Src/jy901.c` 和 `Core/Src/imu_navigation.c` 参与编译，烧录本工程新生成的 `collection_car_combat_v6_1.elf`。无需先用CubeMX重新生成代码；重新生成后须核对手工串口接收和主循环接入是否保留。启动提示应含 `FW=v6.2+REVERSE_IMU_D_RETURN`。

如果你在现场另改过右轮编码器方向或驱动参数，请保留并核对那些修改。本地 LEFT_ENCODER_SIGN / RIGHT_ENCODER_SIGN 均为 +1，不能保证与现场修正相同，不要盲目覆盖标定参数。

## 推荐测试步骤

1. 架空车轮，保持机身平稳。上电后前刷仍可能运行，注意手指和衣物。
2. 蓝牙发送 ASCII 单字符 H，先停轮停刷；再发送 I。I 在前刷 HOLD 状态也可测试，但正在解卡时需先 H 停止。
3. 应收到 MODE=DEBUG_IMU，之后约每 500 ms 一组 IMU 信息。不需要摄像头，也无需发送 Q、E 或其他启动信号。
4. 等待 IMU OK 且 ZERO=1。保持静止记录一组输出，再手动水平旋转车身约 90°记录一组；轻微前后、左右倾斜检查 PITCH/ROLL。轴向取决于 IMU 安装方向。
5. V 立即查询；再次 I 使用后续收到的新鲜角度重新建立相对航向零点；S 退出并停车。

I 只保证测试模式内车轮停车，保留前刷原状态并关闭自动解卡监测；所以建议先 H。I 不会自动清除 HOLD。之后若需要恢复前刷，退出测试后发送 K 并等待恢复完成。当前左轮反馈问题未解决，不建议立即恢复自动行驶。

## 输出含义

以下数值仅为示例，不是实车测量：

```text
IMU OK BAUD=9600 RX=330 FRAME=30 ANGLE=10 AGE=20 CHECKERR=0 UARTERR=0 REARMERR=0
IMU DEG ROLL=0.10 PITCH=-0.20 YAW=35.00 REL_YAW=0.00 ZERO=1 GYRO_N=10 GZ_CDEGPS=0 GAGE=20
```

| 字段 | 含义 |
|---|---|
| OK | 最近 1500 ms 内收到有效角度帧 |
| NO_DATA | 尚未收到任何字节；先查供电、共地、TX→PA10 |
| NO_FRAME | 收到了字节，但尚无有效协议帧；查波特率、输出协议、信号干扰 |
| NO_ANGLE | 已有有效帧，但尚无 0x53 角度帧；查传感器角度输出配置 |
| STALE | 曾收到角度，但超过 1500 ms 没有更新；此时显示的角度是旧值 |
| RX / FRAME / ANGLE | 累计字节数 / 有效协议帧数 / 有效角度帧数，应持续增加 |
| AGE | 最近角度数据距现在的毫秒数；-1 表示从未收到 |
| CHECKERR | 累计校验失败数，观察是否持续增加 |
| UARTERR / REARMERR | 串口错误 / 接收重新挂接失败的累计次数 |
| ROLL / PITCH / YAW | 横滚 / 俯仰 / 航向，单位度 |
| REL_YAW / ZERO | 相对本次 I 零点的航向；ZERO=1 才已建立零点 |
| GYRO_N / GZ_CDEGPS / GAGE | 累计角速度帧 / Z 角速度（度每秒×100）/ 角速度数据年龄（ms） |

REL_YAW 范围约 -180°～180°，不是累计转圈角度。ZERO=0 时 REL_YAW=0 只是占位，不能认为定零成功。再次 I 不清除累计统计，也不是传感器校准；它只在软件内换一个参考角度。

静止时小幅波动不等于通信失败。电机磁场、供电和安装振动可能影响姿态表现，先停刷测试，再比较有电机干扰时的数据。当前不承诺航向精度或自动校准效果。

## 实现与验证

- jy901.c/h：固定 11 字节帧解析、校验、失步恢复、角度与角速度缓存。
- usart.c/h、stm32f1xx_it.c/h、IOC：USART1 引脚、9600 8N1 和中断接入。
- main.c：单字节中断接收、错误恢复、I/V/S 控制、停车与定时日志；中断内不打印蓝牙日志。
- tests/jy901_test.c 与 tests/control_regression.py：协议及实际控制函数回归测试。

协议依据：[WIT 官方标准协议](https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol)。采用 0x55 帧头、前 10 字节累加校验；0x53 有符号原始角度 /32768×180°，0x52 角速度 /32768×2000°/s。

已通过主机回归测试和 clang 语法检查；尚未执行 ARM 链接、烧录或硬件测试。请反馈静止和手动转约 90°时的两行 IMU 输出；若没有 OK，请保留错误状态和计数一起反馈。
