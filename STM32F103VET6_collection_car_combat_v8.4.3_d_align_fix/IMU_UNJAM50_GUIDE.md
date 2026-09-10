# 本次更新与测试

固件标记：`FW=v6.1+IMU_UNJAM50`。使用用户提供的 alignment_visual_stable 包，不覆盖其他工作目录或原压缩包。

## IMU

上传包已经包含上一版 IMU 代码，本次核对保留。JY901S TX→PA10、RX→PA9、VCC→3.3V、GND→GND；STM32 USART1 为 9600 8N1。蓝牙仍是原接口。

先 H 停轮停刷，再 I 进入停车测试；V 查询，S 退出。等待 IMU OK、ZERO=1，再手动转动车身观察 REL_YAW。I 不校准传感器。D 已改为RETURN/Coverage/卸货全链路测试。完整字段见 IMU_TEST_GUIDE.md。

## 解卡后的前进 5 cm

正常流程：停轮停刷 150 ms → 前刷反转并限距倒退 → 停轮停刷 150 ms → 前刷正转、车辆前进 50 mm → 停轮并恢复原任务。

- J 手动解卡和 Q 中自动解卡都执行这个流程，无需额外前进命令。
- 前进时新增 UNJAM=ADVANCING；V 可查看 ADVANCE_L、ADVANCE_R、GOAL=50，单位 mm。完成打印 UNJAM ADVANCE DONE。
- 两轮分别按正向编码器轮程累计，各自达到 50 mm 后停止，两轮均达到才算完成。不是延时假定 5 cm；受打滑、惯性和标定影响，实际地面距离不能保证精确 5 cm。
- 前進逻辑速度 15，经原 PWM 最低 60% 映射约 66%，可能叠加原起步补偿，不能把它理解为 15% 实际 PWM。
- 1500 ms 内走不够、任一轮累计反向超过 5 mm、两轮轮程差超过 30 mm 时停轮停刷进入 HOLD；不会宣称完成，也不会自动重新前进。
- 现有前向超声在 SETTLING/ADVANCING 阶段继续采样，300 ms 内有效距离小于等于 HC_STOP_MM 时中止。单探头存在盲区，无回波/旧数据不阻止动作，不是完整防碰撞或后向保护。
- S/X/0/H/O 可中断。故障后先检查原因，K 只恢复前刷，绝不执行 5 cm 前进；重新 J 才重新尝试解卡。前刷故障保护保持原逻辑。
- 自动任务恢复保留原计数与里程计，清理旧目标并等待新视觉；手动 J 完成后待机，不重复执行此前 F。

参数位于 Core/Src/main.c：COLLECTOR_ADVANCE_DISTANCE_MM=50、COLLECTOR_ADVANCE_TIMEOUT_MS=1500、COLLECTOR_ADVANCE_PERCENT=15、COLLECTOR_ADVANCE_SKEW_MM=30。此前左轮 RPM=0 的故障必须解决，否则本阶段会保护停车；不建议放宽超时或伪造计数绕过。

## 取消 reacquire

取消额外的 1.5 s 等待与 ACTION=REACQUIRE。恢复原逻辑：目标不可见/空帧先停轮，ACTION=LOST；距最后看到目标 600 ms 后清除锁定并进入 60°分段扫描。期间收到有效目标可继续跟随。摄像头真正失联仍按原保护停车，不能把失联当作普通目标丢失。

双轮差速对准、视觉稳定性修复和进料补走均保留。visual(2).py 与上传包一致，本次无需为这些控制修改重刷 OpenMV。

## 烧录和试车

1. 在 STM32CubeIDE 导入完整工程，Refresh、Clean、Build 并烧录新 ELF；启动标记必须是 IMU_UNJAM50。
2. 先架空轮子，核对 F 时两轮实际正转且编码器均为正向有效反馈。异常先停止排查。
3. 空旷环境发送 J，观察反转/倒退后出现 ADVANCING，完成后待机；随时 H 急停。随后再测试 Q 的自动解卡和任务恢复。
4. 若出现 ADVANCE FAILED，发送 V，记录左右轮程和错误信息反馈；不要连续发 K/Q 跳过原因。

主机回归和 HAL C 语法检查不能替代 ARM 编译、传感器/电机实测。未连接实车验证。本次没有上传 GitHub。
