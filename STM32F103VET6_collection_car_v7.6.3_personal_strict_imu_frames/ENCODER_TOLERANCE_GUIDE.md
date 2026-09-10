# 前刷编码器采样与容错

固件标记 `FW=v6.1+ENC_TOLERANT`。基于 IMU_UNJAM50，视觉脚本未改，IMU 的 D/V/S 测试、解卡倒退后前进 50 mm、Greedy 回退 REACQUIRE 等功能保留。本次不修改接线、前刷实际转向或 SIGN=-1。

## 修正

- PB12/PB15 通过一次 GPIOB IDR 快照获取，减少分两次读取 A/B 的跨边沿风险。两路必须继续在同一 GPIO 端口，初始化会检查。
- 00→11 / 01→10 等非法跳变仍累计 ENCERR，且不计入有效转数；不猜测方向补脉冲。
- 每 100 ms 更新错误统计。孤立 1～2 次错误不会仅因这一错误立即停车，而是输出限频 WARN。
- 约 1 s 滚动窗口累计 >=8 次，或连续 3 个采样周期均有错误，触发 ENCODER_ERRORS 并停轮停刷到 HOLD。干净样本清除连续计数；超过 200 ms 的长采样间隔不沿用连续计数。主循环延迟时错误按本次采样时刻归入窗口，不能当作精确边沿时间统计。
- 错误窗口统计在未启用 AUTO 时也运行，但不会仅因窗口超限主动停车；Q 开启监测并重新建立窗口。原累计 ENCERR 仍保留，N 只重置累计显示基准。
- 启动宽限期不豁免密集错误；无脉冲、持续反向、堵转、解卡超时与可选电流保护保留。错误样本仍不算可靠的正向反馈，不用噪声伪造转动。
- 已标定 CPR 的反转两圈、手动一圈标定仍沿用严格无非法跳变要求。当前 CPR=0 的定时反转不承诺精确两圈。

## 新反馈

```text
BRUSH ENCODER WARN: ERR_WIN=1 TOTAL=1; MONITORING
ENC_HEALTH=WARN ERR_DELTA=1 ERR_WIN=1 ERR_STREAK=1 WINDOW_MS=1000 LIMIT=8 STREAK_LIMIT=3
```

这是格式示例。WARN 是编码器告警，不是自动解卡动作；其他保护仍可能同时停车。

- ENCERR：上电或 N 清零基准以来累计非法跳变。
- ERR_DELTA：最新采样区间新增错误。
- ERR_WIN：约最近 1 秒窗口错误总数。
- ERR_STREAK：连续有错误的采样周期数，上限为配置触发值。
- ENC_HEALTH：OK 无窗口错误；WARN 未超阈值；FAULT 窗口/连续阈值达到。只说明编码器统计，不代表整车无其他故障。
- LAST_EVENT：保留的上一次保护事件。停止后窗口过期可显示 ENC_HEALTH=OK、LAST_EVENT=ENCODER_ERRORS，HOLD 仍保持，需要排查后人为恢复。
- 触发停车时另打印 BRUSH ENCODER FAULT: DELTA=... WINDOW=... STREAK=... TOTAL=...，即使稍后 V 时窗口过期，也保留事发统计在日志里。

## 烧录和验证

1. 完整工程导入 CubeIDE，Refresh、Clean、Build 后烧录；确认 READY 中版本为 ENC_TOLERANT。无需改 OpenMV 或再次交换 A/B。
2. 先 H→N→K，核对前刷实际向内收集，V 中 DELTA 正向、ENCERR 不密集增长；有人看护、叶片附近不要放手或异物，H 可停止。
3. 在安全空旷场地发送一次 Q。观察是否只有偶发 WARN 而能继续任务，测试完 H。
4. 若出现 FAULT/HOLD，先发 V 保存整段日志，再排查，别先 N 清零或反复 Q。K 只恢复前刷；重新 Q 是新任务，会重置收集任务信息。

配置位于 Core/Inc/brush_feedback_config.h：BRUSH_ERROR_WINDOW_MS=1000、BRUSH_ERROR_WINDOW_LIMIT=8、BRUSH_ERROR_CONSECUTIVE_LIMIT=3、BRUSH_ERROR_WARN_MS=1000。这些是初始测试值，不建议为继续行驶无限放宽。若错误频繁，应检查供电、共地、接线、信号质量与中断响应。

主机回归及 C 语法验证通过不等于真实信号采样时序已验证；无法保证消除电气干扰。尚未 ARM 链接、烧录或实车测试，未上传 GitHub。
