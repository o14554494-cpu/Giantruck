# collection_car v7.6.3 个人赛周期IMU校准版

本包以用户提供的 `v7.6.2_competition_greedy_dual_imu_reference_unload30` 为唯一基础。烧录后上电自动进入个人赛Greedy：先静止采集10帧IMU初始零点，再开始6×60°扫描。新增每20秒到期、在下一次安全停车扫描边界执行的短期IMU校准。

## 比赛时操作

1. 按规定起点和初始朝向摆放小车；
2. 给 STM32、OpenMV、驱动器和传感器上电；
3. 保持小车静止，等待 IMU 零点锁定；
4. 车辆自动执行扫描、Greedy收集、黑区返回和舱门卸货。

蓝牙仅用于安全/观察：`V` 查询状态，`S` 紧急停车，`X`/`0` 停止并清理任务，`H` 停止并保持刷子故障状态。其他命令（包括 `Q`、`A`、`C`、`D`、`I`、`U`、`G`、`J`、`K`、数字调速等）均被忽略，不会切换模式或控制车辆。

## 重要反馈

```text
COMPETITION GREEDY AUTO: STARTED ON POWER-UP
IMU START ZERO LOCKED 10/10: GREEDY OBJECT SCAN START
```

如果 3 秒内无法完成零点采集，会安全停车并报告 `IMU START ZERO TIMEOUT`；检查 PA9/PA10、JY901S供电和安装后重新上电。

黑色卸货转向保持原 `unload30` 逻辑：“80 mm预退 → 双轮IMU对齐上电初始方向 → 双轮逆时针30° → 后退入黑区 → 舵机开舱门/抖动卸货”；转向后编码器倒车目标为300 mm，后方超声≤120 mm和10 s兜底仍保留。

空扫描找不到物块时，原流程保持不变：“对齐上电初始方向 → IMU闭环逆时针90° → 后方超声倒车至180 mm → 再逆时针90° → 沿当前方向慢速前进寻找黑区”。

周期校准的详细原理、触发条件和蓝牙反馈见 `PERSONAL_PERIODIC_IMU_GUIDE.md`。本包未包含ARM链接产物，需在STM32CubeIDE中Clean/Build后烧录。
