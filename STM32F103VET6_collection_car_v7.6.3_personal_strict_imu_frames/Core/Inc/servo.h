/**
  ******************************************************************************
  * @file    servo.h
  * @brief   MG995 后舱门舵机驱动（TIM1_CH1 / PA8, 50Hz PWM）
  *
  * 卸货时让舵机持续运动，把车厢里的物块从车尾推出去：
  *   - 普通 180° 舵机：按固定路径往复扫动（推荐默认）。
  *     受上底板限制不能 360° 旋转，路径为：
  *     初始垂直(90°) -> 按反向后的开门方向转到0° -> 回扫到180°清理剩余物块
 *     -> 保持在180°，底盘完成抖动后恢复垂直(90°)。
  *   - 改机 360° 连续旋转舵机：输出一个偏离中位的脉冲，持续旋转
  *
  * 两种模式只需改 SERVO_MODE 一个宏，其余代码不变。
  ******************************************************************************
  */
#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 舵机工作模式 -----------------------------------------------------------*/
#define SERVO_MODE_SWEEP        0U  /* 普通 180° 舵机：往复摆动扫动物块     */
#define SERVO_MODE_CONTINUOUS   1U  /* 360° 改机：持续单向旋转拍动物块     */

/* 你的 MG995 是哪种就设哪种：
 *   出厂原装(只能转 0~180°)  -> SERVO_MODE_SWEEP
 *   已改成 360° 连续旋转的   -> SERVO_MODE_CONTINUOUS
 * 改宏后重新编译即可，不需要改其它代码。 */
#define SERVO_MODE SERVO_MODE_SWEEP

/* 脉冲范围(us)。MG995 标准 0.5ms~2.5ms 对应 0°~180°，1.5ms 是中位。 */
#define SERVO_MIN_PULSE_US      500U
#define SERVO_MAX_PULSE_US      2500U
#define SERVO_NEUTRAL_PULSE_US  1500U

/* 普通舵机舱门扫摆路径参数 ------------------------------------------------
 * 路径:初始垂直(90°) -> 反向转90°(0°) -> 回扫180°(180°)清理剩余物块
 *      -> 恢复垂直(90°)，循环往复。
 * 若装好后实际转向相反，把 CCW_DEG 和 CW_DEG 两个值对调即可。 */
#define SERVO_HATCH_INITIAL_DEG  90U    /* 初始位置:垂直 */
#define SERVO_HATCH_CCW_DEG      0U     /* 已按实车反馈反转第一次开门方向 */
#define SERVO_HATCH_CW_DEG       180U   /* 反向后的180°清理回扫终点 */
#define SERVO_HATCH_MOVE90_MS    500U   /* 每段 90° 行程用时 */
#define SERVO_HATCH_SWEEP180_MS  1000U  /* 180° 清理行程用时 */
#define SERVO_HATCH_SEQUENCE_MS                                          \
  (SERVO_HATCH_MOVE90_MS + SERVO_HATCH_SWEEP180_MS)

/* 连续旋转舵机：脉冲偏离中位越多转速越快；小于 1500 则反转。
 * 1700 约为全速正转，1300 约为全速反转，装车前先用 G 命令实测方向。 */
#define SERVO_CR_SPEED_PULSE_US 1700U

void Servo_Init(void);

/* 进入卸货(EJECT)阶段时调用：启动一次卸货动作。重复调用不会重置相位。 */
void Servo_StartEject(void);

/* 离开卸货阶段或需要急停时调用：回到中位(1500us, 舵机保持不动)。 */
void Servo_Stop(void);

/* 主循环周期调用，驱动扫摆角度变化；连续旋转模式下为空操作。 */
void Servo_Update(uint32_t now_ms);

/* 当前舵机是否在持续运动 */
uint8_t Servo_IsRunning(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
