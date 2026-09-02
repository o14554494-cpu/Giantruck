/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "path_planner.h"
#include "target_map.h"
#include "vision_protocol.h"
#include "mission_extension.h"
#include "combat_strategy.h"
#include "servo.h"
#include "drive_pwm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  float integral;
  float previous_error;
} MotorPid_t;

typedef enum
{
  AUTO_IDLE = 0,
  AUTO_SCAN,
  AUTO_PLAN,
  AUTO_NAVIGATE,
  AUTO_FINAL_ALIGN,
  AUTO_COLLECT,
  AUTO_RECOVER,
  AUTO_COMBAT_PATROL,
  AUTO_COMPLETE
} AutoState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_COMMAND_TIMEOUT_MS  1000U
#define MOTOR_DEFAULT_SPEED       40U
#define MOTOR_TURN_SPEED          35U

/*
 * Set this to 1 only after both quadrature encoders are wired and tested.
 * With value 0, the original open-loop PWM control remains active while
 * encoder speed measurement and the PID implementation are still compiled.
 */
#define MOTOR_CLOSED_LOOP_ENABLE          0U
#define MOTOR_CONTROL_PERIOD_MS           10U
#define ENCODER_COUNTS_PER_WHEEL_REV      1560.0f
#define MOTOR_MAX_RPM                     120.0f
#define ENCODER_FILTER_ALPHA              0.30f
#define LEFT_ENCODER_SIGN                 1.0f
#define RIGHT_ENCODER_SIGN                1.0f

/* Initial values only. Tune them with the wheels lifted from the ground. */
#define MOTOR_PID_KP                      0.45f
#define MOTOR_PID_KI                      1.20f
#define MOTOR_PID_KD                      0.00f
#define MOTOR_PID_INTEGRAL_LIMIT          30.0f

/* Mechanical calibration values. Measure these again on the final vehicle. */
#define WHEEL_DIAMETER_MM                  80.0f
#define WHEEL_TRACK_MM                     184.0f
#define ROBOT_PI                           3.14159265358979323846f

/* Camera coordinate to robot-centre correction along the forward axis. */
#define CAMERA_FORWARD_OFFSET_MM           0.0f

/* Autonomous rolling-planner and final-approach parameters. */
#define AUTO_UPDATE_PERIOD_MS              20U
#define AUTO_SCAN_SPEED_PERCENT            18
#define AUTO_SCAN_TIMEOUT_MS              12000U
#define AUTO_SCAN_MIN_ROTATION_RAD         5.70f
#define AUTO_APPROACH_SPEED_PERCENT        35
#define AUTO_FINAL_SPEED_PERCENT           20
#define AUTO_TURN_SPEED_PERCENT            28
#define AUTO_APPROACH_RADIUS_MM            220.0f
#define AUTO_COLLECT_TRIGGER_MM            105U
#define AUTO_ALIGN_TOLERANCE_MM            25
#define AUTO_COLLECT_DURATION_MS           900U
#define AUTO_RECOVER_DURATION_MS           900U
#define AUTO_VISION_TIMEOUT_MS             1200U
#define AUTO_VISION_RETRY_MS               500U
/* Independent front brush: runs continuously after peripheral startup.
 * Raw TIM8 compare value, 0..1000; 500 = 50% duty. */
#define COLLECTOR_RUN_PWM                  500U
#define AUTO_REPLAN_PERIOD_MS              700U
#define AUTO_EMPTY_RESCAN_LIMIT             1U
#define AUTO_EMPTY_RESCAN_TIMEOUT_MS        3000U
#define AUTO_VISION_MATCH_MAX_ERROR_MM      300.0f

/* Must match the limits used by the paired OpenMV visual.py. */
#define VISION_MIN_FORWARD_MM               60U
#define VISION_MAX_FORWARD_MM               2500U
#define VISION_MIN_ACCEPTED_QUALITY         15U

/* Active buzzer used for target-found notification. The commonly used
 * STM32F103VET6 board routes its buzzer to PB8. If the actual carrier board
 * uses another pin, only these definitions need to be changed. */
#define BUZZER_PORT                         GPIOB
#define BUZZER_PIN                          GPIO_PIN_8
#define BUZZER_ACTIVE_STATE                 GPIO_PIN_SET
#define BUZZER_INACTIVE_STATE               GPIO_PIN_RESET
#define BUZZER_PULSE_MS                     120U
#define BUZZER_GAP_MS                        90U
#define BUZZER_MAX_PENDING_PULSES             8U

#define MOTOR_LEFT_IN1_PORT       GPIOC
#define MOTOR_LEFT_IN1_PIN        GPIO_PIN_0
#define MOTOR_LEFT_IN2_PORT       GPIOC
#define MOTOR_LEFT_IN2_PIN        GPIO_PIN_1
#define MOTOR_RIGHT_IN1_PORT      GPIOC
#define MOTOR_RIGHT_IN1_PIN       GPIO_PIN_2
#define MOTOR_RIGHT_IN2_PORT      GPIOC
#define MOTOR_RIGHT_IN2_PIN       GPIO_PIN_3

/* HC-SR04 倒车测距 */
#define HC_TRIG_PORT      GPIOB
#define HC_TRIG_PIN       GPIO_PIN_5
#define HC_ECHO_PORT      GPIOC
#define HC_ECHO_PIN       GPIO_PIN_6
#define HC_STOP_MM        150U   /* 距障碍物 15cm 停车,按需调 */
#define HC_SAMPLE_MS      100U   /* 每 100ms 测一次(大于模块 60ms 最小间隔) */
/* The extension uses this HC-SR04 as a front sensor. Keep the received
 * rear-only stop block compiled but disabled to avoid stopping while the car
 * is backing away from a front wall. */
#define LEGACY_REAR_HCSR04_STOP_ENABLE 0U

/* 电机启动补偿参数 */
#define MOTOR_BOOST_PERCENT        80U     /* 启动时的高占空比（建议70~90） */
#define MOTOR_BOOST_DURATION_MS    200U    /* 启动补偿持续时间（建议150~300ms） */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Separate bytes prevent UART4 vision traffic and USART3 Bluetooth traffic
 * from overwriting one another when both receive interrupts are active. */
static uint8_t vision_rx_byte;
static uint8_t bluetooth_rx_byte;
static uint8_t motor_speed_percent = MOTOR_DEFAULT_SPEED;
static uint8_t last_drive_command = 'S';
static uint8_t motor_running;
static uint32_t last_motion_command_tick;
static volatile uint8_t motor_control_due;
static int16_t left_target_percent;
static int16_t right_target_percent;
static int16_t left_pwm_percent;
static int16_t right_pwm_percent;
static float left_speed_rpm;
static float right_speed_rpm;
static MotorPid_t left_pid;
static MotorPid_t right_pid;
static RobotPose_t robot_pose;
static TargetMap_t target_map;
static MapTarget_t planner_candidates[TARGET_MAP_MAX_TARGETS];
static PlannerRoute_t planned_route;
static VisionFrame_t latest_vision_frame;
static uint32_t latest_vision_tick;
static uint32_t last_auto_update_tick;
static uint32_t auto_state_start_tick;
static uint32_t last_plan_tick;
static uint32_t current_scan_timeout_ms;
static uint32_t accepted_vision_frame_count;
static uint8_t vision_motion_hold;
static uint32_t vision_retry_tick;
static float scan_accumulated_angle;
static uint16_t current_target_id;
static uint8_t plan_dirty;
static uint8_t empty_plan_retry_count;
static float combat_patrol_x_mm;
static float combat_patrol_y_mm;
static AutoState_t auto_state = AUTO_IDLE;
static volatile uint8_t uart_packet_active;
static volatile uint8_t manual_command_pending;

static volatile uint32_t hc_distance_mm;
static uint32_t hc_last_sample_tick;
static uint8_t hc_close_count;
/*最近一次有效距离，0=无效*/
/* 连续近距离计数，防毛刺*/

/* 蓝牙 G 命令手动测试舵机时置 1，测试结束或进入自动卸货后清 0 */
static uint8_t servo_test_active;
static uint8_t buzzer_active;
static uint8_t buzzer_pending_pulses;
static uint32_t buzzer_state_tick;

/* 电机启动补偿状态（每个通道独立） */
static int16_t  ch1_last_target = 0;       /* CH1上一次的目标占空比 */
static int16_t  ch2_last_target = 0;       /* CH2上一次的目标占空比 */
static uint32_t ch1_boost_start_tick = 0;  /* CH1启动补偿开始时间 */
static uint32_t ch2_boost_start_tick = 0;  /* CH2启动补偿开始时间 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Motor_SetOne(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                         GPIO_TypeDef *in2_port, uint16_t in2_pin,
                         uint32_t channel, int16_t speed_percent);
static void Motor_Set(int16_t left_speed, int16_t right_speed);
static void Motor_SetTarget(int16_t left_speed, int16_t right_speed);
static void Motor_Stop(void);
static void Motor_ControlUpdate(void);
#if MOTOR_CLOSED_LOOP_ENABLE == 1U
static float Motor_PidStep(MotorPid_t *pid, float target_rpm,
                           float measured_rpm, float feedforward_percent);
#endif
static void Motor_ResetPid(MotorPid_t *pid);
static void Motor_ApplyDriveCommand(uint8_t command);
static void Motor_ProcessCommand(uint8_t command);
static void Motor_SendText(const char *text);
static void Motor_SendStatus(void);
static void Buzzer_Init(void);
static void Buzzer_NotifyTargetFound(void);
static void Buzzer_Update(void);
static void Robot_UpdateOdometry(int32_t left_delta, int32_t right_delta);
static void Vision_ProcessIncoming(void);
static void Vision_ApplyFrame(const VisionFrame_t *frame);
static void Vision_SendMode(char mode, uint8_t color);
static void Auto_StartScan(uint32_t timeout_ms);
static void Auto_Start(void);
static void Auto_Stop(uint8_t clear_map);
static void Auto_Update(void);
static void Auto_BuildPlan(void);
static void Auto_DriveTowardLocal(float forward_mm, float left_mm,
                                  int16_t base_speed);
static void Auto_SendMap(void);
static void Auto_SendPlan(void);
static const VisionTarget_t *Auto_FindCurrentVisionTarget(void);
static const char *Auto_StateName(AutoState_t state);
static void Mission_UpdateIntegration(void);
static void CollectorDirection_Init(void);
void motor3_forward(uint16_t speed);
static uint8_t Auto_CheckVision(uint32_t now);
static void Mission_ServoUpdate(MissionUnloadState_t unload_state,
                                uint32_t now_ms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Motor_SendText(const char *text)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)text, (uint16_t)strlen(text), 50U);
}

static void Buzzer_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = BUZZER_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_PORT, &gpio);
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_INACTIVE_STATE);

  buzzer_active = 0U;
  buzzer_pending_pulses = 0U;
  buzzer_state_tick = HAL_GetTick();
}

static void Buzzer_NotifyTargetFound(void)
{
  if (buzzer_active == 0U)
  {
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_ACTIVE_STATE);
    buzzer_active = 1U;
    buzzer_state_tick = HAL_GetTick();
  }
  else if (buzzer_pending_pulses < BUZZER_MAX_PENDING_PULSES)
  {
    buzzer_pending_pulses++;
  }
}

static void Buzzer_Update(void)
{
  uint32_t now = HAL_GetTick();

  if (buzzer_active != 0U)
  {
    if ((now - buzzer_state_tick) >= BUZZER_PULSE_MS)
    {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_INACTIVE_STATE);
      buzzer_active = 0U;
      buzzer_state_tick = now;
    }
  }
  else if ((buzzer_pending_pulses != 0U) &&
           ((now - buzzer_state_tick) >= BUZZER_GAP_MS))
  {
    buzzer_pending_pulses--;
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_ACTIVE_STATE);
    buzzer_active = 1U;
    buzzer_state_tick = now;
  }
}

static float Robot_AbsFloat(float value)
{
  return value < 0.0f ? -value : value;
}

static const char *Auto_StateName(AutoState_t state)
{
  switch (state)
  {
    case AUTO_IDLE:        return "IDLE";
    case AUTO_SCAN:        return "SCAN";
    case AUTO_PLAN:        return "PLAN";
    case AUTO_NAVIGATE:    return "NAVIGATE";
    case AUTO_FINAL_ALIGN: return "ALIGN";
    case AUTO_COLLECT:     return "COLLECT";
    case AUTO_RECOVER:     return "RECOVER";
    case AUTO_COMBAT_PATROL:return "PATROL";
    case AUTO_COMPLETE:    return "COMPLETE";
    default:               return "UNKNOWN";
  }
}

#if MOTOR_CLOSED_LOOP_ENABLE == 1U
static float Motor_ClampFloat(float value, float minimum, float maximum)
{
  if (value > maximum)
  {
    return maximum;
  }
  if (value < minimum)
  {
    return minimum;
  }
  return value;
}
#endif

static int16_t Motor_ClampPercent(int16_t value)
{
  if (value > 100)
  {
    return 100;
  }
  if (value < -100)
  {
    return -100;
  }
  return value;
}

static void Motor_SetOne(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                         GPIO_TypeDef *in2_port, uint16_t in2_pin,
                         uint32_t channel, int16_t speed_percent)
{
  uint32_t duty;
  uint32_t period;
  int16_t magnitude;
  int16_t target_magnitude;
  int16_t  *last_target;
  uint32_t *boost_start;

  /* 根据通道选择对应的状态变量 */
  if (channel == TIM_CHANNEL_1)
  {
    last_target = &ch1_last_target;
    boost_start = &ch1_boost_start_tick;
  }
  else
  {
    last_target = &ch2_last_target;
    boost_start = &ch2_boost_start_tick;
  }

  /* 速度限幅 */
  if (speed_percent > 100)
  {
    speed_percent = 100;
  }
  else if (speed_percent < -100)
  {
    speed_percent = -100;
  }

  /* 根据速度正负设置方向引脚 */
  if (speed_percent > 0)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    magnitude = speed_percent;
  }
  else if (speed_percent < 0)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    magnitude = -speed_percent;
  }
  else
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    magnitude = 0;
  }

  /* 保存目标占空比（用于下次判断是否从停止启动） */
  target_magnitude = magnitude;

  /* ========== 启动补偿逻辑 ========== */
  if (magnitude > 0)
  {
    /* 检测是否从停止状态启动（上一次目标为0，本次非0） */
    if (*last_target == 0)
    {
      *boost_start = HAL_GetTick();   /* 记录启动时间 */
    }

    /* 在boost持续时间内，使用高占空比 */
    if ((HAL_GetTick() - *boost_start) < MOTOR_BOOST_DURATION_MS)
    {
      magnitude = MOTOR_BOOST_PERCENT;
    }
  }
  /* ================================== */

  /* 记录本次目标占空比 */
  *last_target = target_magnitude;

  /* 计算PWM占空比并写入比较寄存器 */
  period = __HAL_TIM_GET_AUTORELOAD(&htim3);
  duty = ((uint32_t)magnitude * period) / 100U;
  __HAL_TIM_SET_COMPARE(&htim3, channel, duty);
}

static void Motor_Set(int16_t left_speed, int16_t right_speed)
{
  /* Apply the hardware dead-zone mapping once, at the final wheel output.
   * Targets remain logical commands; PWM telemetry reports actual duty. */
  left_speed = Motor_MapDrivePercent(left_speed);
  right_speed = Motor_MapDrivePercent(right_speed);

  Motor_SetOne(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN,
               MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN,
               TIM_CHANNEL_1, left_speed);

  Motor_SetOne(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN,
               MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN,
               TIM_CHANNEL_2, right_speed);

  left_pwm_percent = left_speed;
  right_pwm_percent = right_speed;
}

static void Motor_SetTarget(int16_t left_speed, int16_t right_speed)
{
  left_target_percent = Motor_ClampPercent(left_speed);
  right_target_percent = Motor_ClampPercent(right_speed);
  motor_running = (left_target_percent != 0) || (right_target_percent != 0);

#if MOTOR_CLOSED_LOOP_ENABLE == 0U
  Motor_Set(left_target_percent, right_target_percent);
#endif
}

static void Motor_ResetPid(MotorPid_t *pid)
{
  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
}

#if MOTOR_CLOSED_LOOP_ENABLE == 1U
static float Motor_PidStep(MotorPid_t *pid, float target_rpm,
                           float measured_rpm, float feedforward_percent)
{
  const float dt_seconds = ((float)MOTOR_CONTROL_PERIOD_MS) / 1000.0f;
  float error = target_rpm - measured_rpm;
  float derivative;
  float output;

  pid->integral += MOTOR_PID_KI * error * dt_seconds;
  pid->integral = Motor_ClampFloat(pid->integral,
                                   -MOTOR_PID_INTEGRAL_LIMIT,
                                   MOTOR_PID_INTEGRAL_LIMIT);

  derivative = (error - pid->previous_error) / dt_seconds;
  pid->previous_error = error;

  output = feedforward_percent + MOTOR_PID_KP * error +
           pid->integral + MOTOR_PID_KD * derivative;

  return Motor_ClampFloat(output, -100.0f, 100.0f);
}
#endif

static void Motor_ControlUpdate(void)
{
  int32_t left_delta;
  int32_t right_delta;
  float rpm_scale;
  float left_instant_rpm;
  float right_instant_rpm;

  left_delta = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  right_delta = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim5);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_SET_COUNTER(&htim5, 0U);

  rpm_scale = 60000.0f /
              (ENCODER_COUNTS_PER_WHEEL_REV * (float)MOTOR_CONTROL_PERIOD_MS);
  left_instant_rpm = (float)left_delta * rpm_scale * LEFT_ENCODER_SIGN;
  right_instant_rpm = (float)right_delta * rpm_scale * RIGHT_ENCODER_SIGN;

  left_speed_rpm += ENCODER_FILTER_ALPHA *
                    (left_instant_rpm - left_speed_rpm);
  right_speed_rpm += ENCODER_FILTER_ALPHA *
                     (right_instant_rpm - right_speed_rpm);

  Robot_UpdateOdometry(left_delta, right_delta);

#if MOTOR_CLOSED_LOOP_ENABLE == 1U
  if (left_target_percent == 0)
  {
    Motor_ResetPid(&left_pid);
    left_pwm_percent = 0;
  }
  else
  {
    float target_rpm = ((float)left_target_percent * MOTOR_MAX_RPM) / 100.0f;
    left_pwm_percent = (int16_t)Motor_PidStep(&left_pid, target_rpm,
                                              left_speed_rpm,
                                              (float)left_target_percent);
  }

  if (right_target_percent == 0)
  {
    Motor_ResetPid(&right_pid);
    right_pwm_percent = 0;
  }
  else
  {
    float target_rpm = ((float)right_target_percent * MOTOR_MAX_RPM) / 100.0f;
    right_pwm_percent = (int16_t)Motor_PidStep(&right_pid, target_rpm,
                                               right_speed_rpm,
                                               (float)right_target_percent);
  }

  Motor_Set(left_pwm_percent, right_pwm_percent);
#endif
}

static void Motor_Stop(void)
{
  left_target_percent = 0;
  right_target_percent = 0;
  motor_running = 0U;
  Motor_ResetPid(&left_pid);
  Motor_ResetPid(&right_pid);
  Motor_Set(0, 0);
  last_drive_command = 'S';
}

static void Motor_ApplyDriveCommand(uint8_t command)
{
  int16_t speed = (int16_t)motor_speed_percent;
  int16_t turn_speed = (int16_t)MOTOR_TURN_SPEED;

  switch (command)
  {
    case 'F':
      Motor_SetTarget(speed, speed);
      break;

    case 'B':
      Motor_SetTarget(-speed, -speed);
      break;

    case 'L':
      Motor_SetTarget(-turn_speed, turn_speed);
      break;

    case 'R':
      Motor_SetTarget(turn_speed, -turn_speed);
      break;

    case 'S':
    default:
      Motor_Stop();
      command = 'S';
      break;
  }

  last_drive_command = command;
  last_motion_command_tick = HAL_GetTick();
}

static void Motor_ProcessCommand(uint8_t command)
{
  if ((command >= 'a') && (command <= 'z'))
  {
    command = (uint8_t)(command - ('a' - 'A'));
  }

  if ((command >= '0') && (command <= '9'))
  {
    motor_speed_percent = (uint8_t)(command - '0') * 10U;

    if (motor_speed_percent == 0U)
    {
      Motor_Stop();
    }
    else if (last_drive_command != 'S')
    {
      Motor_ApplyDriveCommand(last_drive_command);
    }

    Motor_SendText("SPEED UPDATED\r\n");
    return;
  }

  switch (command)
  {
    case 'F':
    case 'B':
    case 'L':
    case 'R':
    case 'S':
      servo_test_active = 0U;
      Servo_Stop();
      if (auto_state != AUTO_IDLE)
      {
        Auto_Stop(0U);
      }
      CombatStrategy_Stop();
      Motor_ApplyDriveCommand(command);
      Motor_SendText("OK\r\n");
      break;

    case 'A':
      servo_test_active = 0U;
      Servo_Stop();
      CombatStrategy_Stop();
      Auto_Start();
      Motor_SendText("TECHNICAL AUTO STARTED\r\n");
      break;

    case 'C':
      servo_test_active = 0U;
      Servo_Stop();
      CombatStrategy_Start(HAL_GetTick());
      Auto_Start();
      Motor_SendText("COMBAT MODE STARTED: 5 MIN ROLLING STRATEGY\r\n");
      break;

    case 'X':
      servo_test_active = 0U;
      Servo_Stop();
      CombatStrategy_Stop();
      Auto_Stop(1U);
      Motor_SendText("AUTO STOPPED, MAP CLEARED\r\n");
      break;

    case 'M':
      Auto_SendMap();
      break;

    case 'P':
      Auto_SendPlan();
      break;

    case 'V':
      Motor_SendStatus();
      break;

    case 'G':
      if (auto_state != AUTO_IDLE)
      {
        Auto_Stop(0U);
      }
      CombatStrategy_Stop();
      if (servo_test_active != 0U)
      {
        servo_test_active = 0U;
        Servo_Stop();
        Motor_SendText("SERVO OFF\r\n");
      }
      else
      {
        servo_test_active = 1U;
        Servo_StartEject();
        Motor_SendText("SERVO ON (EJECT TEST)\r\n");
      }
      break;

    case '\r':
    case '\n':
      break;

    default:
      Motor_Stop();
      Motor_SendText("INVALID COMMAND - MOTOR STOPPED\r\n");
      break;
  }
}

static void Motor_SendStatus(void)
{
  char status[256];
  uint32_t now = HAL_GetTick();
  uint32_t vision_age_ms = accepted_vision_frame_count == 0U ? 0xFFFFFFFFUL :
                           now - latest_vision_tick;

  (void)snprintf(status, sizeof(status),
                 "RPM L=%ld R=%ld PWM L=%d R=%d PID=%s AUTO=%s "
                 "POSE=%ld,%ld,%ld MAP=%u VSEQ=%u VAGE=%lu "
                 "VFRAMES=%lu RXERR=%lu VHOLD=%u\r\n",
                 (long)left_speed_rpm, (long)right_speed_rpm,
                 (int)left_pwm_percent, (int)right_pwm_percent,
#if MOTOR_CLOSED_LOOP_ENABLE == 1U
                 "ON",
#else
                 "OFF",
#endif
                 Auto_StateName(auto_state),
                 (long)robot_pose.x_mm, (long)robot_pose.y_mm,
                 (long)(robot_pose.heading_rad * 1000.0f),
                 (unsigned int)TargetMap_CountAvailable(&target_map),
                 (unsigned int)latest_vision_frame.sequence,
                 (unsigned long)vision_age_ms,
                 (unsigned long)accepted_vision_frame_count,
                 (unsigned long)VisionProtocol_GetErrorCount(),
                 (unsigned int)vision_motion_hold);
  Motor_SendText(status);
  (void)snprintf(status, sizeof(status),
                 "UNLOAD=%s PAYLOAD=%u SERVO=%s FRONT=%lu mm BRUSH=CONT PWM=%u/1000\r\n",
                 MissionExtension_UnloadStateName(),
                 (unsigned int)MissionExtension_HasPayload(),
                 (Servo_IsRunning() != 0U) ? "RUN" : "OFF",
                 (unsigned long)hc_distance_mm,
                 (unsigned int)COLLECTOR_RUN_PWM);
  Motor_SendText(status);
  (void)snprintf(status, sizeof(status),
                 "COMBAT=%s ELAPSED=%lu PAYLOAD=%u OPP_DWELL=%lu\r\n",
                 CombatStrategy_PhaseName(now),
                 (unsigned long)CombatStrategy_ElapsedMs(now),
                 (unsigned int)CombatStrategy_GetPayloadCount(),
                 (unsigned long)CombatStrategy_OpponentDwellMs(now));
  Motor_SendText(status);
}

static void Robot_UpdateOdometry(int32_t left_delta, int32_t right_delta)
{
  const float millimetres_per_count =
      (ROBOT_PI * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_WHEEL_REV;
  float left_distance = (float)left_delta * LEFT_ENCODER_SIGN *
                        millimetres_per_count;
  float right_distance = (float)right_delta * RIGHT_ENCODER_SIGN *
                         millimetres_per_count;
  float centre_distance = (left_distance + right_distance) * 0.5f;
  float heading_change = (right_distance - left_distance) / WHEEL_TRACK_MM;
  float midpoint_heading = robot_pose.heading_rad + heading_change * 0.5f;

  robot_pose.x_mm += centre_distance * Planner_Cos(midpoint_heading);
  robot_pose.y_mm += centre_distance * Planner_Sin(midpoint_heading);
  robot_pose.heading_rad = Planner_NormalizeAngle(
      robot_pose.heading_rad + heading_change);

  if (auto_state == AUTO_SCAN)
  {
    scan_accumulated_angle += Robot_AbsFloat(heading_change);
  }
}

static void Vision_SendMode(char mode, uint8_t color)
{
  char packet[32];
  uint16_t packet_length = VisionProtocol_FormatCommand(
      packet, sizeof(packet), mode, color);

  if (packet_length != 0U)
  {
    HAL_UART_Transmit(&huart4, (uint8_t *)packet, packet_length, 50U);
  }
}

static void Vision_ApplyFrame(const VisionFrame_t *frame)
{
  float heading_cos;
  float heading_sin;
  uint8_t index;
  uint32_t now = HAL_GetTick();

  if (frame == NULL)
  {
    return;
  }

  latest_vision_frame = *frame;
  latest_vision_tick = now;
  accepted_vision_frame_count++;
  heading_cos = Planner_Cos(robot_pose.heading_rad);
  heading_sin = Planner_Sin(robot_pose.heading_rad);

  for (index = 0U; index < frame->target_count; index++)
  {
    const VisionTarget_t *observation = &frame->targets[index];
    uint8_t available_before;
    uint8_t available_after;
    float local_forward;
    float local_left;
    float global_x;
    float global_y;

    if ((observation->forward_mm < VISION_MIN_FORWARD_MM) ||
        (observation->forward_mm > VISION_MAX_FORWARD_MM) ||
        (observation->quality < VISION_MIN_ACCEPTED_QUALITY))
    {
      continue;
    }

    local_forward = (float)observation->forward_mm +
                    CAMERA_FORWARD_OFFSET_MM;
    local_left = -(float)observation->lateral_mm;
    global_x = robot_pose.x_mm +
               heading_cos * local_forward - heading_sin * local_left;
    global_y = robot_pose.y_mm +
               heading_sin * local_forward + heading_cos * local_left;

    /* Reject detections whose ground-plane projection lies outside the
     * 3000 x 2000 mm fenced field. */
    if (MissionExtension_TargetInsideArena(global_x, global_y) == 0U)
    {
      continue;
    }

    available_before = TargetMap_CountAvailable(&target_map);
    (void)TargetMap_Upsert(&target_map, observation->color,
                           global_x, global_y, observation->quality, now);
    available_after = TargetMap_CountAvailable(&target_map);
    if (available_after > available_before)
    {
      Buzzer_NotifyTargetFound();
      Motor_SendText("TARGET FOUND\r\n");
    }
  }
  plan_dirty = 1U;
}

static void Vision_ProcessIncoming(void)
{
  VisionFrame_t frame;

  VisionProtocol_Process();
  while (VisionProtocol_GetFrame(&frame) != 0U)
  {
    Vision_ApplyFrame(&frame);
  }
}

static void Auto_StartScan(uint32_t timeout_ms)
{
  scan_accumulated_angle = 0.0f;
  current_scan_timeout_ms = timeout_ms;
  auto_state = AUTO_SCAN;
  auto_state_start_tick = HAL_GetTick();
  Vision_SendMode('S', 0U);
  /* The next Auto_Update must pass the vision gate before wheel movement.
   * The front brush runs independently of this state machine. */
  Motor_Stop();
}

static void Auto_Start(void)
{
  Motor_Stop();
  memset(&robot_pose, 0, sizeof(robot_pose));
  memset(&planned_route, 0, sizeof(planned_route));
  memset(&latest_vision_frame, 0, sizeof(latest_vision_frame));
  TargetMap_Reset(&target_map);
  MissionExtension_Reset();
  MissionExtension_SetInitialPose(&robot_pose);

  latest_vision_tick = 0U;
  accepted_vision_frame_count = 0U;
  vision_motion_hold = 0U;
  vision_retry_tick = 0U;
  current_target_id = 0U;
  plan_dirty = 0U;
  empty_plan_retry_count = 0U;
  Auto_StartScan(CombatStrategy_IsActive() != 0U ? 3000U :
                                                   AUTO_SCAN_TIMEOUT_MS);
  last_auto_update_tick = auto_state_start_tick;
  last_plan_tick = auto_state_start_tick;
}

static void Auto_Stop(uint8_t clear_map)
{
  vision_motion_hold = 0U;
  auto_state = AUTO_IDLE;
  current_target_id = 0U;
  plan_dirty = 0U;
  memset(&planned_route, 0, sizeof(planned_route));
  Motor_Stop();
  Vision_SendMode('S', 0U);

  if (clear_map != 0U)
  {
    memset(&robot_pose, 0, sizeof(robot_pose));
    memset(&latest_vision_frame, 0, sizeof(latest_vision_frame));
    latest_vision_tick = 0U;
    accepted_vision_frame_count = 0U;
    empty_plan_retry_count = 0U;
    TargetMap_Reset(&target_map);
    MissionExtension_Reset();
    MissionExtension_SetInitialPose(&robot_pose);
  }
}

static void Auto_BuildPlan(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t candidate_count = TargetMap_GetCandidates(
      &target_map, planner_candidates, TARGET_MAP_MAX_TARGETS);

  if (CombatStrategy_IsActive() != 0U)
  {
    CombatSelection_t selection;
    memset(&planned_route, 0, sizeof(planned_route));

    if (CombatStrategy_MatchFinished(now) != 0U)
    {
      current_target_id = 0U;
      auto_state = AUTO_COMPLETE;
      Motor_Stop();
      Vision_SendMode('S', 0U);
      Motor_SendText("COMBAT TIME FINISHED\r\n");
      return;
    }

    if (CombatStrategy_ShouldReturn(now) != 0U)
    {
      current_target_id = 0U;
      auto_state = AUTO_COMPLETE;
      Motor_Stop();
      Vision_SendMode('S', 0U);
      Motor_SendText("COMBAT BATCH READY: RETURN TO OWN WAREHOUSE\r\n");
      return;
    }

    selection = CombatStrategy_SelectTarget(&robot_pose, planner_candidates,
                                             candidate_count, now);
    if (selection.valid != 0U)
    {
      planned_route.count = 1U;
      planned_route.target_ids[0] = selection.target_id;
      planned_route.estimated_cost_mm = selection.score > 0.0f ?
                                         selection.score : 0.0f;
      current_target_id = selection.target_id;
      last_plan_tick = now;
      plan_dirty = 0U;
      auto_state = AUTO_NAVIGATE;
      auto_state_start_tick = now;
      Vision_SendMode('S', 0U);
      Motor_SendText(selection.opponent_warehouse_target != 0U ?
                     "COMBAT TARGET: OPPORTUNISTIC STEAL\r\n" :
                     "COMBAT TARGET: ROLLING HARVEST\r\n");
      return;
    }

    CombatStrategy_GetPatrolWaypoint(&combat_patrol_x_mm,
                                     &combat_patrol_y_mm, now);
    current_target_id = 0U;
    auto_state = AUTO_COMBAT_PATROL;
    auto_state_start_tick = now;
    Motor_SendText(CombatStrategy_GetPayloadCount() != 0U ?
                   "COMBAT MAP EMPTY: SEARCH WITH PAYLOAD\r\n" :
                   "COMBAT MAP EMPTY: REGIONAL PATROL\r\n");
    return;
  }

  Planner_BuildRoute(&robot_pose, planner_candidates,
                     candidate_count, &planned_route);
  last_plan_tick = now;
  plan_dirty = 0U;

  if (planned_route.count == 0U)
  {
    if (empty_plan_retry_count < AUTO_EMPTY_RESCAN_LIMIT)
    {
      empty_plan_retry_count++;
      current_target_id = 0U;
      Motor_Stop();
      Motor_SendText("AUTO RESCAN: NO CONFIRMED TARGETS\r\n");
      Auto_StartScan(AUTO_EMPTY_RESCAN_TIMEOUT_MS);
      return;
    }

    current_target_id = 0U;
    auto_state = AUTO_COMPLETE;
    Motor_Stop();
    Vision_SendMode('S', 0U);
    if ((latest_vision_tick == 0U) ||
        ((now - latest_vision_tick) > AUTO_VISION_TIMEOUT_MS))
    {
      Motor_SendText("AUTO STOPPED: VISION LINK STALE\r\n");
    }
    else
    {
      Motor_SendText("AUTO COMPLETE: NO AVAILABLE TARGETS\r\n");
    }
    return;
  }

  empty_plan_retry_count = 0U;
  current_target_id = planned_route.target_ids[0];
  auto_state = AUTO_NAVIGATE;
  auto_state_start_tick = HAL_GetTick();
  Vision_SendMode('S', 0U);
}

static const VisionTarget_t *Auto_FindCurrentVisionTarget(void)
{
  MapTarget_t *map_target = TargetMap_FindById(&target_map, current_target_id);
  const VisionTarget_t *best_target = NULL;
  float heading_cos;
  float heading_sin;
  float dx;
  float dy;
  float expected_forward;
  float expected_left;
  float expected_lateral;
  float best_error = 3.4e38f;
  uint8_t index;

  if (map_target == NULL)
  {
    return NULL;
  }

  heading_cos = Planner_Cos(robot_pose.heading_rad);
  heading_sin = Planner_Sin(robot_pose.heading_rad);
  dx = map_target->x_mm - robot_pose.x_mm;
  dy = map_target->y_mm - robot_pose.y_mm;
  expected_forward = heading_cos * dx + heading_sin * dy;
  expected_forward -= CAMERA_FORWARD_OFFSET_MM;
  expected_left = -heading_sin * dx + heading_cos * dy;
  expected_lateral = -expected_left;

  for (index = 0U; index < latest_vision_frame.target_count; index++)
  {
    const VisionTarget_t *observation = &latest_vision_frame.targets[index];
    float error;

    if (observation->color != map_target->color)
    {
      continue;
    }
    if ((observation->forward_mm < VISION_MIN_FORWARD_MM) ||
        (observation->forward_mm > VISION_MAX_FORWARD_MM) ||
        (observation->quality < VISION_MIN_ACCEPTED_QUALITY))
    {
      continue;
    }

    error = Robot_AbsFloat((float)observation->lateral_mm - expected_lateral) +
            Robot_AbsFloat((float)observation->forward_mm - expected_forward);
    if (error < best_error)
    {
      best_error = error;
      best_target = observation;
    }
  }
  return (best_error <= AUTO_VISION_MATCH_MAX_ERROR_MM) ? best_target : NULL;
}

static void Auto_DriveTowardLocal(float forward_mm, float left_mm,
                                  int16_t base_speed)
{
  float denominator = Robot_AbsFloat(forward_mm) + Robot_AbsFloat(left_mm);
  int16_t turn;

  if (forward_mm < 0.0f)
  {
    turn = (left_mm >= 0.0f) ? AUTO_TURN_SPEED_PERCENT :
                               -AUTO_TURN_SPEED_PERCENT;
    Motor_SetTarget(-turn, turn);
    return;
  }

  if (denominator < 1.0f)
  {
    Motor_Stop();
    return;
  }

  turn = (int16_t)((left_mm / denominator) * 70.0f);
  if (turn > AUTO_TURN_SPEED_PERCENT)
  {
    turn = AUTO_TURN_SPEED_PERCENT;
  }
  else if (turn < -AUTO_TURN_SPEED_PERCENT)
  {
    turn = -AUTO_TURN_SPEED_PERCENT;
  }

  Motor_SetTarget((int16_t)(base_speed - turn),
                  (int16_t)(base_speed + turn));
}

/* Loss of complete, checksum-validated frames holds autonomous wheel motion.
 * Manual control and an already-started odometry-based unload are separate.
 * On recovery rescan instead of resuming a partly completed pickup. */
static uint8_t Auto_CheckVision(uint32_t now)
{
  uint8_t index;
  if ((auto_state == AUTO_IDLE) || (auto_state == AUTO_COMPLETE))
  {
    vision_motion_hold = 0U;
    return 1U;
  }
  if ((accepted_vision_frame_count == 0U) ||
      ((uint32_t)(now - latest_vision_tick) > AUTO_VISION_TIMEOUT_MS))
  {
    Motor_Stop();
    if (vision_motion_hold == 0U)
    {
      vision_motion_hold = 1U;
      current_target_id = 0U;
      memset(&planned_route, 0, sizeof(planned_route));
      /* Keep already-collected history and payload, discard stale candidates. */
      for (index = 0U; index < TARGET_MAP_MAX_TARGETS; index++)
      {
        if (target_map.targets[index].collected == 0U)
          target_map.targets[index].valid = 0U;
      }
      plan_dirty = 1U;
      Vision_SendMode('S', 0U);
      vision_retry_tick = now;
      Motor_SendText("VISION LOST: WHEELS STOPPED, BRUSH CONTINUES\r\n");
    }
    else if ((uint32_t)(now - vision_retry_tick) >= AUTO_VISION_RETRY_MS)
    {
      vision_retry_tick = now;
      Vision_SendMode('S', 0U);
    }
    return 0U;
  }
  if (vision_motion_hold != 0U)
  {
    vision_motion_hold = 0U;
    empty_plan_retry_count = 0U;
    Auto_StartScan(CombatStrategy_IsActive() != 0U ? 3000U :
                                                    AUTO_SCAN_TIMEOUT_MS);
    Motor_SendText("VISION RESTORED: RESCAN BEFORE COLLECTING\r\n");
  }
  return 1U;
}

static void Auto_Update(void)
{
  uint32_t now = HAL_GetTick();

  /* Run before the 20ms scheduling gate so a stale link cannot keep driving. */
  if (Auto_CheckVision(now) == 0U) return;

  if ((now - last_auto_update_tick) < AUTO_UPDATE_PERIOD_MS)
  {
    return;
  }
  last_auto_update_tick = now;

  if (CombatStrategy_IsActive() != 0U)
  {
    CombatStrategy_UpdateZone(&robot_pose, now);
    if (CombatStrategy_MatchFinished(now) != 0U)
    {
      if (auto_state != AUTO_COMPLETE)
      {
        Motor_Stop();
        current_target_id = 0U;
        auto_state = AUTO_COMPLETE;
        Vision_SendMode('S', 0U);
        Motor_SendText("COMBAT 5 MINUTES FINISHED\r\n");
      }
    }
    else if ((CombatStrategy_ShouldReturn(now) != 0U) &&
             (auto_state != AUTO_COLLECT) &&
             (auto_state != AUTO_FINAL_ALIGN) &&
             (auto_state != AUTO_COMPLETE))
    {
      Motor_Stop();
      current_target_id = 0U;
      auto_state = AUTO_PLAN;
    }
    else if ((CombatStrategy_OpponentDwellMs(now) >=
              COMBAT_MAX_OPPONENT_DWELL_MS) &&
             (auto_state != AUTO_COLLECT) &&
             (auto_state != AUTO_FINAL_ALIGN) &&
             (auto_state != AUTO_COMPLETE))
    {
      /* The 25 s software limit leaves five seconds of margin before the
       * rule's 30 s maximum. Planning will reject further enemy-zone targets. */
      Motor_Stop();
      current_target_id = 0U;
      auto_state = AUTO_PLAN;
      Motor_SendText("COMBAT OPPONENT ZONE LIMIT: EXIT NOW\r\n");
    }
  }

  switch (auto_state)
  {
    case AUTO_IDLE:
    case AUTO_COMPLETE:
      break;

    case AUTO_SCAN:
      Motor_SetTarget(-AUTO_SCAN_SPEED_PERCENT, AUTO_SCAN_SPEED_PERCENT);
      if ((scan_accumulated_angle >= AUTO_SCAN_MIN_ROTATION_RAD) ||
          ((now - auto_state_start_tick) >= current_scan_timeout_ms))
      {
        Motor_Stop();
        auto_state = AUTO_PLAN;
        auto_state_start_tick = now;
      }
      break;

    case AUTO_PLAN:
      Auto_BuildPlan();
      break;

    case AUTO_NAVIGATE:
    {
      MapTarget_t *target = TargetMap_FindById(&target_map, current_target_id);
      float heading_cos;
      float heading_sin;
      float dx;
      float dy;
      float local_forward;
      float local_left;
      float distance;

      if ((target == NULL) || (target->collected != 0U))
      {
        auto_state = AUTO_PLAN;
        break;
      }

      heading_cos = Planner_Cos(robot_pose.heading_rad);
      heading_sin = Planner_Sin(robot_pose.heading_rad);
      dx = target->x_mm - robot_pose.x_mm;
      dy = target->y_mm - robot_pose.y_mm;
      local_forward = heading_cos * dx + heading_sin * dy;
      local_left = -heading_sin * dx + heading_cos * dy;
      distance = Planner_Sqrt(dx * dx + dy * dy);

      if ((plan_dirty != 0U) &&
          ((now - last_plan_tick) >= AUTO_REPLAN_PERIOD_MS) &&
          (distance > (AUTO_APPROACH_RADIUS_MM + 150.0f)))
      {
        Auto_BuildPlan();
        break;
      }

      if (distance <= AUTO_APPROACH_RADIUS_MM)
      {
        Motor_Stop();
        auto_state = AUTO_FINAL_ALIGN;
        auto_state_start_tick = now;
        Vision_SendMode('T', target->color);
      }
      else
      {
        int16_t speed = distance < 500.0f ? 26 :
                        AUTO_APPROACH_SPEED_PERCENT;
        Auto_DriveTowardLocal(local_forward, local_left, speed);
      }
      break;
    }

    case AUTO_FINAL_ALIGN:
    {
      const VisionTarget_t *observation = Auto_FindCurrentVisionTarget();

      if ((now - latest_vision_tick) > AUTO_VISION_TIMEOUT_MS)
      {
        Motor_Stop();
        auto_state = AUTO_RECOVER;
        auto_state_start_tick = now;
        Vision_SendMode('S', 0U);
        break;
      }

      if (observation == NULL)
      {
        /* A fresh empty/mismatched frame is different from a lost link,
         * but it must not leave the previous drive command active. */
        Motor_Stop();
        if ((now - auto_state_start_tick) > AUTO_VISION_TIMEOUT_MS)
        {
          auto_state = AUTO_RECOVER;
          auto_state_start_tick = now;
          Vision_SendMode('S', 0U);
        }
        break;
      }

      if ((observation->forward_mm <= AUTO_COLLECT_TRIGGER_MM) &&
          (Robot_AbsFloat((float)observation->lateral_mm) <=
           (float)AUTO_ALIGN_TOLERANCE_MM))
      {
        auto_state = AUTO_COLLECT;
        auto_state_start_tick = now;
        Motor_SetTarget(AUTO_FINAL_SPEED_PERCENT, AUTO_FINAL_SPEED_PERCENT);
      }
      else
      {
        float local_left = -(float)observation->lateral_mm;
        int16_t speed = Robot_AbsFloat((float)observation->lateral_mm) >
                        (float)AUTO_ALIGN_TOLERANCE_MM ? 0 :
                        AUTO_FINAL_SPEED_PERCENT;
        Auto_DriveTowardLocal((float)observation->forward_mm,
                              local_left, speed);
      }
      break;
    }

    case AUTO_COLLECT:
      Motor_SetTarget(AUTO_FINAL_SPEED_PERCENT, AUTO_FINAL_SPEED_PERCENT);
      if ((now - auto_state_start_tick) >= AUTO_COLLECT_DURATION_MS)
      {
        Motor_Stop();
        TargetMap_MarkCollected(&target_map, current_target_id, now);
        MissionExtension_RecordCollected();
        CombatStrategy_RecordCollected(now);
        current_target_id = 0U;
        plan_dirty = 1U;
        auto_state = AUTO_PLAN;
        auto_state_start_tick = now;
        Vision_SendMode('S', 0U);
      }
      break;

    case AUTO_RECOVER:
      Motor_SetTarget(-AUTO_TURN_SPEED_PERCENT, AUTO_TURN_SPEED_PERCENT);
      if ((now - auto_state_start_tick) >= AUTO_RECOVER_DURATION_MS)
      {
        Motor_Stop();
        auto_state = AUTO_PLAN;
        auto_state_start_tick = now;
      }
      break;

    case AUTO_COMBAT_PATROL:
    {
      uint8_t candidate_count = TargetMap_GetCandidates(
          &target_map, planner_candidates, TARGET_MAP_MAX_TARGETS);
      CombatSelection_t selection = CombatStrategy_SelectTarget(
          &robot_pose, planner_candidates, candidate_count, now);
      float heading_cos = Planner_Cos(robot_pose.heading_rad);
      float heading_sin = Planner_Sin(robot_pose.heading_rad);
      float dx = combat_patrol_x_mm - robot_pose.x_mm;
      float dy = combat_patrol_y_mm - robot_pose.y_mm;
      float distance = Planner_Sqrt(dx * dx + dy * dy);
      float local_forward = heading_cos * dx + heading_sin * dy;
      float local_left = -heading_sin * dx + heading_cos * dy;

      if (selection.valid != 0U)
      {
        Motor_Stop();
        auto_state = AUTO_PLAN;
        break;
      }
      if (distance <= 180.0f)
      {
        Motor_Stop();
        CombatStrategy_AdvancePatrol();
        Auto_StartScan(2200U);
      }
      else
      {
        Auto_DriveTowardLocal(local_forward, local_left, 28);
      }
      break;
    }

    default:
      Auto_Stop(0U);
      break;
  }
}

static void Auto_SendMap(void)
{
  char line[128];
  uint8_t index;

  (void)snprintf(line, sizeof(line), "MAP COUNT=%u\r\n",
                 (unsigned int)TargetMap_CountAvailable(&target_map));
  Motor_SendText(line);

  for (index = 0U; index < TARGET_MAP_MAX_TARGETS; index++)
  {
    const MapTarget_t *target = &target_map.targets[index];
    if (target->valid == 0U)
    {
      continue;
    }

    (void)snprintf(line, sizeof(line),
                   "ID=%u C=%u X=%ld Y=%ld Q=%u SEEN=%u DONE=%u\r\n",
                   (unsigned int)target->id, (unsigned int)target->color,
                   (long)target->x_mm, (long)target->y_mm,
                   (unsigned int)target->quality,
                   (unsigned int)target->seen_count,
                   (unsigned int)target->collected);
    Motor_SendText(line);
  }
}

static void Auto_SendPlan(void)
{
  char line[128];
  uint8_t index;

  (void)snprintf(line, sizeof(line), "PLAN N=%u COST=%ld CURRENT=%u:",
                 (unsigned int)planned_route.count,
                 (long)planned_route.estimated_cost_mm,
                 (unsigned int)current_target_id);
  Motor_SendText(line);

  for (index = 0U; index < planned_route.count; index++)
  {
    (void)snprintf(line, sizeof(line), " %u",
                   (unsigned int)planned_route.target_ids[index]);
    Motor_SendText(line);
  }
  Motor_SendText("\r\n");
}

//----------motor_3 控制函数---------------//
void motor3_forward(uint16_t speed)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, speed);
}

/* 卸货(EJECT)阶段驱动 MG995 后舱门舵机持续运动帮助物块卸下。
 * 离开 EJECT 阶段自动回到中位；手动 G 测试模式下不被自动复位。 */
static void Mission_ServoUpdate(MissionUnloadState_t unload_state,
                                uint32_t now_ms)
{
  if (unload_state == MISSION_UNLOAD_EJECT)
  {
    servo_test_active = 0U; /* 自动卸货优先于手动测试 */
    Servo_StartEject();
    Servo_Update(now_ms);
  }
  else if (servo_test_active == 0U)
  {
    Servo_Stop();
  }
}

/* PB0/PB1 were used by the third motor functions but were not configured by
 * the received CubeMX project. Configure them here without changing the
 * original motor GPIO layout. */
static void CollectorDirection_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
}

static void Mission_UpdateIntegration(void)
{
  uint32_t now = HAL_GetTick();
  MissionUnloadState_t unload_state = MissionExtension_GetUnloadState();

  /* Do not let the reactive avoidance state overwrite the vision hold. */
  if (vision_motion_hold != 0U)
  {
    Motor_Stop();
    return;
  }

  /* 舵机只在 EJECT 阶段持续运动，其余状态保持中位 */
  Mission_ServoUpdate(unload_state, now);

  if ((auto_state == AUTO_COMPLETE) &&
      (MissionExtension_HasPayload() != 0U) &&
      ((unload_state == MISSION_UNLOAD_IDLE) ||
       (unload_state == MISSION_UNLOAD_DONE)))
  {
    MissionExtension_StartUnload(now);
    unload_state = MissionExtension_GetUnloadState();
    Mission_ServoUpdate(unload_state, now);
    Motor_SendText("UNLOAD STARTED: GO TO LOWER-LEFT TARGET ZONE\r\n");
  }

  if ((unload_state != MISSION_UNLOAD_IDLE) &&
      (unload_state != MISSION_UNLOAD_DONE))
  {
    /* No rear sensor is assigned in the received hardware. Pass 0 so the
     * odometry stop coordinate is used. A rear sensor can be wired later. */
    MissionDriveOutput_t output =
        MissionExtension_UpdateUnload(&robot_pose, 0U, now);
    if (unload_state == MISSION_UNLOAD_GO_STAGE)
    {
      int16_t safe_left;
      int16_t safe_right;
      uint8_t request_replan;
      if (MissionExtension_ApplyForwardSafety(&robot_pose, hc_distance_mm, now,
                                              output.left_percent,
                                              output.right_percent,
                                              &safe_left, &safe_right,
                                              &request_replan) != 0U)
      {
        output.left_percent = safe_left;
        output.right_percent = safe_right;
      }
      (void)request_replan; /* GO_STAGE recomputes its local vector every loop. */
    }
    Motor_SetTarget(output.left_percent, output.right_percent);
    /* Unloading owns the rear servo only; it must not change the front brush. */
    if (output.finished != 0U)
    {
      Motor_Stop();
      if (CombatStrategy_IsActive() != 0U)
      {
        CombatStrategy_RecordDeposit(now);
        if (CombatStrategy_MatchFinished(now) == 0U)
        {
          Motor_SendText("COMBAT DEPOSIT COMPLETE: NEXT CYCLE\r\n");
          Auto_StartScan(2200U);
        }
        else
        {
          Motor_SendText("COMBAT FINAL DEPOSIT COMPLETE\r\n");
        }
      }
      else
      {
        Motor_SendText("UNLOAD COMPLETE\r\n");
      }
    }
    return;
  }

  /* The single HC-SR04 must face forward for this guard. Disable it during
   * final alignment/collection because a target block is intentionally close. */
  if ((auto_state == AUTO_NAVIGATE) ||
      (auto_state == AUTO_COMBAT_PATROL) ||
      ((auto_state == AUTO_IDLE) && (last_drive_command == 'F')))
  {
    int16_t safe_left;
    int16_t safe_right;
    uint8_t request_replan;
    if (MissionExtension_ApplyForwardSafety(&robot_pose, hc_distance_mm, now,
                                            left_target_percent,
                                            right_target_percent,
                                            &safe_left, &safe_right,
                                            &request_replan) != 0U)
    {
      Motor_SetTarget(safe_left, safe_right);
    }
    if (request_replan != 0U)
    {
      plan_dirty = 1U;
      last_plan_tick = 0U;
      Motor_SendText("OBSTACLE AVOIDED - ROUTE REPLAN REQUESTED\r\n");
    }
  }
}

/* 初始化引脚 + 打开 DWT 微秒计数器(72MHz 时钟) */
static void HCSR04_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  gpio.Pin = HC_TRIG_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(HC_TRIG_PORT, &gpio);
  HAL_GPIO_WritePin(HC_TRIG_PORT, HC_TRIG_PIN, GPIO_PIN_RESET);

  gpio.Pin = HC_ECHO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HC_ECHO_PORT, &gpio);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t HCSR04_Micros(void)
{
  return DWT->CYCCNT / 72U;   /* 72MHz → 微秒 */
}

/* 测一次距离,返回毫米;超时/无回波返回 0 */
static uint32_t HCSR04_Measure(void)
{
  uint32_t start;
  uint32_t t0;
  uint32_t timeout;

  /* 发 10us 触发脉冲 */
  HAL_GPIO_WritePin(HC_TRIG_PORT, HC_TRIG_PIN, GPIO_PIN_SET);
  t0 = HCSR04_Micros();
  while ((HCSR04_Micros() - t0) < 10U) { }
  HAL_GPIO_WritePin(HC_TRIG_PORT, HC_TRIG_PIN, GPIO_PIN_RESET);

  /* 等 ECHO 变高(超时 20ms) */
  timeout = HCSR04_Micros() + 20000U;
  while (HAL_GPIO_ReadPin(HC_ECHO_PORT, HC_ECHO_PIN) == GPIO_PIN_RESET)
  {
    if ((int32_t)(HCSR04_Micros() - timeout) > 0) return 0U;
  }
  start = HCSR04_Micros();

  /* 等 ECHO 变低(超时 30ms) */
  timeout = HCSR04_Micros() + 30000U;
  while (HAL_GPIO_ReadPin(HC_ECHO_PORT, HC_ECHO_PIN) != GPIO_PIN_RESET)
  {
    if ((int32_t)(HCSR04_Micros() - timeout) > 0) return 0U;
  }

  /* 距离(mm) = 回波时间(us) × 343m/s ÷ 2 */
  return ((HCSR04_Micros() - start) * 343U) / 2000U;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_SET_COUNTER(&htim5, 0U);

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  VisionProtocol_Init();
  TargetMap_Reset(&target_map);
  memset(&robot_pose, 0, sizeof(robot_pose));
  memset(&planned_route, 0, sizeof(planned_route));
  memset(&latest_vision_frame, 0, sizeof(latest_vision_frame));
  MissionExtension_Reset();
  MissionExtension_SetInitialPose(&robot_pose);
  CombatStrategy_Reset();

  if (HAL_UART_Receive_IT(&huart4, &vision_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UART_Receive_IT(&huart3, &bluetooth_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  Motor_Stop();
  Buzzer_Init();
  Vision_SendMode('S', 0U);
  Motor_SendText("READY: A=TECH, C=COMBAT, F/B/L/R/S, X=RESET, M/P/V, G=SERVO\r\n");

  //前刷独立常转：初始化完成后正转，不受视觉/驾驶/卸货状态控制//
  CollectorDirection_Init();
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  motor3_forward(COLLECTOR_RUN_PWM);

  //后舱门 MG995 舵机初始化：TIM1_CH1(PA8)，50Hz，回到中位//
  Servo_Init();

  //超声初始化//
  HCSR04_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (manual_command_pending != 0U)
    {
      uint8_t command = manual_command_pending;
      manual_command_pending = 0U;
      Motor_ProcessCommand(command);
    }

    Buzzer_Update();

    Vision_ProcessIncoming();

    if (motor_control_due != 0U)
    {
      motor_control_due = 0U;
      Motor_ControlUpdate();
    }

    Auto_Update();
    Mission_UpdateIntegration();

    if ((CombatStrategy_IsActive() != 0U) &&
        ((auto_state == AUTO_NAVIGATE) ||
         (auto_state == AUTO_COMBAT_PATROL)))
    {
      uint8_t forward_commanded =
          (left_target_percent > 10) && (right_target_percent > 10);
      if (CombatStrategy_UpdateStuck(&robot_pose, forward_commanded,
                                     HAL_GetTick()) != 0U)
      {
        Motor_Stop();
        auto_state = AUTO_RECOVER;
        auto_state_start_tick = HAL_GetTick();
        Motor_SendText("COMBAT STUCK 4S: RECOVER BEFORE FORCED RESTART\r\n");
      }
    }

    if ((auto_state == AUTO_IDLE) && motor_running &&
        ((HAL_GetTick() - last_motion_command_tick) > MOTOR_COMMAND_TIMEOUT_MS))
    {
      Motor_Stop();
      Motor_SendText("COMMAND TIMEOUT - MOTOR STOPPED\r\n");
    }
    /* HC-SR04 每 100ms 测一次;正在倒车且连续两次小于阈值 → 停车 */
        if ((HAL_GetTick() - hc_last_sample_tick) >= HC_SAMPLE_MS)
        {
          hc_last_sample_tick = HAL_GetTick();
          hc_distance_mm = HCSR04_Measure();

          /* 调试用:把距离发到串口,验证完可注释掉 */
          {
            char hc_line[48];
            (void)snprintf(hc_line, sizeof(hc_line), "DIST=%lu mm\r\n",
                           (unsigned long)hc_distance_mm);
            Motor_SendText(hc_line);
          }

          if (hc_distance_mm != 0U)
          {
            if (hc_distance_mm <= HC_STOP_MM)
            {
              hc_close_count++;
            }
            else
            {
              hc_close_count = 0U;
            }
          }

          if ((LEGACY_REAR_HCSR04_STOP_ENABLE != 0U) &&
              (last_drive_command == 'B') && (hc_close_count >= 2U))
          {
            Motor_Stop();
            hc_close_count = 0U;
            Motor_SendText("REVERSE STOP - UNLOAD POSITION\r\n");
          }
        }
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    motor_control_due = 1U;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    uint8_t byte = vision_rx_byte;

    if (byte == '$')
    {
      uart_packet_active = 1U;
      VisionProtocol_RxByteFromISR(byte);
    }
    else if (uart_packet_active != 0U)
    {
      VisionProtocol_RxByteFromISR(byte);
      if (byte == '\n')
      {
        uart_packet_active = 0U;
      }
    }
    (void)HAL_UART_Receive_IT(&huart4, &vision_rx_byte, 1U);
  }
  /* ===== 下面是新增的 USART3（蓝牙）处理 ===== */
    else if (huart->Instance == USART3)
    {
      uint8_t byte = bluetooth_rx_byte;

      // 蓝牙只接收手动命令，不处理视觉协议包（$开头的）
      if ((byte != '\r') && (byte != '\n') && (byte != '$'))
      {
        manual_command_pending = byte;
      }

      // 重新启动接收中断
      (void)HAL_UART_Receive_IT(&huart3, &bluetooth_rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    uart_packet_active = 0U;
    (void)HAL_UART_Receive_IT(&huart4, &vision_rx_byte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    (void)HAL_UART_Receive_IT(&huart3, &bluetooth_rx_byte, 1U);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
