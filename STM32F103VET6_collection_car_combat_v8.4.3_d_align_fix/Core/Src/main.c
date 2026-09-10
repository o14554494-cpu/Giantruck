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
#include "brush_feedback.h"
#include "greedy_collection.h"
#include "greedy_local.h"
#include "jy901.h"
#include "imu_navigation.h"

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
  AUTO_COMPLETE,
  AUTO_DEBUG,
  AUTO_UNJAM,
  AUTO_UNLOAD_WAIT,
  AUTO_UNLOAD_ALIGN_ZERO,
  AUTO_UNLOAD_REVERSE_WALL,
  AUTO_UNLOAD_TURN_CCW_90,
  AUTO_UNLOAD_SEARCH_FORWARD,
  AUTO_UNLOAD_APPROACH,
  AUTO_UNLOAD_BACKUP_BEFORE_ALIGN,
  AUTO_UNLOAD_ALIGN_INITIAL,
  AUTO_UNLOAD_TURN_CCW_30,
  AUTO_UNLOAD_REVERSE,
  AUTO_COLLISION_RECOVER,
  AUTO_POST_UNLOAD_PACE_FORWARD,
  AUTO_POST_UNLOAD_PACE_REVERSE
} AutoState_t;

typedef enum
{
  COLLISION_RECOVER_STOP = 0,
  COLLISION_RECOVER_BACKUP,
  COLLISION_RECOVER_TURN
} CollisionRecoveryPhase_t;

typedef enum
{
  BLACK_BUMP_IDLE = 0,
  BLACK_BUMP_STOP,
  BLACK_BUMP_BACKUP,
  BLACK_BUMP_TURN_RIGHT,
  BLACK_BUMP_ESCAPE
} BlackBumpPhase_t;

typedef enum
{
  COLLECTOR_IDLE = 0,
  COLLECTOR_STOPPING,
  COLLECTOR_REVERSING,
  COLLECTOR_SETTLING,
  COLLECTOR_ADVANCING,
  COLLECTOR_HOLD
} CollectorRecoveryState_t;

typedef struct
{
  float integral;
  float previous_error;
  uint32_t previous_tick;
  uint8_t valid;
} BlackHeadingPid_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef COMPETITION_GREEDY_ONLY
#define COMPETITION_GREEDY_ONLY       0U
#endif
#ifndef COMPETITION_COMBAT_BLUETOOTH
#define COMPETITION_COMBAT_BLUETOOTH  1U
#endif
#define MOTOR_COMMAND_TIMEOUT_MS  1000U
#define MOTOR_DEFAULT_SPEED       40U
#define MOTOR_TURN_SPEED          35U

/*
 * Closed-loop wheel-speed PID. Both quadrature encoders must report the
 * correct sign and counts/revolution before the real vehicle is released.
 */
#ifndef MOTOR_CLOSED_LOOP_ENABLE
#define MOTOR_CLOSED_LOOP_ENABLE          1U
#endif
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
#define DEBUG_VISION_TIMEOUT_MS            600U
#define DEBUG_FORWARD_PERCENT             20
#define DEBUG_TURN_PERCENT                18
#define VISION_REPORT_PERIOD_MS            500U
#define VISION_WALL_CONFIRM_PACKETS          2U
#define VISION_WALL_CLEAR_PACKETS            2U
#define VISION_WALL_STALE_MS                700U
/* Combat collision detection uses signed wheel-encoder travel, never the
 * OpenMV blue-wall flag.  It is active only before the 240 s return phase. */
#define COLLISION_MONITOR_WINDOW_MS         3000U
#define COLLISION_MIN_WHEEL_TRAVEL_MM       10.0f
#define COLLISION_RECOVER_STOP_MS            150U
#define COLLISION_RECOVER_BACKUP_MM          50.0f
#define COLLISION_RECOVER_BACKUP_PERCENT      22
#define COLLISION_RECOVER_BACKUP_TIMEOUT_MS  4000U
#define COLLISION_RECOVER_TURN_RAD           (ROBOT_PI * 110.0f / 180.0f)
#define COLLISION_RECOVER_TURN_PERCENT        35
#define COLLISION_RECOVER_TURN_TIMEOUT_MS    6000U
#define BLUETOOTH_QUEUE_SIZE               32U
/* Independent front brush: normally runs continuously after startup;
 * an explicit J recovery or its interrupted HOLD is the only exception.
 * Raw TIM8 compare value, 0..1000; 500 = 50% duty. */
#define COLLECTOR_RUN_PWM                  500U
/* Legacy timed J fallback only when brush CPR has not been calibrated.
 * Calibrated manual/automatic recovery uses encoder counts instead. */
#define COLLECTOR_REVERSE_PWM              500U
#define COLLECTOR_REVERSE_TURNS            2U
#define COLLECTOR_REVERSE_RPM_ESTIMATE      60U
#define COLLECTOR_REVERSE_DURATION_MS      (60000U * COLLECTOR_REVERSE_TURNS / COLLECTOR_REVERSE_RPM_ESTIMATE)
#define COLLECTOR_DIRECTION_PAUSE_MS       150U
#define COLLECTOR_BACKUP_DISTANCE_MM       80.0f
#define COLLECTOR_BACKUP_TIMEOUT_MS        600U
#define COLLECTOR_BACKUP_PERCENT           15
#define COLLECTOR_ADVANCE_DISTANCE_MM      50.0f
#define COLLECTOR_ADVANCE_TIMEOUT_MS       1500U
#define COLLECTOR_ADVANCE_PERCENT          15
#define COLLECTOR_ADVANCE_SKEW_MM          30.0f
#if COLLECTOR_REVERSE_RPM_ESTIMATE == 0
#error "Calibrated brush RPM must be positive"
#elif COLLECTOR_REVERSE_DURATION_MS < 100 || COLLECTOR_REVERSE_DURATION_MS > 6000
#error "Brush reversal duration must be between 100 and 6000 ms"
#endif
#define AUTO_REPLAN_PERIOD_MS              700U
#define AUTO_EMPTY_RESCAN_LIMIT             1U
#define AUTO_EMPTY_RESCAN_TIMEOUT_MS        3000U
#define AUTO_VISION_MATCH_MAX_ERROR_MM      300.0f

/* Must match the limits used by the paired OpenMV visual.py. */
#define VISION_MIN_FORWARD_MM               60U
#define VISION_MAX_FORWARD_MM               2500U
#define VISION_MIN_ACCEPTED_QUALITY         15U

/* Target notifications now use Bluetooth; hold the old buzzer inactive. */
#define BUZZER_PORT                         GPIOB
#define BUZZER_PIN                          GPIO_PIN_8
#define BUZZER_INACTIVE_STATE               GPIO_PIN_RESET

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
/* The only HC-SR04 faces rearward. Never use it as a front obstacle sensor. */
#define REAR_HCSR04_MANUAL_STOP_ENABLE 1U

/* 电机启动补偿参数 */
#define MOTOR_BOOST_PERCENT        80U     /* 启动时的高占空比（建议70~90） */
#define MOTOR_BOOST_DURATION_MS    200U    /* 启动补偿持续时间（建议150~300ms） */
#define STRAIGHT_IMU_KP             12.0f
#define STRAIGHT_IMU_KI              0.05f
#define STRAIGHT_IMU_MAX_CORRECTION     4
#define STRAIGHT_IMU_DEADBAND_RAD    0.035f
#define STRAIGHT_IMU_I_LIMIT          0.30f

/* Q/D must establish the vehicle's real startup direction before any wheel
 * motion.  Ten distinct, quiet JY901 angle packets are required; the raw yaw
 * is then preserved as the zero reference for the whole task. */
#define GREEDY_IMU_ZERO_REQUIRED_FRAMES       10U
#define GREEDY_IMU_ZERO_STABILITY_RAW        364 /* About 2 degrees. */
#define GREEDY_IMU_ZERO_MAX_AGE_MS            200U
#define GREEDY_IMU_ZERO_TIMEOUT_MS           3000U

/* The startup reference above is immutable for the whole mission and remains
 * the only reference used by return/unload turns.  A second, local reference
 * is refreshed only while the chassis is stopped.  It removes old sensor
 * offset from the next straight/reverse segment without redefining startup
 * zero. */
#define GREEDY_IMU_LOCAL_SETTLE_MS             300U
#define GREEDY_IMU_LOCAL_REQUIRED_FRAMES         6U
#define GREEDY_IMU_LOCAL_STABILITY_RAW         273 /* About 1.5 degrees. */
#define GREEDY_IMU_LOCAL_MAX_AGE_MS            200U
#define GREEDY_IMU_LOCAL_TIMEOUT_MS           2000U
#define GREEDY_IMU_LOCAL_PERIOD_MS           30000U

/* Startup yaw capture has three destinations. STANDBY is used at power-up:
 * IMU capture begins immediately, while the five-minute clock and Greedy wait
 * until Bluetooth Q explicitly starts the match. */
#define IMU_ZERO_DEST_SCAN                       0U
#define IMU_ZERO_DEST_UNLOAD                     1U
#define IMU_ZERO_DEST_STANDBY                    2U

/* Reactive combat patrol does not trust a long-lived world-coordinate route.
 * After an empty 360-degree scan it drives one local straight segment, while
 * continuously accepting OpenMV targets and blue-wall avoidance, then scans
 * again.  The time cap prevents a missing wheel encoder from driving forever. */
#define COMBAT_PATROL_DISTANCE_MM             1000.0f
#define COMBAT_PATROL_FORWARD_PERCENT             32
#define COMBAT_PATROL_TIMEOUT_MS               9000U
#define COMBAT_PATROL_SETTLE_MS                 250U
#define POST_UNLOAD_PACE_DISTANCE_MM           300.0f
#define POST_UNLOAD_PACE_PERCENT                    20
#define POST_UNLOAD_PACE_LEG_TIMEOUT_MS         2500U

/* visual_latest_20260908.py sends decimal normalized X and decimal percent.
 * vision_protocol.c converts them to X x1000 and percentage x10. */
#define BLACK_VISION_START_X10           30U  /* 3.0% for 3 packets: target found. */
#define BLACK_VISION_ARRIVE_X10         400U  /* 40.0% for 3 new packets: turn around. */
#define BLACK_VISION_STALE_MS           700U
#define BLACK_VISION_CONFIRM_PACKETS      3U
#define BLACK_VISION_CENTER_DEADBAND     80   /* 8% of half image width. */
#define BLACK_VISION_APPROACH_PERCENT    16
#define BLACK_VISION_STEER_MAX           14
#define BLACK_VISION_APPROACH_TIMEOUT_MS 20000U
#define BLACK_RETURN_CCW_90_RAD           (ROBOT_PI * 0.5f)
#define BLACK_RETURN_REVERSE_PERCENT      12
#define BLACK_RETURN_FORWARD_PERCENT      12
#define BLACK_RETURN_REAR_STOP_MM        180U   /* Rear wall stop threshold. */
#define BLACK_RETURN_REAR_CONFIRM_PACKETS  3U
#define BLACK_RETURN_ALIGN_TIMEOUT_MS  12000U /* IMU zero/CCW90 alignment. */
#define BLACK_RETURN_REVERSE_TIMEOUT_MS 20000U /* Slow 12% reverse to wall. */
#define BLACK_RETURN_FORWARD_TIMEOUT_MS 35000U /* Slow forward black search. */
#define BLACK_ALIGN_PRESETTLE_MS         300U
#define BLACK_ALIGN_STABLE_FRAMES          5U
#define BLACK_ALIGN_STABILITY_RAD       0.02618f /* 1.5 degrees/frame. */
#define BLACK_HEADING_TOLERANCE_RAD       0.03491f /* 2 degrees. */
#define BLACK_HEADING_SETTLE_MS          500U
#define BLACK_HEADING_PID_KP              18.0f /* Other alignment steps. */
#define BLACK_TURN_PID_KP                 24.0f /* Dedicated 90/CCW30 turns. */
#define BLACK_HEADING_PID_KI               0.30f
#define BLACK_HEADING_PID_KD               0.80f
#define BLACK_HEADING_PID_I_LIMIT           0.40f
#define BLACK_HEADING_PID_MIN_PERCENT         10
#define BLACK_HEADING_PID_MAX_PERCENT         20
#define BLACK_LIVE_REPORT_MS              500U
#define BLACK_TURN_TIMEOUT_MS              12000U /* Two-wheel IMU CCW 30-degree turn. */
#define BLACK_REVERSE_PERCENT            18
#define BLACK_REVERSE_DISTANCE_MM        300.0f
#define BLACK_REVERSE_FALLBACK_MS        10000U /* Post-turn reverse fallback. */
#define BLACK_REAR_SONAR_STOP_MM         120U
/* Once the black zone is confirmed, first release the front intake from the
 * wall by backing up a short, measured distance before starting the
 * two-wheel initial-heading alignment and CCW 30-degree turn.  The post-turn
 * reverse has sensor/encoder stops plus a timed eject
 * fallback, so a missing rear sensor cannot leave the vehicle indefinitely. */
#define BLACK_PRE_TURN_BACKUP_PERCENT       BLACK_REVERSE_PERCENT
#define BLACK_PRE_TURN_BACKUP_DISTANCE_MM  80.0f
#define BLACK_PRE_TURN_BACKUP_TIMEOUT_MS   5000U /* 80 mm encoder backup. */
#define BLACK_UNLOAD_CCW_30_RAD           (ROBOT_PI * (30.0f / 180.0f))

/* During the wall-line search there is no front range sensor.  Treat either
 * commanded-forward drive wheel remaining nearly stopped as a contact rather
 * than requiring a simultaneous IMU impact.  The persistence window rejects
 * normal encoder quantisation/startup transients. */
#define BLACK_BUMP_ARM_MS                  500U
#define BLACK_BUMP_CONFIRM_MS              250U
#define BLACK_BUMP_MAX_TARGET_RATIO          0.30f
#define BLACK_BUMP_MAX_STALL_RPM              3.0f
#define BLACK_BUMP_STOP_MS                    80U
#define BLACK_BUMP_BACKUP_PERCENT             10
#define BLACK_BUMP_BACKUP_DISTANCE_MM         40.0f
#define BLACK_BUMP_BACKUP_TIMEOUT_MS         800U
#define BLACK_BUMP_RIGHT_RAD                  0.17453f /* 10 degrees clockwise. */
#define BLACK_BUMP_ESCAPE_DISTANCE_MM        100.0f
#define BLACK_BUMP_ESCAPE_TIMEOUT_MS        1500U
#define BLACK_BUMP_MAX_ATTEMPTS                 5U
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
static uint8_t imu_rx_byte;
static volatile uint32_t imu_uart_errors, imu_rearm_errors;
static volatile uint8_t imu_rx_retry;
static uint32_t imu_report_tick, imu_zero_after_frame;
static uint8_t imu_zero_valid;
static int16_t imu_yaw_reference;
static ImuNavigation_t imu_navigation;
static ImuNavigation_t imu_local_navigation;
static uint8_t greedy_imu_zero_pending;
static uint8_t greedy_imu_zero_to_unload;
static uint8_t greedy_imu_zero_samples;
static uint32_t greedy_imu_zero_start_tick;
static uint32_t greedy_imu_zero_last_frame;
static int16_t greedy_imu_zero_anchor_raw;
static uint8_t greedy_imu_local_pending;
static uint8_t greedy_imu_local_resume_scan;
static uint8_t greedy_imu_local_samples;
static uint8_t greedy_imu_local_reference_valid;
static uint32_t greedy_imu_local_start_tick;
static uint32_t greedy_imu_local_last_frame;
static uint32_t greedy_imu_local_last_refresh_tick;
static int16_t greedy_imu_local_anchor_raw;
static uint8_t straight_hold_active;
static uint8_t straight_hold_target_valid;
static uint8_t straight_hold_use_start_reference;
static int16_t straight_hold_base_percent;
static float straight_hold_target_heading;
static float straight_hold_integral;
static uint32_t straight_hold_tick;
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
static uint8_t vision_wall_valid;
static uint8_t vision_wall_raw;
static uint8_t vision_wall_confirmed;
static uint8_t vision_wall_hit_packets;
static uint8_t vision_wall_clear_packets;
static uint32_t vision_wall_tick;
static VisionBlackZone_t latest_black_zone;
static uint32_t latest_black_tick;
static uint8_t black_start_packets;
static uint8_t black_arrive_packets;
static uint32_t black_start_consumed_tick;
static uint32_t black_arrive_consumed_tick;
static uint32_t black_live_report_tick;
static uint32_t black_heading_settle_tick;
static uint32_t black_rear_consumed_tick;
static uint8_t black_rear_close_packets;
static float black_return_initial_heading_rad;
/* Empty-scan return uses two separate CCW 90-degree IMU turns: phase 1
 * before the rear-wall reverse, phase 2 after the 180 mm sonar stop. */
static uint8_t black_return_turn_phase;
static uint8_t black_return_heading_valid;
static uint8_t black_align_input_ready;
static uint8_t black_align_stable_samples;
static uint32_t black_align_last_frame;
static uint32_t black_align_reject_count;
static float black_align_last_heading_rad;
static BlackHeadingPid_t black_heading_pid;
static float unload_turn_angle_rad;
static float unload_turn_start_heading_rad;
static float unload_turn_target_heading_rad;
static float unload_pre_turn_left_mm;
static float unload_pre_turn_right_mm;
static float unload_reverse_left_mm;
static float unload_reverse_right_mm;
static uint8_t unload_sonar_close_packets;
static BlackBumpPhase_t black_bump_phase;
static uint32_t black_bump_phase_tick;
static uint32_t black_bump_suspect_tick;
static uint8_t black_bump_suspect_active;
static uint8_t black_bump_attempts;
static float black_bump_left_mm;
static float black_bump_right_mm;
static float black_search_heading_rad;
static CollisionRecoveryPhase_t collision_recovery_phase;
static uint32_t collision_recovery_phase_tick;
static uint8_t collision_monitor_active;
static uint32_t collision_monitor_tick;
static float collision_monitor_left_mm;
static float collision_monitor_right_mm;
static float collision_backup_left_mm;
static float collision_backup_right_mm;
static float collision_turn_angle_rad;
static float scan_accumulated_angle;
static uint16_t current_target_id;
static uint8_t plan_dirty;
static uint8_t empty_plan_retry_count;
static uint8_t greedy_active;
static uint8_t greedy_scan_turning;
static uint8_t greedy_scan_steps;
static uint32_t greedy_scan_phase_tick;
static float greedy_scan_step_start_angle;
static float greedy_collect_goal_mm;
static float greedy_collect_left_mm;
static float greedy_collect_right_mm;
static const char *greedy_action = "SEARCH";
static float combat_patrol_x_mm;
static float combat_patrol_y_mm;
static uint8_t combat_autonomy_enabled;
static uint8_t combat_final_return_started;
static uint8_t combat_finish_reported;
/* Q starts the only match clock. P changes collection policy without ever
 * restarting that clock. Once P commits an empty-scan unload, Q cannot cancel
 * unloading or the post-unload pacing sequence. */
static uint8_t combat_p_empty_unload_mode;
static uint8_t combat_p_unload_committed;
static uint8_t combat_p_unload_completed;
static float post_unload_pace_left_mm;
static float post_unload_pace_right_mm;
static float post_unload_pace_heading_rad;
static uint32_t post_unload_pace_leg_tick;
static float combat_patrol_left_mm;
static float combat_patrol_right_mm;
static AutoState_t auto_state = AUTO_IDLE;
static volatile uint8_t uart_packet_active;
static volatile uint8_t bluetooth_queue[BLUETOOTH_QUEUE_SIZE];
static volatile uint8_t bluetooth_read_index;
static volatile uint8_t bluetooth_write_index;
static volatile uint8_t bluetooth_stop_pending;
static volatile uint32_t bluetooth_drop_count;
static uint32_t vision_report_tick;
static uint8_t vision_report_due = 1U;
/* Per-observation result for diagnostics: raw detections are always reported. */
static const char *vision_filter_reason[VISION_MAX_TARGETS];
static const char *debug_action = "OFF";
static int16_t debug_target_index = -1;

static volatile uint32_t hc_distance_mm;
static uint32_t hc_last_sample_tick;
static uint8_t hc_close_count;
static uint8_t ultrasonic_monitor_active;
static uint32_t ultrasonic_report_tick;
/*最近一次有效距离，0=无效*/
/* 连续近距离计数，防毛刺*/

/* 蓝牙 G 命令手动测试舵机时置 1，测试结束或进入自动卸货后清 0 */
static uint8_t servo_test_active;

static CollectorRecoveryState_t collector_recovery_state = COLLECTOR_IDLE;
static AutoState_t collector_resume_state = AUTO_IDLE;
static uint32_t collector_phase_tick;
static float collector_left_travel_mm;
static float collector_right_travel_mm;
static uint8_t collector_backup_done;
static uint8_t collector_advance_pending;
static float collector_advance_left_mm, collector_advance_right_mm;
static uint32_t collector_reverse_start_count, collector_reverse_start_errors;
static uint32_t collector_reverse_progress, collector_reverse_progress_tick;
static uint32_t collector_healthy_tick;
static uint32_t collector_hold_report_tick;
static uint32_t brush_error_warn_tick;
static uint8_t collector_use_encoder, collector_auto_cycle, collector_auto_attempts;
static uint8_t collector_healthy_tracking;

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
static void Ultrasonic_SendStatus(uint32_t now);
static void Ultrasonic_MonitorUpdate(uint32_t now);
static void Buzzer_Init(void);
static void Bluetooth_QueueFromISR(uint8_t command);
static uint8_t Bluetooth_GetCommand(uint8_t *command);
static void Bluetooth_ProcessPending(void);
static void Vision_ReportTelemetry(uint8_t force);
static void Debug_Start(void);
static void IMU_CopySnapshot(JY901Snapshot *out);
static void IMU_FormatAngle(char *out, size_t size, int32_t raw);
static void IMU_StartRelativeReference(uint32_t now, uint8_t capture_fresh_now);
static void IMU_TryCaptureRelativeReference(const JY901Snapshot *snapshot,
                                            uint32_t now);
static void IMU_SendStatus(uint32_t now);
static void Debug_Update(uint32_t now);
static void IMU_NavigationUpdate(uint32_t now);
static void GreedyImuZero_Start(uint32_t now, uint8_t destination);
static void GreedyImuZero_Update(uint32_t now);
static void GreedyImuLocal_Start(uint32_t now, uint8_t resume_scan);
static void GreedyImuLocal_Update(uint32_t now);
static void Competition_InitializeStandby(void);
static void CombatGreedy_StartOrResume(uint32_t now);
static void CombatGreedy_StartPatrol(uint32_t now);
static void CombatGreedy_UpdatePatrol(uint32_t now);
static void Combat_StartFinalUnload(uint32_t now, const char *reason);
static uint8_t CombatMission_Service(uint32_t now);
static void PostUnloadPace_Start(uint32_t now);
static void PostUnloadPace_Update(uint32_t now);
static void Greedy_SendStatus(uint8_t detailed);
static void Greedy_UpdateLocal(uint32_t now);
static void Greedy_StartPickup(const VisionTarget_t *observation, uint32_t now);
static void Greedy_UpdateScan(uint32_t now);
static void StraightHold_Reset(void);
static void StraightHold_Service(uint32_t now);
static uint8_t Auto_AvailableCount(void);
static const char *Robot_ModeName(void);
static void Robot_UpdateOdometry(int32_t left_delta, int32_t right_delta);
static void Vision_ProcessIncoming(void);
static uint8_t Vision_WallFresh(uint32_t now);
static void CollisionMonitor_Reset(uint32_t now);
static void CollisionMonitor_RecordTravel(float left_mm, float right_mm);
static uint8_t CollisionMonitor_Update(uint8_t forward_commanded, uint32_t now);
static void CollisionRecovery_Start(uint32_t now);
static void CollisionRecovery_Update(uint32_t now);
static void BlackHeadingPid_Reset(uint32_t now);
static int16_t BlackHeadingPid_CommandWithKp(float error, uint32_t now,
                                             float kp);
static uint8_t BlackUnload_AlignWithKp(float target_heading, uint32_t now,
                                       float kp);
static uint8_t BlackUnload_Align(float target_heading, uint32_t now);
static void BlackAlignStable_Reset(void);
static uint8_t BlackAlignStable_Ready(uint32_t now);
static void BlackUnload_StartStraight(int16_t speed,
                                      float target_heading,
                                      uint32_t now);
static void BlackBump_Reset(uint8_t reset_attempts);
static void BlackBump_RecordTravel(float left_mm, float right_mm);
static uint8_t BlackBump_Update(uint32_t now);
static void BlackUnload_Update(uint32_t now);
static void Vision_ApplyFrame(const VisionFrame_t *frame);
static void Vision_SendMode(char mode, uint8_t color);
static void Auto_StartScan(uint32_t timeout_ms);
/* Retained for host regression of the legacy technical-mode safety path; the
 * Bluetooth combat build intentionally never calls it. */
static void Auto_Start(void) __attribute__((unused));
static void Auto_Stop(uint8_t clear_map);
static void Auto_Update(void);
static void Auto_BuildPlan(void);
static void Auto_DriveTowardLocal(float forward_mm, float left_mm,
                                  int16_t base_speed);
static void Auto_SendMap(void);
static void Auto_SendPlan(void) __attribute__((unused));
static const VisionTarget_t *Auto_FindCurrentVisionTarget(void);
static const char *Auto_StateName(AutoState_t state);
static void Mission_UpdateIntegration(void);
static void CollectorDirection_Init(void);
void motor3_forward(uint16_t speed);
static void motor3_stop(void);
static void motor3_reverse(uint16_t speed);
static void CollectorRecovery_Start(void);
static void CollectorRecovery_Update(void);
static void CollectorRecovery_Abort(void);
static void CollectorRecovery_Restore(void);
static void CollectorRecovery_RecordTravel(float left_mm, float right_mm);
static void CollectorRecovery_SendStatus(void);
static const char *CollectorRecovery_StateName(void);
static void CollectorFeedback_Update(void);
static void CollectorFeedback_SendStatus(void);
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

}

static void Bluetooth_QueueFromISR(uint8_t command)
{
  uint8_t next;
  if (command >= 'a' && command <= 'z') command -= ('a' - 'A');
  if (command == '\r' || command == '\n' || command == '$') return;
  if (command == 'S' || command == 'X' || command == '0' || command == 'H')
  {
    bluetooth_read_index = bluetooth_write_index;
    if (bluetooth_stop_pending != 'X') bluetooth_stop_pending = command;
    return;
  }
  /* A queued stop wins over subsequent motion in the same input burst. */
  if (bluetooth_stop_pending != 0U) return;
  next = (uint8_t)((bluetooth_write_index + 1U) % BLUETOOTH_QUEUE_SIZE);
  if (next == bluetooth_read_index)
  {
    bluetooth_drop_count++;
    bluetooth_read_index = bluetooth_write_index;
    bluetooth_stop_pending = 'S';
    return;
  }
  bluetooth_queue[bluetooth_write_index] = command;
  bluetooth_write_index = next;
}

static uint8_t Bluetooth_GetCommand(uint8_t *command)
{
  uint8_t available = 0U;
  uint32_t irq_mask = __get_PRIMASK();
  __disable_irq();
  if (bluetooth_stop_pending != 0U)
  {
    *command = bluetooth_stop_pending;
    bluetooth_stop_pending = 0U;
    bluetooth_read_index = bluetooth_write_index;
    available = 1U;
  }
  else if (bluetooth_read_index != bluetooth_write_index)
  {
    *command = bluetooth_queue[bluetooth_read_index];
    bluetooth_read_index = (uint8_t)((bluetooth_read_index + 1U) % BLUETOOTH_QUEUE_SIZE);
    available = 1U;
  }
  __set_PRIMASK(irq_mask);
  return available;
}

static void Bluetooth_ProcessPending(void)
{
  uint8_t command;
  uint8_t budget = 4U;
  while (budget-- != 0U && Bluetooth_GetCommand(&command) != 0U)
    Motor_ProcessCommand(command);
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
    case AUTO_DEBUG:       return "DEBUG";
    case AUTO_UNJAM:       return "UNJAM";
    case AUTO_UNLOAD_WAIT: return "UNLOAD_START";
    case AUTO_UNLOAD_ALIGN_ZERO: return "UNLOAD_ALIGN_ZERO";
    case AUTO_UNLOAD_REVERSE_WALL: return "UNLOAD_REVERSE_WALL";
    case AUTO_UNLOAD_TURN_CCW_90: return "UNLOAD_CCW_90";
    case AUTO_UNLOAD_SEARCH_FORWARD: return "UNLOAD_SEARCH_FORWARD";
    case AUTO_UNLOAD_APPROACH: return "UNLOAD_BLACK_APPROACH";
    case AUTO_UNLOAD_BACKUP_BEFORE_ALIGN: return "UNLOAD_BACKUP_80";
    case AUTO_UNLOAD_ALIGN_INITIAL: return "UNLOAD_ALIGN_INITIAL";
    case AUTO_UNLOAD_TURN_CCW_30: return "UNLOAD_TURN_CCW30";
    case AUTO_UNLOAD_REVERSE: return "UNLOAD_REVERSE";
    case AUTO_COLLISION_RECOVER: return "COLLISION_RECOVER";
    case AUTO_POST_UNLOAD_PACE_FORWARD: return "PACE_FORWARD_300";
    case AUTO_POST_UNLOAD_PACE_REVERSE: return "PACE_REVERSE_300";
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

static void StraightHold_Reset(void)
{
  straight_hold_active = 0U;
  straight_hold_target_valid = 0U;
  straight_hold_use_start_reference = 0U;
  straight_hold_base_percent = 0;
  straight_hold_integral = 0.0f;
  straight_hold_tick = HAL_GetTick();
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
    /* A short search pulse must not spend its entire duration at 80% boost.
     * Keep the normal >=60% PWM dead-zone mapping for these pulses. */
    if (!(greedy_active &&
          (auto_state == AUTO_SCAN || auto_state == AUTO_UNLOAD_WAIT ||
           auto_state == AUTO_UNLOAD_ALIGN_ZERO ||
           auto_state == AUTO_UNLOAD_REVERSE_WALL ||
           auto_state == AUTO_UNLOAD_TURN_CCW_90 ||
           auto_state == AUTO_UNLOAD_SEARCH_FORWARD ||
           auto_state == AUTO_UNLOAD_APPROACH ||
           auto_state == AUTO_UNLOAD_BACKUP_BEFORE_ALIGN ||
           auto_state == AUTO_UNLOAD_ALIGN_INITIAL ||
           auto_state == AUTO_UNLOAD_TURN_CCW_30 ||
           auto_state == AUTO_UNLOAD_REVERSE)) &&
        (HAL_GetTick() - *boost_start) < MOTOR_BOOST_DURATION_MS)
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
  int16_t old_left = left_target_percent;
  int16_t old_right = right_target_percent;

  /* Equal-wheel travel uses the IMU straight-hold loop. Black return heading
   * turns use a separate bounded PID in BlackUnload_Align; ordinary scans and
   * pose estimation remain encoder based. */
  if (left_speed == right_speed && left_speed != 0 &&
      auto_state != AUTO_UNLOAD_APPROACH)
  {
    uint32_t now = HAL_GetTick();
    const ImuNavigation_t *hold_navigation;
    if (straight_hold_active == 0U || straight_hold_base_percent != left_speed)
    {
      straight_hold_active = 1U;
      straight_hold_target_valid = 0U;
      straight_hold_use_start_reference = 0U;
      straight_hold_base_percent = left_speed;
      straight_hold_integral = 0.0f;
      straight_hold_tick = now;
    }
    hold_navigation = (straight_hold_use_start_reference == 0U &&
                       greedy_active != 0U &&
                       greedy_imu_local_reference_valid != 0U) ?
                          &imu_local_navigation : &imu_navigation;
    if (ImuNavigation_Fresh(hold_navigation, now))
    {
      float error;
      float correction;
      float dt = (float)(now - straight_hold_tick) / 1000.0f;
      if (straight_hold_target_valid == 0U)
      {
        straight_hold_target_heading = hold_navigation->heading_rad;
        straight_hold_target_valid = 1U;
      }
      if (dt < 0.005f) dt = 0.005f;
      if (dt > 0.100f) dt = 0.100f;
      error = Planner_NormalizeAngle(straight_hold_target_heading -
                                     hold_navigation->heading_rad);
      if (Robot_AbsFloat(error) <= STRAIGHT_IMU_DEADBAND_RAD)
      {
        straight_hold_integral = 0.0f;
        correction = 0.0f;
      }
      else
      {
        straight_hold_integral += error * dt;
        if (straight_hold_integral > STRAIGHT_IMU_I_LIMIT)
          straight_hold_integral = STRAIGHT_IMU_I_LIMIT;
        if (straight_hold_integral < -STRAIGHT_IMU_I_LIMIT)
          straight_hold_integral = -STRAIGHT_IMU_I_LIMIT;
        correction = STRAIGHT_IMU_KP * error +
                     STRAIGHT_IMU_KI * straight_hold_integral;
        if (correction > STRAIGHT_IMU_MAX_CORRECTION)
          correction = STRAIGHT_IMU_MAX_CORRECTION;
        if (correction < -STRAIGHT_IMU_MAX_CORRECTION)
          correction = -STRAIGHT_IMU_MAX_CORRECTION;
      }
      left_speed -= (int16_t)(correction >= 0.0f ? correction + 0.5f : correction - 0.5f);
      right_speed += (int16_t)(correction >= 0.0f ? correction + 0.5f : correction - 0.5f);
    }
    straight_hold_tick = now;
  }
  else
  {
    StraightHold_Reset();
  }
  left_target_percent = Motor_ClampPercent(left_speed);
  right_target_percent = Motor_ClampPercent(right_speed);
  motor_running = (left_target_percent != 0) || (right_target_percent != 0);

#if MOTOR_CLOSED_LOOP_ENABLE == 1U
  /* Do not carry integral memory through a stop or direction reversal. */
  if (left_target_percent == 0 || old_left == 0 ||
      ((left_target_percent > 0) != (old_left > 0)))
    Motor_ResetPid(&left_pid);
  if (right_target_percent == 0 || old_right == 0 ||
      ((right_target_percent > 0) != (old_right > 0)))
    Motor_ResetPid(&right_pid);
#endif

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
  StraightHold_Reset();
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
  char reply[24];
  if ((command >= 'a') && (command <= 'z'))
  {
    command = (uint8_t)(command - ('a' - 'A'));
  }

#if COMPETITION_GREEDY_ONLY == 1U
  /* Keep the competition controls plus the explicit D unload test reachable
   * when the restricted command set is selected. */
  if (command != 'Q' && command != 'P' && command != 'C' && command != 'D' &&
      command != 'S' && command != 'Z' && command != 'X' && command != '0' &&
      command != 'H' && command != 'V' && command != '\r' && command != '\n')
  {
    Motor_SendText("COMPETITION: CMD IGNORED; Q=TIMED P=EMPTY_UNLOAD S=WHEELS Z=BRUSH V=STATUS\r\n");
    return;
  }
#endif

  if (ultrasonic_monitor_active != 0U && command != 'U' && command != 'V' &&
      command != '\r' && command != '\n')
    ultrasonic_monitor_active = 0U;

  /* The recovery owns both wheel and brush outputs. A queued drive/mode
   * command cannot override it; stopping explicitly cancels all resumption. */
  if (command == 'S' || command == 'X' || command == '0' || command == 'H' || command == 'O')
  {
    BrushFeedback_SetEnabled(0U, HAL_GetTick());
    collector_auto_attempts = collector_healthy_tracking = 0U;
  }
  if (collector_recovery_state != COLLECTOR_IDLE)
  {
    if (command == 'S' || command == 'X' || command == '0')
      CollectorRecovery_Abort();
    else if (command == 'V' && auto_state != AUTO_DEBUG &&
             ultrasonic_monitor_active == 0U)
    {
      CollectorRecovery_SendStatus();
      CollectorFeedback_SendStatus();
      return;
    }
    else if (command != 'J' && command != 'K' && command != 'H' && command != 'O' &&
             command != 'N' && command != 'T' &&
             !((command == 'G' || command == 'I' || command == 'U' ||
                (command == 'V' && (auto_state == AUTO_DEBUG || ultrasonic_monitor_active))) &&
               collector_recovery_state == COLLECTOR_HOLD) &&
             command != '\r' && command != '\n')
    {
      Motor_SendText(collector_recovery_state == COLLECTOR_HOLD ?
          "JAM_HOLD: COMMAND RECEIVED; V=STATUS, CHECK FAULT THEN K=RESTORE; WAIT BEFORE Q\r\n" :
          "UNJAM BUSY: S=ABORT, J=RETRY IN HOLD, K=RESTORE IN HOLD\r\n");
      return;
    }
  }

  /* The power-up IMU capture defines the immutable match heading. Driving
   * during these stationary samples would save a false zero. Q remains
   * available and will begin scanning as soon as calibration completes. */
  if (greedy_imu_zero_pending != 0U &&
      greedy_imu_zero_to_unload == IMU_ZERO_DEST_STANDBY &&
      (command == 'F' || command == 'B' || command == 'L' || command == 'R'))
  {
    Motor_Stop();
    Motor_SendText("BOOT IMU ZERO BUSY: KEEP CAR STILL; MOTION REJECTED\r\n");
    return;
  }
  if (combat_final_return_started != 0U &&
      (command == 'F' || command == 'B' || command == 'L' || command == 'R'))
  {
    Motor_SendText("FINAL RETURN OWNS WHEELS: F/B/L/R REJECTED; S=PAUSE X=ABORT\r\n");
    return;
  }

  if ((command >= '0') && (command <= '9'))
  {
    motor_speed_percent = (uint8_t)(command - '0') * 10U;

    if (motor_speed_percent == 0U)
    {
      servo_test_active = 0U;
      Servo_Stop();
      Auto_Stop(0U);
      combat_autonomy_enabled = 0U;
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
      combat_autonomy_enabled = 0U;
      Motor_ApplyDriveCommand(command);
      if (command == 'S')
        Motor_SendText("WHEELS STOPPED BY S; BRUSH OUTPUT UNCHANGED\r\n");
      else
      {
        (void)snprintf(reply, sizeof(reply), "CMD=%c OK\r\n", command);
        Motor_SendText(reply);
      }
      if (CombatStrategy_IsActive() != 0U)
        Motor_SendText("COMBAT MANUAL OVERRIDE: MATCH CLOCK CONTINUES; Q=TIMED P=EMPTY_UNLOAD\r\n");
      break;

    case 'Z':
    {
      BrushFeedbackSnapshot_t feedback;
      if (combat_final_return_started != 0U ||
          (auto_state >= AUTO_UNLOAD_WAIT && auto_state <= AUTO_UNLOAD_REVERSE))
      {
        Motor_SendText("Z REJECTED: UNLOAD OWNS BRUSH; S=STOP WHEELS X=ABORT\r\n");
        break;
      }
      feedback = BrushFeedback_Snapshot(HAL_GetTick());
      if (feedback.drive != 0)
      {
        motor3_stop();
        BrushFeedback_SetEnabled(0U, HAL_GetTick());
        collector_auto_attempts = collector_healthy_tracking = 0U;
        Motor_SendText("BRUSH STOPPED BY Z; WHEELS/AUTO UNCHANGED\r\n");
      }
      else
      {
        motor3_forward(COLLECTOR_RUN_PWM);
        BrushFeedback_ArmForCollection(HAL_GetTick());
        Motor_SendText("BRUSH FORWARD BY Z; WHEELS/AUTO UNCHANGED\r\n");
      }
      break;
    }

    case 'A':
      Motor_Stop();
      Motor_SendText("A DISABLED IN COMBAT BUILD; Q=TIMED P=EMPTY_UNLOAD\r\n");
      break;

    case 'C':
      if (CombatStrategy_IsActive() == 0U)
        Motor_SendText("C REJECTED: Q MUST START THE MATCH CLOCK FIRST\r\n");
      else if (combat_p_unload_committed != 0U)
        Motor_SendText("C REJECTED: P UNLOAD/PACING ALREADY COMMITTED\r\n");
      else
      {
        combat_p_empty_unload_mode = 0U;
        CombatGreedy_StartOrResume(HAL_GetTick());
      }
      break;

    case 'D':
    {
      uint32_t now = HAL_GetTick();
      servo_test_active = 0U;
      Servo_Stop();
      Combat_StartFinalUnload(now, "BLUETOOTH D TEST");
      Motor_SendText("MODE=D_BLACK_UNLOAD: MATCH CLOCK CONTINUES\r\n");
      break;
    }

    case 'I':
      Debug_Start();
      Motor_SendText("MODE=DEBUG_IMU: JY901S PA10 RX; WHEELS STOPPED; I=NEW YAW REF V=STATUS S=EXIT\r\n");
      break;

    case 'U':
    {
      uint32_t now = HAL_GetTick();
      servo_test_active = 0U;
      Servo_Stop();
      Auto_Stop(0U);
      combat_autonomy_enabled = 0U;
      ultrasonic_monitor_active = 1U;
      ultrasonic_report_tick = now - 200U;
      hc_last_sample_tick = now - HC_SAMPLE_MS;
      Motor_SendText("MODE=REAR_SONAR: WHEELS STOPPED; LIVE DISTANCE EVERY 200MS; V=NOW S=EXIT\r\n");
      break;
    }

    case 'Q':
    {
      if (combat_p_unload_committed != 0U)
        Motor_SendText("Q REJECTED: P UNLOAD/PACING ALREADY COMMITTED\r\n");
      else
      {
        combat_p_empty_unload_mode = 0U;
        CombatGreedy_StartOrResume(HAL_GetTick());
      }
      break;
    }

    case 'J':
      CollectorRecovery_Start();
      break;

    case 'K':
      CollectorRecovery_Restore();
      break;

    case 'H':
      CollectorRecovery_Abort();
      Motor_SendText("BRUSH CALIBRATION HOLD: N=ZERO, TURN SHAFT ONCE, T=TEACH, K=RUN\r\n");
      break;

    case 'N':
      if (collector_recovery_state == COLLECTOR_HOLD)
      {
        BrushFeedback_Zero();
        Motor_SendText("BRUSH COUNTER ZEROED\r\n");
      }
      else Motor_SendText("N REQUIRES H: BRUSH STOPPED\r\n");
      break;

    case 'T':
      if (collector_recovery_state == COLLECTOR_HOLD && BrushFeedback_TeachOneTurn())
        Motor_SendText("BRUSH CPR TAUGHT IN RAM; SAVE VALUE IN CONFIG FOR NEXT BOOT\r\n");
      else Motor_SendText("TEACH REJECTED: H,N, ONE SHAFT TURN; CHECK COUNTS/ERRORS\r\n");
      CollectorFeedback_SendStatus();
      break;

    case 'E':
    {
      BrushFeedbackSnapshot_t feedback = BrushFeedback_Snapshot(HAL_GetTick());
      if (feedback.counts_per_rev >= 4U && feedback.drive > 0 &&
          feedback.rpm_x10 >= BRUSH_STALL_RPM_X10 && feedback.errors == 0U)
      {
        BrushFeedback_SetEnabled(1U, HAL_GetTick());
        collector_auto_attempts = collector_healthy_tracking = 0U;
        Motor_SendText("AUTO_UNJAM ARMED\r\n");
      }
      else Motor_SendText("ARM REJECTED: CALIBRATE CPR, CHECK POSITIVE FORWARD RPM/ENC ERRORS\r\n");
      CollectorFeedback_SendStatus();
      break;
    }

    case 'O':
      if (collector_recovery_state != COLLECTOR_IDLE) CollectorRecovery_Abort();
      Motor_SendText("AUTO_UNJAM DISARMED\r\n");
      break;

    case 'X':
      servo_test_active = 0U;
      Servo_Stop();
      Auto_Stop(1U);
      if (CombatStrategy_IsActive() != 0U)
        Motor_SendText("AUTO STOPPED, MAP CLEARED; MATCH CLOCK STILL RUNNING FROM FIRST Q\r\n");
      else
        Motor_SendText("AUTO STOPPED, MAP CLEARED; MATCH CLOCK WAITS FOR Q\r\n");
      break;

    case 'M':
      Auto_SendMap();
      break;

    case 'P':
    {
      uint32_t now = HAL_GetTick();
      if (CombatStrategy_IsActive() == 0U)
      {
        Motor_SendText("P REJECTED: SEND Q ONCE TO START THE MATCH CLOCK\r\n");
      }
      else if (CombatStrategy_MatchFinished(now) != 0U)
      {
        Motor_SendText("P REJECTED: 300S MATCH FINISHED\r\n");
      }
      else if (combat_p_unload_committed != 0U)
      {
        if (combat_p_unload_completed != 0U && auto_state == AUTO_IDLE)
          PostUnloadPace_Start(now);
        else if (combat_p_unload_completed == 0U && auto_state == AUTO_IDLE)
          Combat_StartFinalUnload(now, "P RESUME UNLOAD");
        else
          Motor_SendText("P ACTIVE: UNLOAD OR 300MM PACING CONTINUES; MATCH CLOCK UNCHANGED\r\n");
      }
      else
      {
        combat_p_empty_unload_mode = 1U;
        CombatGreedy_StartOrResume(now);
        /* P changes only the collection/unload policy.  A valid startup/Q
         * heading remains the immutable reference for the later return.  The
         * helper still performs its existing invalid-reference fallback if
         * no global reference is available. */
        Motor_SendText("MODE=P: GREEDY; IMU ZERO PRESERVED; FIRST EMPTY 6X60 SCAN UNLOADS; TIMER PRESERVED\r\n");
      }
      break;
    }

    case 'V':
      if (ultrasonic_monitor_active != 0U) { Ultrasonic_SendStatus(HAL_GetTick()); break; }
      if (auto_state == AUTO_DEBUG) { IMU_SendStatus(HAL_GetTick()); break; }
      Motor_SendStatus();
      Vision_ReportTelemetry(1U);
      break;

    case 'G':
      /* A bench hatch test always cancels wheel/unload motion, even if the
       * last command was manual F/B. It never clears a brush fault HOLD. */
      Auto_Stop(0U);
      combat_autonomy_enabled = 0U;
      if (servo_test_active != 0U)
      {
        servo_test_active = 0U;
        Servo_Stop();
        Motor_SendText("SERVO OFF: NEUTRAL 1500us\r\n");
      }
      else
      {
        servo_test_active = 1U;
        Servo_StartEject();
        Motor_SendText("SERVO ON: PA8 SWEEP TEST; G=NEUTRAL, S=STOP\r\n");
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
  BrushFeedbackSnapshot_t brush = BrushFeedback_Snapshot(HAL_GetTick());
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
                 (unsigned int)Auto_AvailableCount(),
                 (unsigned int)latest_vision_frame.sequence,
                 (unsigned long)vision_age_ms,
                 (unsigned long)accepted_vision_frame_count,
                 (unsigned long)VisionProtocol_GetErrorCount(),
                 (unsigned int)vision_motion_hold);
  Motor_SendText(status);
  CollectorRecovery_SendStatus();
  CollectorFeedback_SendStatus();
  (void)snprintf(status, sizeof(status),
                 "UNLOAD=%s PAYLOAD=%u SERVO=%s REAR=%lu mm BRUSH=%s PWM=%u/1000\r\n",
                 MissionExtension_UnloadStateName(),
                 (unsigned int)MissionExtension_HasPayload(),
                 (Servo_IsRunning() != 0U) ? "RUN" : "OFF",
                 (unsigned long)hc_distance_mm,
                 brush.drive != 0 ? (brush.drive > 0 ? "FORWARD" : "REVERSE") : "STOP",
                 (unsigned int)COLLECTOR_RUN_PWM);
  Motor_SendText(status);
  (void)snprintf(status, sizeof(status),
                 "COMBAT=%s CLOCK=%s ELAPSED=%lu/300000 POLICY=%s AUTO_EN=%u FINAL_RETURN=%u P_COMMIT=%u PAYLOAD=%u OPP_DWELL=%lu\r\n",
                 CombatStrategy_PhaseName(now),
                 CombatStrategy_IsActive() != 0U ? "RUN_FROM_Q" : "WAIT_Q",
                 (unsigned long)CombatStrategy_ElapsedMs(now),
                 combat_p_empty_unload_mode != 0U ? "P_EMPTY_UNLOAD" : "Q_TIMED",
                 (unsigned int)combat_autonomy_enabled,
                 (unsigned int)combat_final_return_started,
                 (unsigned int)combat_p_unload_committed,
                 (unsigned int)CombatStrategy_GetPayloadCount(),
                 (unsigned long)CombatStrategy_OpponentDwellMs(now));
  Motor_SendText(status);
  (void)snprintf(status, sizeof(status),
                 "MODE=%s DBG=%s PICK=%d BTDROP=%lu VWALL_DIAG=%s WRAW=%u WAGE=%lu COLLISION=ENCODER_ONLY\r\n",
                 Robot_ModeName(), debug_action, (int)debug_target_index,
                 (unsigned long)bluetooth_drop_count,
                 Vision_WallFresh(now) == 0U ? "STALE" :
                   (vision_wall_confirmed != 0U ? "HIT" : "CLEAR"),
                 (unsigned int)vision_wall_raw,
                 vision_wall_valid != 0U ?
                   (unsigned long)(now - vision_wall_tick) : 0xFFFFFFFFUL);
  Motor_SendText(status);
  (void)snprintf(status, sizeof(status),
                 "IMU_NAV=%s HEADING_MRAD=%ld REF=%u REF_RAW=%d REJECT=%lu STALE_REC=%lu SIGN=%d\r\n",
                 ImuNavigation_Fresh(&imu_navigation, now) ? "OK" :
                   (imu_navigation.valid ? "STALE" : "WAIT"),
                 (long)(imu_navigation.heading_rad * 1000.0f),
                 (unsigned)imu_navigation.reference_valid,
                 (int)imu_navigation.reference_yaw_raw,
                 (unsigned long)imu_navigation.rejected_steps,
                 (unsigned long)imu_navigation.stale_recoveries,
                 IMU_NAV_YAW_SIGN > 0.0f ? 1 : -1);
  Motor_SendText(status);
  (void)snprintf(status, sizeof(status),
      "IMU_ROLE=START_REF+LOCAL_STRAIGHT+BLACK_RETURN ZERO=%s %u/%u ALIGN_STABLE=%u/%u ALIGN_ERR_MRAD=%ld HOLD=%s TARGET_MRAD=%ld\r\n",
      greedy_imu_zero_pending != 0U ? "WAIT" :
        (black_return_heading_valid != 0U ? "LOCKED" : "OFF"),
      (unsigned)greedy_imu_zero_samples,
      (unsigned)GREEDY_IMU_ZERO_REQUIRED_FRAMES,
      (unsigned)black_align_stable_samples,
      (unsigned)BLACK_ALIGN_STABLE_FRAMES,
      (long)(Planner_NormalizeAngle(black_return_initial_heading_rad -
                                    imu_navigation.heading_rad) * 1000.0f),
      straight_hold_active == 0U ? "OFF" :
        (straight_hold_target_valid != 0U ? "ON" : "WAIT"),
      (long)(straight_hold_target_heading * 1000.0f));
  Motor_SendText(status);
  (void)snprintf(status, sizeof(status),
      "IMU_DUAL START_RAW=%d LOCAL=%s LOCAL_RAW=%d SAMPLE=%u/%u LOCAL_AGE=%lu ms\r\n",
      (int)imu_navigation.reference_yaw_raw,
      greedy_imu_local_pending != 0U ? "REFRESH" :
        (greedy_imu_local_reference_valid != 0U ? "LOCKED" : "WAIT"),
      (int)imu_local_navigation.reference_yaw_raw,
      (unsigned)greedy_imu_local_samples,
      (unsigned)GREEDY_IMU_LOCAL_REQUIRED_FRAMES,
      greedy_imu_local_reference_valid != 0U ?
        (unsigned long)(now - greedy_imu_local_last_refresh_tick) :
        0xFFFFFFFFUL);
  Motor_SendText(status);
  (void)snprintf(status, sizeof(status),
      "BLACK SEEN=%u X=%d Y=%d COVER_X10=%u AGE=%ld START=%u/3 ARRIVE=%u/3 BUMP=%u/%u TURN_MRAD=%ld REV=%ld/%ld\r\n",
      (unsigned)latest_black_zone.seen,
      (int)latest_black_zone.center_x_milli,
      (int)latest_black_zone.center_y_milli,
      (unsigned)latest_black_zone.coverage_x10,
      latest_black_tick != 0U ? (long)(now - latest_black_tick) : -1L,
      (unsigned)black_start_packets, (unsigned)black_arrive_packets,
      (unsigned)black_bump_phase, (unsigned)black_bump_attempts,
      (long)(unload_turn_angle_rad * 1000.0f),
      (long)unload_reverse_left_mm, (long)unload_reverse_right_mm);
  Motor_SendText(status);
}

static void StraightHold_Service(uint32_t now)
{
  if (straight_hold_active == 0U || straight_hold_base_percent == 0) return;
  if ((uint32_t)(now - straight_hold_tick) < AUTO_UPDATE_PERIOD_MS) return;
  Motor_SetTarget(straight_hold_base_percent, straight_hold_base_percent);
}

static const char *Robot_ModeName(void)
{
  if (ultrasonic_monitor_active != 0U) return "REAR_SONAR";
  if (auto_state == AUTO_DEBUG) return "DEBUG_IMU";
  if (collector_recovery_state == COLLECTOR_HOLD) return "JAM_HOLD";
  if (collector_recovery_state != COLLECTOR_IDLE) return "UNJAM";
  if (CombatStrategy_IsActive() != 0U && greedy_active) return "COMBAT_GREEDY";
  if (CombatStrategy_IsActive() != 0U && auto_state == AUTO_IDLE) return "COMBAT_STANDBY";
  if (greedy_active) return "GREEDY";
  if (CombatStrategy_IsActive() != 0U) return "COMBAT";
  if (auto_state == AUTO_IDLE) return "MANUAL";
  return "TECH";
}

static void Ultrasonic_SendStatus(uint32_t now)
{
  char line[128];
  uint32_t distance = hc_distance_mm;
  uint32_t age = now - hc_last_sample_tick;
  if (distance == 0U)
    (void)snprintf(line, sizeof(line),
        "SONAR REAR=NO_ECHO VALID=0 AGE=%lu ms STOP=%u mm\r\n",
        (unsigned long)age, (unsigned int)HC_STOP_MM);
  else
    (void)snprintf(line, sizeof(line),
        "SONAR REAR=%lu mm VALID=1 AGE=%lu ms CLOSE=%u STOP=%u mm\r\n",
        (unsigned long)distance, (unsigned long)age,
        (unsigned int)(distance <= HC_STOP_MM), (unsigned int)HC_STOP_MM);
  Motor_SendText(line);
  ultrasonic_report_tick = now;
}

static void Ultrasonic_MonitorUpdate(uint32_t now)
{
  if (ultrasonic_monitor_active != 0U &&
      (uint32_t)(now - ultrasonic_report_tick) >= 200U)
    Ultrasonic_SendStatus(now);
}

static void Vision_ReportTelemetry(uint8_t force)
{
  if (auto_state == AUTO_DEBUG) return;
  /* Avoid multi-line blocking UART reports while controlling short backup.
   * J transition reports and V's short recovery status remain available. */
  if (collector_recovery_state != COLLECTOR_IDLE) return;

  char line[176];
  uint8_t index;
  uint32_t now = HAL_GetTick();
  uint32_t age = now - latest_vision_tick;
  uint32_t timeout = auto_state == AUTO_DEBUG ? DEBUG_VISION_TIMEOUT_MS : AUTO_VISION_TIMEOUT_MS;
  if (greedy_active && !force &&
      (uint32_t)(now - vision_report_tick) < GREEDY_LOCAL_REPORT_MS) return;
  if (force == 0U && vision_report_due == 0U &&
      (uint32_t)(now - vision_report_tick) < VISION_REPORT_PERIOD_MS) return;
  vision_report_tick = now;
  vision_report_due = 0U;
  if (greedy_active) Greedy_SendStatus(0U);
  if (accepted_vision_frame_count == 0U)
  {
    (void)snprintf(line, sizeof(line), "VISION NO_FRAME RXERR=%lu MODE=%s DBG=%s\r\n",
                   (unsigned long)VisionProtocol_GetErrorCount(), Robot_ModeName(), debug_action);
    Motor_SendText(line);
    return;
  }
  (void)snprintf(line, sizeof(line),
      "VISION %s SEQ=%u RAW=%u MAP=%u AGE=%lu RXERR=%lu MODE=%s DBG=%s PICK=%d\r\n",
      age > timeout ? "STALE" : "OK", (unsigned int)latest_vision_frame.sequence,
      (unsigned int)latest_vision_frame.target_count,
      (unsigned int)Auto_AvailableCount(), (unsigned long)age,
      (unsigned long)VisionProtocol_GetErrorCount(), Robot_ModeName(), debug_action,
      (int)debug_target_index);
  Motor_SendText(line);
  if (age > timeout) return;
  if (greedy_active && !force) return; /* Full per-target detail only on V. */
  for (index = 0U; index < latest_vision_frame.target_count; index++)
  {
    const VisionTarget_t *target = &latest_vision_frame.targets[index];
    (void)snprintf(line, sizeof(line),
        "SEEN I=%u C=%u X=%d Y=%u Q=%u PX=%lu FILTER=%s\r\n",
        (unsigned int)index, (unsigned int)target->color, (int)target->lateral_mm,
        (unsigned int)target->forward_mm, (unsigned int)target->quality,
        (unsigned long)target->pixels,
        vision_filter_reason[index] != NULL ? vision_filter_reason[index] : "UNKNOWN");
    Motor_SendText(line);
  }
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

  CollectorRecovery_RecordTravel(left_distance, right_distance);
  BlackBump_RecordTravel(left_distance, right_distance);
  CollisionMonitor_RecordTravel(left_distance, right_distance);
  if (greedy_active && auto_state == AUTO_COLLECT &&
      collector_recovery_state == COLLECTOR_IDLE && !vision_motion_hold)
  {
    /* Signed travel: reversing or pivoting must not complete a pickup. */
    greedy_collect_left_mm += left_distance;
    greedy_collect_right_mm += right_distance;
  }

  robot_pose.x_mm += centre_distance * Planner_Cos(midpoint_heading);
  robot_pose.y_mm += centre_distance * Planner_Sin(midpoint_heading);
  robot_pose.heading_rad = Planner_NormalizeAngle(
      robot_pose.heading_rad + heading_change);

  if (auto_state == AUTO_SCAN)
  {
    scan_accumulated_angle += greedy_active ? heading_change : Robot_AbsFloat(heading_change);
  }
  if (auto_state == AUTO_UNLOAD_BACKUP_BEFORE_ALIGN)
  {
    /* Count only correctly signed reverse travel during the new pre-turn
     * clearance.  Keep these counters separate from the post-turn reverse. */
    if (left_distance < 0.0f) unload_pre_turn_left_mm -= left_distance;
    if (right_distance < 0.0f) unload_pre_turn_right_mm -= right_distance;
  }
  if (auto_state == AUTO_UNLOAD_REVERSE)
  {
    /* Only correctly signed reverse travel counts toward entering the zone. */
    if (left_distance < 0.0f) unload_reverse_left_mm -= left_distance;
    if (right_distance < 0.0f) unload_reverse_right_mm -= right_distance;
  }
  if (auto_state == AUTO_COMBAT_PATROL)
  {
    if (left_distance > 0.0f) combat_patrol_left_mm += left_distance;
    if (right_distance > 0.0f) combat_patrol_right_mm += right_distance;
  }
  if (auto_state == AUTO_POST_UNLOAD_PACE_FORWARD)
  {
    if (left_distance > 0.0f) post_unload_pace_left_mm += left_distance;
    if (right_distance > 0.0f) post_unload_pace_right_mm += right_distance;
  }
  else if (auto_state == AUTO_POST_UNLOAD_PACE_REVERSE)
  {
    if (left_distance < 0.0f) post_unload_pace_left_mm -= left_distance;
    if (right_distance < 0.0f) post_unload_pace_right_mm -= right_distance;
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

  if (accepted_vision_frame_count == 0U ||
      (latest_vision_frame.target_count == 0U && frame->target_count != 0U))
    vision_report_due = 1U;
  latest_vision_frame = *frame;
  latest_vision_tick = now;
  accepted_vision_frame_count++;
  if (auto_state == AUTO_DEBUG) return; /* No camera-to-motion/map work in D. */
  if (collector_recovery_state != COLLECTOR_IDLE) return;
  if (greedy_active)
  {
    /* No world-map writes, no pose/arena filtering for camera-relative Q. */
    for (index = 0U; index < frame->target_count; index++)
      vision_filter_reason[index] = "LOCAL_ONLY";
    if (auto_state != AUTO_COLLECT && auto_state != AUTO_COMPLETE &&
        auto_state != AUTO_IDLE && auto_state != AUTO_UNLOAD_WAIT &&
        auto_state != AUTO_UNLOAD_ALIGN_ZERO &&
        auto_state != AUTO_UNLOAD_REVERSE_WALL &&
        auto_state != AUTO_UNLOAD_TURN_CCW_90 &&
        auto_state != AUTO_UNLOAD_SEARCH_FORWARD &&
        auto_state != AUTO_UNLOAD_APPROACH &&
        auto_state != AUTO_UNLOAD_BACKUP_BEFORE_ALIGN &&
        auto_state != AUTO_UNLOAD_ALIGN_INITIAL &&
        auto_state != AUTO_UNLOAD_TURN_CCW_30 &&
        auto_state != AUTO_COLLISION_RECOVER &&
        auto_state != AUTO_UNLOAD_REVERSE)
      GreedyLocal_Observe(frame, now);
    return;
  }
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

    vision_filter_reason[index] = "OK";
    if ((observation->forward_mm < VISION_MIN_FORWARD_MM) ||
        (observation->forward_mm > VISION_MAX_FORWARD_MM))
    {
      vision_filter_reason[index] = "RANGE";
      continue;
    }
    if (auto_state == AUTO_DEBUG)
    {
      vision_filter_reason[index] = "DEBUG_BYPASS";
      continue;
    }
    if (observation->quality < VISION_MIN_ACCEPTED_QUALITY)
    {
      vision_filter_reason[index] = "LOW_Q";
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
      vision_filter_reason[index] = "OUTSIDE";
      continue;
    }

    available_before = TargetMap_CountAvailable(&target_map);
    (void)TargetMap_Upsert(&target_map, observation->color,
                           global_x, global_y, observation->quality, now);
    available_after = TargetMap_CountAvailable(&target_map);
    if (available_after > available_before)
    {
      Motor_SendText("MAP TARGET CONFIRMED\r\n");
    }
  }
  plan_dirty = 1U;
}

static void Vision_ProcessIncoming(void)
{
  VisionFrame_t frame;
  uint8_t wall_hit;
  VisionBlackZone_t black_zone;

  VisionProtocol_Process();
  while (VisionProtocol_GetFrame(&frame) != 0U)
  {
    Vision_ApplyFrame(&frame);
  }
  while (VisionProtocol_GetWallState(&wall_hit) != 0U)
  {
    vision_wall_valid = 1U;
    vision_wall_raw = wall_hit;
    vision_wall_tick = HAL_GetTick();
    if (wall_hit != 0U)
    {
      vision_wall_clear_packets = 0U;
      if (vision_wall_hit_packets < VISION_WALL_CONFIRM_PACKETS)
        vision_wall_hit_packets++;
      if (vision_wall_hit_packets >= VISION_WALL_CONFIRM_PACKETS)
        vision_wall_confirmed = 1U;
    }
    else
    {
      vision_wall_hit_packets = 0U;
      if (vision_wall_clear_packets < VISION_WALL_CLEAR_PACKETS)
        vision_wall_clear_packets++;
      if (vision_wall_clear_packets >= VISION_WALL_CLEAR_PACKETS)
        vision_wall_confirmed = 0U;
    }
  }
  while (VisionProtocol_GetBlackZone(&black_zone) != 0U)
  {
    uint32_t now = HAL_GetTick();
    latest_black_zone = black_zone;
    latest_black_tick = now;
    if (greedy_active != 0U &&
        (uint32_t)(now - black_live_report_tick) >= BLACK_LIVE_REPORT_MS)
    {
      char report[144];
      black_live_report_tick = now;
      (void)snprintf(report, sizeof(report),
          "BLACK LIVE SEEN=%u COVER_X10=%u X=%d AGE=0 STATE=%s\r\n",
          (unsigned)black_zone.seen, (unsigned)black_zone.coverage_x10,
          (int)black_zone.center_x_milli, Auto_StateName(auto_state));
      Motor_SendText(report);
    }
  }
}

static uint8_t Vision_WallFresh(uint32_t now)
{
  return (uint8_t)(vision_wall_valid != 0U &&
                   (uint32_t)(now - vision_wall_tick) <= VISION_WALL_STALE_MS);
}

static void CollisionMonitor_Reset(uint32_t now)
{
  collision_monitor_active = 0U;
  collision_monitor_tick = now;
  collision_monitor_left_mm = 0.0f;
  collision_monitor_right_mm = 0.0f;
}

static void CollisionMonitor_RecordTravel(float left_mm, float right_mm)
{
  if (auto_state == AUTO_COLLISION_RECOVER)
  {
    if (collision_recovery_phase == COLLISION_RECOVER_BACKUP)
    {
      if (left_mm < 0.0f) collision_backup_left_mm -= left_mm;
      if (right_mm < 0.0f) collision_backup_right_mm -= right_mm;
    }
    else if (collision_recovery_phase == COLLISION_RECOVER_TURN &&
             left_mm < 0.0f && right_mm > 0.0f)
    {
      collision_turn_angle_rad += (right_mm - left_mm) / WHEEL_TRACK_MM;
    }
    return;
  }
  if (collision_monitor_active != 0U)
  {
    if (left_mm > 0.0f) collision_monitor_left_mm += left_mm;
    if (right_mm > 0.0f) collision_monitor_right_mm += right_mm;
  }
}

static uint8_t CollisionMonitor_Update(uint8_t forward_commanded, uint32_t now)
{
  if (forward_commanded == 0U)
  {
    CollisionMonitor_Reset(now);
    return 0U;
  }
  if (collision_monitor_active == 0U)
  {
    collision_monitor_active = 1U;
    collision_monitor_tick = now;
    collision_monitor_left_mm = 0.0f;
    collision_monitor_right_mm = 0.0f;
    return 0U;
  }
  /* As soon as either wheel has advanced 10 mm, begin a fresh 3 s window.
   * A collision is declared only if BOTH wheel distances stay below 10 mm. */
  if (collision_monitor_left_mm >= COLLISION_MIN_WHEEL_TRAVEL_MM ||
      collision_monitor_right_mm >= COLLISION_MIN_WHEEL_TRAVEL_MM)
  {
    collision_monitor_tick = now;
    collision_monitor_left_mm = 0.0f;
    collision_monitor_right_mm = 0.0f;
    return 0U;
  }
  return (uint8_t)((uint32_t)(now - collision_monitor_tick) >=
                   COLLISION_MONITOR_WINDOW_MS);
}

static void CollisionRecovery_Start(uint32_t now)
{
  CollisionMonitor_Reset(now);

  Motor_Stop();
  current_target_id = 0U;
  GreedyLocal_Reset(now, 0U);
  collision_backup_left_mm = 0.0f;
  collision_backup_right_mm = 0.0f;
  collision_turn_angle_rad = 0.0f;
  collision_recovery_phase = COLLISION_RECOVER_STOP;
  collision_recovery_phase_tick = now;
  auto_state = AUTO_COLLISION_RECOVER;
  auto_state_start_tick = now;
  greedy_action = "COLLISION_STOP";
  Motor_SendText("COLLISION: BOTH WHEELS <10MM/3S; BACKUP 50MM THEN CCW110\r\n");
}

static void CollisionRecovery_Update(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - collision_recovery_phase_tick);

  if (collision_recovery_phase == COLLISION_RECOVER_STOP)
  {
    Motor_Stop();
    if (elapsed >= COLLISION_RECOVER_STOP_MS)
    {
      collision_backup_left_mm = 0.0f;
      collision_backup_right_mm = 0.0f;
      collision_recovery_phase = COLLISION_RECOVER_BACKUP;
      collision_recovery_phase_tick = now;
      greedy_action = "COLLISION_BACK";
    }
    return;
  }
  if (collision_recovery_phase == COLLISION_RECOVER_BACKUP)
  {
    Motor_SetTarget(-COLLISION_RECOVER_BACKUP_PERCENT,
                    -COLLISION_RECOVER_BACKUP_PERCENT);
    if (collision_backup_left_mm >= COLLISION_RECOVER_BACKUP_MM &&
        collision_backup_right_mm >= COLLISION_RECOVER_BACKUP_MM)
    {
      Motor_Stop();
      collision_turn_angle_rad = 0.0f;
      collision_recovery_phase = COLLISION_RECOVER_TURN;
      collision_recovery_phase_tick = now;
      greedy_action = "COLLISION_CCW110";
    }
    else if (elapsed >= COLLISION_RECOVER_BACKUP_TIMEOUT_MS)
    {
      Motor_Stop();
      Auto_Stop(0U);
      combat_autonomy_enabled = 0U;
      Motor_SendText("COLLISION BACKUP TIMEOUT: AUTO PAUSED; CHECK ENCODERS/OBSTRUCTION\r\n");
    }
    return;
  }

  Motor_SetTarget(-COLLISION_RECOVER_TURN_PERCENT,
                   COLLISION_RECOVER_TURN_PERCENT);
  if (collision_turn_angle_rad >= COLLISION_RECOVER_TURN_RAD)
  {
    Motor_Stop();
    Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
    greedy_action = "SEARCH";
    CollisionMonitor_Reset(now);
    Motor_SendText("COLLISION RECOVERY COMPLETE: BACK50 CCW110; GREEDY RESCAN\r\n");
  }
  else if (elapsed >= COLLISION_RECOVER_TURN_TIMEOUT_MS)
  {
    Motor_Stop();
    Auto_Stop(0U);
    combat_autonomy_enabled = 0U;
    Motor_SendText("COLLISION TURN TIMEOUT: AUTO PAUSED; CHECK WHEEL ENCODERS\r\n");
  }
}

static void Auto_StartScan(uint32_t timeout_ms)
{
  scan_accumulated_angle = 0.0f;
  current_scan_timeout_ms = timeout_ms;
  auto_state = AUTO_SCAN;
  auto_state_start_tick = HAL_GetTick();
  greedy_scan_turning = 0U;
  greedy_scan_steps = 0U;
  greedy_scan_phase_tick = auto_state_start_tick;
  greedy_scan_step_start_angle = 0.0f;
  if (greedy_active)
  {
    GreedyLocal_Reset(auto_state_start_tick, 0U);
    current_target_id = 0U;
    greedy_action = "SEARCH";
  }
  Vision_SendMode('S', 0U);
  /* The next Auto_Update must pass the vision gate before wheel movement.
   * The front brush runs independently of this state machine. */
  Motor_Stop();
}

static void Greedy_UpdateScan(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - greedy_scan_phase_tick);
  float progress = scan_accumulated_angle - greedy_scan_step_start_angle;
  /* Thirty seconds is only a fallback.  Refresh at this already-stopped scan
   * boundary, never by interrupting a turn or a target approach. */
  if (greedy_scan_turning == 0U && greedy_scan_steps == 0U &&
      greedy_imu_local_reference_valid != 0U &&
      (uint32_t)(now - greedy_imu_local_last_refresh_tick) >=
          GREEDY_IMU_LOCAL_PERIOD_MS)
  {
    GreedyImuLocal_Start(now, 1U);
    return;
  }
  if (GreedyLocal_Visible(now))
  {
    Motor_Stop();
    current_target_id = 1U; /* Boolean lock token, not a global object ID. */
    auto_state = AUTO_NAVIGATE;
    auto_state_start_tick = now;
    return;
  }
  if (greedy_scan_turning)
  {
    if (progress >= GREEDY_SCAN_STEP_RAD)
    {
      char report[80];
      Motor_Stop();
      greedy_scan_turning = 0U;
      greedy_scan_steps++;
      greedy_scan_phase_tick = now;
      (void)snprintf(report, sizeof(report),
          "GREEDY SCAN ENCODER STEP=%u/6; OBSERVE\r\n",
          (unsigned)greedy_scan_steps);
      Motor_SendText(report);
      return;
    }
    if (elapsed >= GREEDY_SCAN_TURN_MAX_MS)
    {
      Motor_Stop();
      BrushFeedback_SetEnabled(0U, now);
      Auto_Stop(0U);
      Motor_SendText("GREEDY SCAN TIMEOUT: 60DEG NOT REACHED; CHECK WHEEL ENCODERS\r\n");
      return;
    }
    Motor_SetTarget(-GREEDY_SCAN_SPEED_PERCENT, GREEDY_SCAN_SPEED_PERCENT);
    return;
  }

  Motor_Stop();
  if (elapsed < GREEDY_SCAN_OBSERVE_MS) return;
  /* Six measured 60-degree turns, each followed by a short observation. */
  if (greedy_scan_steps >= GREEDY_SCAN_STEPS)
  {
    Motor_Stop();
    GreedyLocal_Reset(now, 0U);
    current_target_id = 0U;
    if (CombatStrategy_IsActive() != 0U &&
        combat_p_empty_unload_mode != 0U)
    {
      combat_p_unload_committed = 1U;
      Combat_StartFinalUnload(now, "P EMPTY 6X60 SCAN");
      Motor_SendText("MODE=P EMPTY SCAN: IMMEDIATE UNLOAD COMMITTED; TIMER UNCHANGED\r\n");
    }
    else if (CombatStrategy_IsActive() != 0U &&
        CombatStrategy_ElapsedMs(now) < COMBAT_ENDGAME_RETURN_MS)
    {
      CombatGreedy_StartPatrol(now);
    }
    else
    {
      Combat_StartFinalUnload(now, "EMPTY SCAN AFTER ENDGAME");
    }
    return;
  }
  greedy_scan_turning = 1U;
  greedy_scan_phase_tick = now;
  greedy_scan_step_start_angle = scan_accumulated_angle;
  Motor_SetTarget(-GREEDY_SCAN_SPEED_PERCENT, GREEDY_SCAN_SPEED_PERCENT);
}

static void Greedy_StartPickup(const VisionTarget_t *observation, uint32_t now)
{
  char report[112];
  auto_state = AUTO_COLLECT;
  auto_state_start_tick = now;
  greedy_action = "FEED";
  greedy_collect_goal_mm = (float)observation->forward_mm +
      CAMERA_FORWARD_OFFSET_MM + GREEDY_COLLECT_EXTRA_MM;
  if (greedy_collect_goal_mm < GREEDY_COLLECT_EXTRA_MM)
    greedy_collect_goal_mm = GREEDY_COLLECT_EXTRA_MM;
  greedy_collect_left_mm = greedy_collect_right_mm = 0.0f;
  (void)snprintf(report, sizeof(report),
      "GREEDY PICKUP START: GOAL=%lu EXTRA=%lu mm; WHEEL_DISTANCE\r\n",
      (unsigned long)greedy_collect_goal_mm, (unsigned long)GREEDY_COLLECT_EXTRA_MM);
  Motor_SendText(report);
  Motor_SetTarget(GREEDY_FORWARD_SPEED_PERCENT, GREEDY_FORWARD_SPEED_PERCENT);
}

static void Greedy_UpdateLocal(uint32_t now)
{
  const GreedyLocalTarget_t *track = GreedyLocal_Get();
  const VisionTarget_t *t = &track->target;
  float lateral;
  if (auto_state == AUTO_SCAN) { Greedy_UpdateScan(now); return; }
  if (auto_state == AUTO_PLAN) { Auto_BuildPlan(); return; }
  if (auto_state == AUTO_RECOVER || !track->locked)
  { Auto_StartScan(AUTO_SCAN_TIMEOUT_MS); return; }
  if (!GreedyLocal_Visible(now))
  {
    Motor_Stop();
    greedy_action = "LOST";
    /* Restore the original loss timeout; no extended reacquire state. */
    if ((uint32_t)(now - track->last_seen_ms) >= GREEDY_LOCAL_LOST_MS)
    {
      Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
      Motor_SendText("GREEDY TARGET LOST: RESCAN\r\n");
    }
    return;
  }
  if ((uint32_t)(now - track->locked_since_ms) >= GREEDY_LOCAL_FOLLOW_MAX_MS)
  {
    Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
    Motor_SendText("GREEDY FOLLOW TIMEOUT: STOP THEN RESCAN\r\n");
    return;
  }
  current_target_id = 1U;
  lateral = Robot_AbsFloat((float)t->lateral_mm);
  auto_state = t->forward_mm <= GREEDY_COLLECT_TRIGGER_MM ? AUTO_FINAL_ALIGN : AUTO_NAVIGATE;
  if (auto_state == AUTO_FINAL_ALIGN && lateral <= AUTO_ALIGN_TOLERANCE_MM)
  { Greedy_StartPickup(t, now); return; }
  /* Camera X>0 means right. Keep both wheels moving while correcting: a
   * one-wheel pivot used to make the car orbit the block and approach its
   * edge. The larger bounded differential compensates for the 60% physical
   * PWM floor. Slow down in final alignment so the intake can centre before
   * the block enters the camera blind zone. */
  {
    int16_t base_speed = auto_state == AUTO_FINAL_ALIGN ?
        GREEDY_ALIGN_SPEED_PERCENT : GREEDY_FORWARD_SPEED_PERCENT;
    int16_t turn_limit = auto_state == AUTO_FINAL_ALIGN ?
        GREEDY_ALIGN_STEER_MAX_PERCENT : GREEDY_STEER_MAX_PERCENT;
    int16_t turn = lateral <= AUTO_ALIGN_TOLERANCE_MM ? 0 :
        (int16_t)(GREEDY_STEER_GAIN * t->lateral_mm / t->forward_mm);
    int16_t left_command;
    int16_t right_command;
    if (turn > turn_limit) turn = turn_limit;
    if (turn < -turn_limit) turn = -turn_limit;
    left_command = base_speed + turn;
    right_command = base_speed - turn;
    /* Both commands stay positive; do not fall back to a pivot. */
    if (left_command < 1) left_command = 1;
    if (right_command < 1) right_command = 1;
    Motor_SetTarget(left_command, right_command);
    greedy_action = turn == 0 ? "FOLLOW" :
                    (turn > 0 ? "DIFF_RIGHT" : "DIFF_LEFT");
  }
}

static void Auto_Start(void)
{
  greedy_active = 0U;
  Greedy_Reset();
  GreedyLocal_Reset(HAL_GetTick(), 0U);
  greedy_action = "SEARCH";
  greedy_collect_goal_mm = 0.0f;
  greedy_collect_left_mm = greedy_collect_right_mm = 0.0f;
  debug_action = "OFF";
  debug_target_index = -1;
  vision_report_due = 1U;
  Motor_Stop();
  memset(&robot_pose, 0, sizeof(robot_pose));
  memset(&planned_route, 0, sizeof(planned_route));
  memset(&latest_vision_frame, 0, sizeof(latest_vision_frame));
  TargetMap_Reset(&target_map);
  MissionExtension_Reset();
  MissionExtension_SetInitialPose(&robot_pose);
  ImuNavigation_Reset(&imu_navigation);
  ImuNavigation_Reset(&imu_local_navigation);
  greedy_imu_zero_pending = 0U;
  greedy_imu_zero_to_unload = 0U;
  greedy_imu_zero_samples = 0U;
  greedy_imu_zero_start_tick = 0U;
  greedy_imu_zero_last_frame = 0U;
  greedy_imu_zero_anchor_raw = 0;
  greedy_imu_local_pending = 0U;
  greedy_imu_local_resume_scan = 0U;
  greedy_imu_local_samples = 0U;
  greedy_imu_local_reference_valid = 0U;
  greedy_imu_local_start_tick = 0U;
  greedy_imu_local_last_frame = 0U;
  greedy_imu_local_last_refresh_tick = 0U;
  greedy_imu_local_anchor_raw = 0;

  latest_vision_tick = 0U;
  accepted_vision_frame_count = 0U;
  vision_motion_hold = 0U;
  vision_retry_tick = 0U;
  vision_wall_valid = 0U;
  vision_wall_raw = 0U;
  vision_wall_confirmed = 0U;
  vision_wall_hit_packets = 0U;
  vision_wall_clear_packets = 0U;
  vision_wall_tick = 0U;
  memset(&latest_black_zone, 0, sizeof(latest_black_zone));
  latest_black_tick = 0U;
  black_start_packets = 0U;
  black_arrive_packets = 0U;
  black_start_consumed_tick = 0U;
  black_arrive_consumed_tick = 0U;
  black_live_report_tick = 0U;
  black_heading_settle_tick = 0U;
  black_rear_consumed_tick = 0U;
  black_rear_close_packets = 0U;
  black_return_initial_heading_rad = 0.0f;
  black_return_turn_phase = 0U;
  black_return_heading_valid = 0U;
  black_align_input_ready = 0U;
  black_align_stable_samples = 0U;
  black_align_last_frame = 0U;
  black_align_reject_count = 0U;
  black_align_last_heading_rad = 0.0f;
  memset(&black_heading_pid, 0, sizeof(black_heading_pid));
  unload_turn_angle_rad = 0.0f;
  unload_turn_start_heading_rad = 0.0f;
  unload_turn_target_heading_rad = 0.0f;
  unload_pre_turn_left_mm = unload_pre_turn_right_mm = 0.0f;
  unload_reverse_left_mm = unload_reverse_right_mm = 0.0f;
  unload_sonar_close_packets = 0U;
  black_search_heading_rad = 0.0f;
  BlackBump_Reset(1U);
  collision_recovery_phase = COLLISION_RECOVER_STOP;
  collision_recovery_phase_tick = 0U;
  CollisionMonitor_Reset(auto_state_start_tick);
  collision_backup_left_mm = collision_backup_right_mm = 0.0f;
  collision_turn_angle_rad = 0.0f;
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
  uint8_t preserve_match_imu = (uint8_t)(
      CombatStrategy_IsActive() != 0U &&
      CombatStrategy_MatchFinished(HAL_GetTick()) == 0U);
  CollisionMonitor_Reset(HAL_GetTick());
  greedy_active = 0U;
  if (preserve_match_imu == 0U)
  {
    ImuNavigation_Reset(&imu_navigation);
    ImuNavigation_Reset(&imu_local_navigation);
    greedy_imu_zero_pending = 0U;
    greedy_imu_zero_to_unload = IMU_ZERO_DEST_SCAN;
    greedy_imu_zero_samples = 0U;
    greedy_imu_local_pending = 0U;
    greedy_imu_local_resume_scan = 0U;
    greedy_imu_local_samples = 0U;
    greedy_imu_local_reference_valid = 0U;
    greedy_imu_local_last_refresh_tick = 0U;
  }
  else
  {
    /* Manual recovery pauses autonomy but the match clock and immutable
     * power-up yaw survive.  A pending startup capture also keeps running in
     * standby; the local straight reference is rebuilt when Q/P resumes. */
    if (greedy_imu_zero_pending != 0U)
      greedy_imu_zero_to_unload = IMU_ZERO_DEST_STANDBY;
    ImuNavigation_Reset(&imu_local_navigation);
    greedy_imu_local_pending = 0U;
    greedy_imu_local_resume_scan = 0U;
    greedy_imu_local_samples = 0U;
    greedy_imu_local_reference_valid = 0U;
    greedy_imu_local_last_refresh_tick = HAL_GetTick();
  }
  GreedyLocal_Reset(HAL_GetTick(), 0U);
  /* Manual takeover must also cancel a previous unloading/avoidance action. */
  MissionExtension_CancelMotion();
  debug_action = "OFF";
  debug_target_index = -1;
  vision_report_due = 1U;
  vision_motion_hold = 0U;
  vision_wall_confirmed = 0U;
  vision_wall_hit_packets = 0U;
  vision_wall_clear_packets = 0U;
  black_heading_settle_tick = 0U;
  black_rear_close_packets = 0U;
  black_start_consumed_tick = 0U;
  black_arrive_consumed_tick = 0U;
  black_return_initial_heading_rad = 0.0f;
  black_return_turn_phase = 0U;
  black_return_heading_valid = preserve_match_imu != 0U ?
      imu_navigation.reference_valid : 0U;
  black_align_input_ready = 0U;
  black_align_stable_samples = 0U;
  black_search_heading_rad = 0.0f;
  BlackBump_Reset(1U);
  memset(&black_heading_pid, 0, sizeof(black_heading_pid));
  unload_turn_angle_rad = 0.0f;
  unload_turn_start_heading_rad = 0.0f;
  unload_turn_target_heading_rad = 0.0f;
  unload_pre_turn_left_mm = unload_pre_turn_right_mm = 0.0f;
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

static void IMU_CopySnapshot(JY901Snapshot *out)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  JY901_Snapshot(out);
  __set_PRIMASK(mask);
}

static void IMU_FormatAngle(char *out, size_t size, int32_t raw)
{
  int32_t cdeg = JY901_AngleCdeg(raw);
  uint32_t magnitude = (uint32_t)(cdeg < 0 ? -cdeg : cdeg);
  (void)snprintf(out, size, "%s%lu.%02lu", cdeg < 0 ? "-" : "",
      (unsigned long)(magnitude / 100U), (unsigned long)(magnitude % 100U));
}

static void IMU_StartRelativeReference(uint32_t now,
                                       uint8_t capture_fresh_now)
{
  JY901Snapshot snapshot;
  IMU_CopySnapshot(&snapshot);
  now = HAL_GetTick(); /* The UART IRQ may have updated angle_tick meanwhile. */
  imu_zero_after_frame = snapshot.angle_frames;
  imu_zero_valid = 0U;
  if (capture_fresh_now != 0U && snapshot.angle_frames != 0U &&
      (uint32_t)(now - snapshot.angle_tick) <= JY901_STALE_MS)
  {
    imu_yaw_reference = snapshot.angle[2];
    imu_zero_valid = 1U;
  }
  imu_report_tick = now - JY901_REPORT_MS;
}

static void IMU_TryCaptureRelativeReference(const JY901Snapshot *snapshot,
                                            uint32_t now)
{
  if (imu_zero_valid == 0U && snapshot->angle_frames != 0U &&
      snapshot->angle_frames != imu_zero_after_frame &&
      (uint32_t)(now - snapshot->angle_tick) <= JY901_STALE_MS)
  {
    imu_yaw_reference = snapshot->angle[2];
    imu_zero_valid = 1U;
  }
}

static void IMU_SendStatus(uint32_t now)
{
  JY901Snapshot s;
  char line[240], roll[16], pitch[16], yaw[16], relative[16];
  IMU_CopySnapshot(&s);
  now = HAL_GetTick(); /* Snapshot may have been updated by an IRQ after caller's tick. */
  const char *state = JY901_Status(&s, now);
  if (auto_state == AUTO_DEBUG)
  {
    debug_action = state;
    IMU_TryCaptureRelativeReference(&s, now);
  }
  (void)snprintf(line, sizeof(line),
      "IMU %s BAUD=%lu RX=%lu FRAME=%lu ANGLE=%lu AGE=%ld CHECKERR=%lu UARTERR=%lu REARMERR=%lu\r\n",
      state, (unsigned long)JY901_UART_BAUD, (unsigned long)s.bytes,
      (unsigned long)s.frames, (unsigned long)s.angle_frames,
      s.angle_frames ? (long)(uint32_t)(now - s.angle_tick) : -1L,
      (unsigned long)s.checksum_errors, (unsigned long)imu_uart_errors,
      (unsigned long)imu_rearm_errors);
  Motor_SendText(line);
  if (s.angle_frames)
  {
    IMU_FormatAngle(roll, sizeof(roll), s.angle[0]);
    IMU_FormatAngle(pitch, sizeof(pitch), s.angle[1]);
    IMU_FormatAngle(yaw, sizeof(yaw), s.angle[2]);
    IMU_FormatAngle(relative, sizeof(relative),
        imu_zero_valid ? JY901_RelativeYawRaw(s.angle[2], imu_yaw_reference) : 0);
    (void)snprintf(line, sizeof(line),
        "IMU DEG ROLL=%s PITCH=%s YAW=%s REL_YAW=%s ZERO=%u GYRO_N=%lu GZ_CDEGPS=%ld GAGE=%ld\r\n",
        roll, pitch, yaw, relative, (unsigned)imu_zero_valid,
        (unsigned long)s.gyro_frames, (long)((int64_t)s.gyro[2] * 200000 / 32768),
        s.gyro_frames ? (long)(uint32_t)(now - s.gyro_tick) : -1L);
    Motor_SendText(line);
  }
  imu_report_tick = now;
}

static void Debug_Start(void)
{
  uint32_t now = HAL_GetTick();
  servo_test_active = 0U;
  Servo_Stop();
  Auto_Stop(0U);
  combat_autonomy_enabled = 0U;
  BrushFeedback_SetEnabled(0U, HAL_GetTick());
  auto_state = AUTO_DEBUG;
  debug_action = "IMU_WAIT";
  debug_target_index = -1;
  IMU_StartRelativeReference(now, 0U);
}

static void Debug_Update(uint32_t now)
{
  /* IMU diagnostics own no motion, even with no camera or in brush HOLD. */
  Motor_Stop();
  if ((uint32_t)(now - imu_report_tick) >= JY901_REPORT_MS)
    IMU_SendStatus(now);
}

/* Parse/navigation-filter the JY901 heading for straight hold and the black
 * return 0/90-degree PID. Do not fuse it into robot_pose. */
static void IMU_NavigationUpdate(uint32_t now)
{
  JY901Snapshot snapshot;
  IMU_CopySnapshot(&snapshot);
  (void)ImuNavigation_Update(&imu_navigation, &snapshot, now,
                             robot_pose.heading_rad);
  if (greedy_imu_local_reference_valid != 0U)
    (void)ImuNavigation_Update(&imu_local_navigation, &snapshot, now, 0.0f);
}

static void GreedyImuZero_Start(uint32_t now, uint8_t destination)
{
  Motor_Stop();
  ImuNavigation_Reset(&imu_navigation);
  ImuNavigation_Reset(&imu_local_navigation);
  greedy_imu_zero_pending = 1U;
  greedy_imu_zero_to_unload = destination;
  greedy_imu_zero_samples = 0U;
  greedy_imu_zero_start_tick = now;
  greedy_imu_zero_last_frame = 0U;
  greedy_imu_zero_anchor_raw = 0;
  greedy_imu_local_pending = 0U;
  greedy_imu_local_resume_scan = 0U;
  greedy_imu_local_samples = 0U;
  greedy_imu_local_reference_valid = 0U;
  greedy_imu_local_start_tick = 0U;
  greedy_imu_local_last_frame = 0U;
  greedy_imu_local_last_refresh_tick = now;
  greedy_imu_local_anchor_raw = 0;
  black_return_initial_heading_rad = 0.0f;
  black_return_turn_phase = 0U;
  black_return_heading_valid = 0U;
  imu_zero_valid = 0U;
  greedy_action = "IMU_ZERO_WAIT";
  if (destination == IMU_ZERO_DEST_UNLOAD)
  {
    auto_state = AUTO_UNLOAD_ALIGN_ZERO;
    auto_state_start_tick = now;
    Vision_SendMode('S', 0U);
  }
  else if (destination == IMU_ZERO_DEST_SCAN)
  {
    Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
    /* Auto_StartScan changes the action to SEARCH; zero capture still owns
     * the stopped wheels until ten stationary samples have been accepted. */
    greedy_action = "IMU_ZERO_WAIT";
  }
  else
  {
    auto_state = AUTO_IDLE;
    auto_state_start_tick = now;
    Vision_SendMode('S', 0U);
    greedy_action = "IMU_ZERO_STANDBY";
  }
}

static void GreedyImuZero_Update(uint32_t now)
{
  JY901Snapshot snapshot;
  int32_t movement_raw;
  Motor_Stop();
  IMU_CopySnapshot(&snapshot);

  if ((uint32_t)(now - greedy_imu_zero_start_tick) >=
      GREEDY_IMU_ZERO_TIMEOUT_MS)
  {
    if (greedy_imu_zero_to_unload == IMU_ZERO_DEST_STANDBY)
    {
      greedy_imu_zero_pending = 0U;
      greedy_imu_zero_samples = 0U;
      auto_state = AUTO_IDLE;
      greedy_action = "IMU_ZERO_FAULT";
      Motor_SendText("IMU START ZERO TIMEOUT: MATCH CLOCK UNCHANGED; CHECK PA10/9600 THEN Q\r\n");
    }
    else
    {
      BrushFeedback_SetEnabled(0U, now);
      Auto_Stop(0U);
      Motor_SendText("IMU START ZERO TIMEOUT: KEEP CAR STILL, CHECK PA10/9600; TASK STOPPED\r\n");
    }
    return;
  }
  if (snapshot.angle_frames == 0U ||
      snapshot.angle_frames == greedy_imu_zero_last_frame ||
      (uint32_t)(now - snapshot.angle_tick) > GREEDY_IMU_ZERO_MAX_AGE_MS)
    return;

  greedy_imu_zero_last_frame = snapshot.angle_frames;
  if (greedy_imu_zero_samples == 0U)
  {
    greedy_imu_zero_anchor_raw = snapshot.angle[2];
    greedy_imu_zero_samples = 1U;
    return;
  }

  movement_raw = JY901_RelativeYawRaw(snapshot.angle[2],
                                      greedy_imu_zero_anchor_raw);
  if (movement_raw < 0) movement_raw = -movement_raw;
  if (movement_raw > GREEDY_IMU_ZERO_STABILITY_RAW)
  {
    /* The vehicle or sensor moved during capture.  Restart the consecutive
     * stationary window instead of averaging a false startup direction. */
    greedy_imu_zero_anchor_raw = snapshot.angle[2];
    greedy_imu_zero_samples = 1U;
    greedy_action = "IMU_ZERO_MOVED";
    return;
  }
  if (greedy_imu_zero_samples < GREEDY_IMU_ZERO_REQUIRED_FRAMES)
    greedy_imu_zero_samples++;
  if (greedy_imu_zero_samples < GREEDY_IMU_ZERO_REQUIRED_FRAMES)
  {
    greedy_action = "IMU_ZERO_SAMPLE";
    return;
  }

  /* Use the most recent member of the stable window as the exact raw-yaw
   * reference.  Future headings are derived directly from this preserved
   * reference, so an IMU gap can no longer replace zero with odometry. */
  if (ImuNavigation_SetReference(&imu_navigation, &snapshot, now, 0.0f) == 0U)
    return;
  if (ImuNavigation_SetReference(&imu_local_navigation, &snapshot, now,
                                 0.0f) == 0U)
    return;
  imu_yaw_reference = snapshot.angle[2];
  imu_zero_valid = 1U;
  robot_pose.heading_rad = 0.0f;
  black_return_initial_heading_rad = 0.0f;
  black_return_turn_phase = 0U;
  black_return_heading_valid = 1U;
  greedy_imu_local_reference_valid = 1U;
  greedy_imu_local_last_refresh_tick = now;
  greedy_imu_zero_pending = 0U;

  if (greedy_imu_zero_to_unload == IMU_ZERO_DEST_UNLOAD)
  {
    auto_state = AUTO_UNLOAD_ALIGN_ZERO;
    auto_state_start_tick = now;
    BlackAlignStable_Reset();
    BlackHeadingPid_Reset(now);
    greedy_action = "ZERO_SETTLE";
    Motor_SendText("IMU START ZERO LOCKED 10/10: SETTLE THEN ALIGN INITIAL 0 DEG\r\n");
  }
  else if (greedy_imu_zero_to_unload == IMU_ZERO_DEST_SCAN)
  {
    Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
    Motor_SendText("IMU START ZERO LOCKED 10/10: GREEDY OBJECT SCAN START\r\n");
  }
  else
  {
    greedy_active = 0U;
    auto_state = AUTO_IDLE;
    auto_state_start_tick = now;
    greedy_action = "STANDBY";
    Motor_Stop();
    Motor_SendText("IMU START ZERO LOCKED 10/10: STANDBY; Q=START MATCH/GREEDY\r\n");
  }
}

static void GreedyImuLocal_Start(uint32_t now, uint8_t resume_scan)
{
  JY901Snapshot snapshot;
  if (greedy_active == 0U || imu_navigation.reference_valid == 0U ||
      greedy_imu_zero_pending != 0U || greedy_imu_local_pending != 0U)
    return;

  Motor_Stop();
  IMU_CopySnapshot(&snapshot);
  greedy_imu_local_pending = 1U;
  greedy_imu_local_resume_scan = resume_scan;
  greedy_imu_local_samples = 0U;
  greedy_imu_local_start_tick = now;
  greedy_imu_local_last_frame = snapshot.angle_frames;
  greedy_imu_local_anchor_raw = 0;
  greedy_action = "LOCAL_IMU_SETTLE";
  Motor_SendText("IMU LOCAL REFRESH: STOP 300MS, THEN 6 STABLE FRAMES; START ZERO PRESERVED\r\n");
}

static void GreedyImuLocal_Update(uint32_t now)
{
  JY901Snapshot snapshot;
  int32_t movement_raw;

  Motor_Stop();
  if ((uint32_t)(now - greedy_imu_local_start_tick) >=
      GREEDY_IMU_LOCAL_TIMEOUT_MS)
  {
    greedy_imu_local_pending = 0U;
    greedy_imu_local_samples = 0U;
    greedy_imu_local_last_refresh_tick = now;
    Motor_SendText("IMU LOCAL REFRESH TIMEOUT: KEEP OLD LOCAL REF; RESCAN\r\n");
    if (greedy_imu_local_resume_scan != 0U)
      Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
    return;
  }

  if ((uint32_t)(now - greedy_imu_local_start_tick) <
      GREEDY_IMU_LOCAL_SETTLE_MS)
    return;

  IMU_CopySnapshot(&snapshot);
  if (snapshot.angle_frames == 0U ||
      snapshot.angle_frames == greedy_imu_local_last_frame ||
      (uint32_t)(now - snapshot.angle_tick) > GREEDY_IMU_LOCAL_MAX_AGE_MS)
    return;

  greedy_imu_local_last_frame = snapshot.angle_frames;
  if (greedy_imu_local_samples == 0U)
  {
    greedy_imu_local_anchor_raw = snapshot.angle[2];
    greedy_imu_local_samples = 1U;
    greedy_action = "LOCAL_IMU_SAMPLE";
    return;
  }

  movement_raw = JY901_RelativeYawRaw(snapshot.angle[2],
                                      greedy_imu_local_anchor_raw);
  if (movement_raw < 0) movement_raw = -movement_raw;
  if (movement_raw > GREEDY_IMU_LOCAL_STABILITY_RAW)
  {
    greedy_imu_local_anchor_raw = snapshot.angle[2];
    greedy_imu_local_samples = 1U;
    greedy_action = "LOCAL_IMU_MOVED";
    return;
  }

  if (greedy_imu_local_samples < GREEDY_IMU_LOCAL_REQUIRED_FRAMES)
    greedy_imu_local_samples++;
  if (greedy_imu_local_samples < GREEDY_IMU_LOCAL_REQUIRED_FRAMES)
    return;

  if (ImuNavigation_SetReference(&imu_local_navigation, &snapshot, now,
                                 0.0f) == 0U)
    return;
  greedy_imu_local_reference_valid = 1U;
  greedy_imu_local_last_refresh_tick = now;
  greedy_imu_local_pending = 0U;
  greedy_imu_local_samples = 0U;
  StraightHold_Reset();
  Motor_SendText("IMU LOCAL REFRESH LOCKED 6/6: START ZERO UNCHANGED; RESCAN\r\n");
  if (greedy_imu_local_resume_scan != 0U)
    Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
}

static void Competition_InitializeStandby(void)
{
  uint32_t now = HAL_GetTick();

  /* IMU acquisition begins at power-up, but the five-minute match clock is
   * deliberately not started until the first Q command. */
  servo_test_active = 0U;
  Servo_Stop();
  Motor_Stop();
  motor3_stop();
  BrushFeedback_SetEnabled(0U, now);
  greedy_active = 0U;
  combat_autonomy_enabled = 0U;
  combat_final_return_started = 0U;
  combat_finish_reported = 0U;
  combat_p_empty_unload_mode = 0U;
  combat_p_unload_committed = 0U;
  combat_p_unload_completed = 0U;
  post_unload_pace_left_mm = 0.0f;
  post_unload_pace_right_mm = 0.0f;
  post_unload_pace_heading_rad = 0.0f;
  post_unload_pace_leg_tick = 0U;
  combat_patrol_left_mm = 0.0f;
  combat_patrol_right_mm = 0.0f;
  CombatStrategy_Stop();
  black_live_report_tick = now - BLACK_LIVE_REPORT_MS;
  collector_auto_attempts = 0U;
  collector_healthy_tracking = 0U;
  GreedyImuZero_Start(now, IMU_ZERO_DEST_STANDBY);
  Motor_SendText("MATCH CLOCK ARMED: FIRST Q STARTS 300S; P/C CANNOT START OR RESET IT\r\n");
  Motor_SendText("STANDBY: WHEELS/BRUSH STOPPED; KEEP STILL FOR IMU ZERO; Q=START TIMED GREEDY\r\n");
}

static void CombatGreedy_StartOrResume(uint32_t now)
{
  char report[112];

  if (CombatStrategy_IsActive() == 0U)
  {
    /* This path is reachable only from Q.  P and C are rejected before the
     * helper when no match clock exists. */
    CombatStrategy_Start(now);
    combat_final_return_started = 0U;
    combat_finish_reported = 0U;
    combat_p_unload_committed = 0U;
    combat_p_unload_completed = 0U;
    Motor_SendText("MATCH CLOCK STARTED BY FIRST Q: T=0/300000MS\r\n");
  }
  if (CombatStrategy_MatchFinished(now) != 0U)
  {
    Motor_Stop();
    Motor_SendText("COMBAT FINISHED: Q/P/C REJECTED; POWER-CYCLE FOR A NEW MATCH\r\n");
    return;
  }
  if (CombatStrategy_ElapsedMs(now) >= COMBAT_ENDGAME_RETURN_MS)
  {
    Combat_StartFinalUnload(now, "BLUETOOTH RESUME DURING FINAL MINUTE");
    return;
  }

  servo_test_active = 0U;
  Servo_Stop();
  MissionExtension_CancelMotion();
  Motor_Stop();
  /* A bench D test may have set this flag before minute four.  Returning to
   * Greedy restores normal collection and keeps the real 240 s transition
   * armed. */
  combat_final_return_started = 0U;
  greedy_active = 1U;
  combat_autonomy_enabled = 1U;
  Greedy_Reset();
  GreedyLocal_Reset(now, 0U);
  current_target_id = 0U;
  memset(&planned_route, 0, sizeof(planned_route));
  memset(&latest_vision_frame, 0, sizeof(latest_vision_frame));
  latest_vision_tick = 0U;
  accepted_vision_frame_count = 0U;
  vision_motion_hold = 0U;
  vision_retry_tick = now;
  plan_dirty = 1U;
  combat_patrol_left_mm = 0.0f;
  combat_patrol_right_mm = 0.0f;
  black_live_report_tick = now - BLACK_LIVE_REPORT_MS;
  collector_auto_attempts = 0U;
  collector_healthy_tracking = 0U;
  motor3_forward(COLLECTOR_RUN_PWM);
  BrushFeedback_ArmForCollection(now);

  if (greedy_imu_zero_pending != 0U)
  {
    greedy_imu_zero_to_unload = IMU_ZERO_DEST_SCAN;
    Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
    greedy_action = "IMU_ZERO_WAIT";
    Motor_SendText("COMBAT GREEDY ARMED: WAITING FOR POWER-UP IMU ZERO\r\n");
  }
  else if (imu_navigation.reference_valid != 0U)
  {
    Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
    GreedyImuLocal_Start(now, 1U);
    Motor_SendText("COMBAT GREEDY RESUME: START ZERO PRESERVED, LOCAL IMU REFRESH THEN SCAN\r\n");
  }
  else
  {
    GreedyImuZero_Start(now, IMU_ZERO_DEST_SCAN);
    Motor_SendText("COMBAT GREEDY START: IMU ZERO WAS INVALID; KEEP STILL FOR 10 FRAMES\r\n");
  }

  (void)snprintf(report, sizeof(report),
      "MATCH ELAPSED=%lu ms REMAIN_TO_RETURN=%lu ms; TIMER NOT RESET\r\n",
      (unsigned long)CombatStrategy_ElapsedMs(now),
      (unsigned long)(COMBAT_ENDGAME_RETURN_MS - CombatStrategy_ElapsedMs(now)));
  Motor_SendText(report);
}

static void CombatGreedy_StartPatrol(uint32_t now)
{
  Motor_Stop();
  GreedyLocal_Reset(now, 0U);
  current_target_id = 0U;
  combat_patrol_left_mm = 0.0f;
  combat_patrol_right_mm = 0.0f;
  auto_state = AUTO_COMBAT_PATROL;
  auto_state_start_tick = now;
  greedy_action = "PATROL_SETTLE";
  Vision_SendMode('S', 0U);
  Motor_SendText("COMBAT EMPTY SCAN: LOCAL 1000MM PATROL; LIVE TARGET/WALL WATCH\r\n");
}

static void CombatGreedy_UpdatePatrol(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - auto_state_start_tick);

  if (GreedyLocal_Visible(now))
  {
    Motor_Stop();
    current_target_id = 1U;
    auto_state = AUTO_NAVIGATE;
    auto_state_start_tick = now;
    greedy_action = "PATROL_TARGET";
    Motor_SendText("COMBAT PATROL TARGET SEEN: NAVIGATE\r\n");
    return;
  }
  if (elapsed < COMBAT_PATROL_SETTLE_MS)
  {
    Motor_Stop();
    return;
  }
  if ((combat_patrol_left_mm >= COMBAT_PATROL_DISTANCE_MM &&
       combat_patrol_right_mm >= COMBAT_PATROL_DISTANCE_MM) ||
      elapsed >= COMBAT_PATROL_TIMEOUT_MS)
  {
    Motor_Stop();
    Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
    Motor_SendText(elapsed >= COMBAT_PATROL_TIMEOUT_MS ?
        "COMBAT PATROL TIME CAP: STOP AND RESCAN; CHECK ENCODERS IF NO MOTION\r\n" :
        "COMBAT PATROL 1000MM COMPLETE: STOP AND RESCAN\r\n");
    return;
  }
  greedy_action = "PATROL_FORWARD";
  Motor_SetTarget(COMBAT_PATROL_FORWARD_PERCENT,
                  COMBAT_PATROL_FORWARD_PERCENT);
}

static void Combat_StartFinalUnload(uint32_t now, const char *reason)
{
  char report[152];
  uint8_t recovery_was_active =
      (uint8_t)(collector_recovery_state != COLLECTOR_IDLE);

  servo_test_active = 0U;
  Servo_Stop();
  MissionExtension_CancelMotion();
  Motor_Stop();
  /* D/endgame takes ownership away from any Q local-reference refresh.  If
   * this pending gate were left alive, its completion could call
   * Auto_StartScan() and overwrite the unload state immediately after D. */
  greedy_imu_local_pending = 0U;
  greedy_imu_local_resume_scan = 0U;
  greedy_imu_local_samples = 0U;
  StraightHold_Reset();
  /* Collection is over once the final-minute return owns the robot. Disable
   * the brush and its jam monitor so stale feedback cannot pre-empt return. */
  motor3_stop();
  BrushFeedback_SetEnabled(0U, now);
  /* The final minute has higher priority than an intake recovery/HOLD.  The
   * brush is already de-energised above, so cancel all remaining recovery
   * phases before the return state machine takes ownership of the wheels. */
  collector_recovery_state = COLLECTOR_IDLE;
  collector_auto_cycle = 0U;
  collector_use_encoder = 0U;
  collector_backup_done = 0U;
  collector_auto_attempts = 0U;
  collector_healthy_tracking = 0U;
  greedy_active = 1U;
  combat_autonomy_enabled = 1U;
  combat_final_return_started = 1U;
  if (combat_p_empty_unload_mode != 0U)
    combat_p_unload_committed = 1U;
  GreedyLocal_Reset(now, 0U);
  current_target_id = 0U;
  combat_patrol_left_mm = 0.0f;
  combat_patrol_right_mm = 0.0f;
  black_start_packets = black_arrive_packets = 0U;
  black_start_consumed_tick = black_arrive_consumed_tick = 0U;
  black_heading_settle_tick = 0U;
  black_rear_consumed_tick = hc_last_sample_tick;
  black_rear_close_packets = 0U;
  /* Align against the immutable startup/Q reference, not a newly sampled
   * local heading.  The normal startup reference heading is 0 rad, but using
   * the stored field also preserves a deliberately non-zero reference. */
  black_return_initial_heading_rad = imu_navigation.reference_valid != 0U ?
      imu_navigation.reference_heading_rad : 0.0f;
  black_return_turn_phase = 0U;
  black_return_heading_valid = imu_navigation.reference_valid;
  black_search_heading_rad = 0.0f;
  BlackBump_Reset(1U);
  BlackAlignStable_Reset();
  BlackHeadingPid_Reset(now);
  Vision_SendMode('S', 0U);

  if (imu_navigation.reference_valid == 0U)
  {
    GreedyImuZero_Start(now, IMU_ZERO_DEST_UNLOAD);
    Motor_SendText("COMBAT FINAL RETURN: IMU START REF INVALID; STOP FOR 10-FRAME ZERO\r\n");
  }
  else
  {
    greedy_imu_zero_pending = 0U;
    auto_state = AUTO_UNLOAD_ALIGN_ZERO;
    auto_state_start_tick = now;
    greedy_action = "ZERO_SETTLE";
  }
  (void)snprintf(report, sizeof(report),
      "COMBAT FINAL RETURN START ELAPSED=%lu ms REASON=%s\r\n",
      (unsigned long)CombatStrategy_ElapsedMs(now),
      reason != NULL ? reason : "ENDGAME");
  Motor_SendText(report);
  if (recovery_was_active != 0U)
    Motor_SendText("COMBAT ENDGAME PREEMPTED BRUSH RECOVERY/HOLD; BRUSH OFF\r\n");
}

static uint8_t CombatMission_Service(uint32_t now)
{
  if (CombatStrategy_IsActive() == 0U) return 0U;

  if (CombatStrategy_MatchFinished(now) != 0U)
  {
    Motor_Stop();
    motor3_stop();
    BrushFeedback_SetEnabled(0U, now);
    collector_recovery_state = COLLECTOR_IDLE;
    collector_auto_cycle = 0U;
    collector_advance_pending = 0U;
    Servo_Stop();
    MissionExtension_CancelMotion();
    greedy_active = 0U;
    combat_autonomy_enabled = 0U;
    auto_state = AUTO_COMPLETE;
    if (combat_finish_reported == 0U)
    {
      combat_finish_reported = 1U;
      Motor_SendText("COMBAT 300S FINISHED: ALL MOTION STOPPED\r\n");
    }
    return 1U;
  }

  if (CombatStrategy_ElapsedMs(now) >= COMBAT_ENDGAME_RETURN_MS &&
      combat_final_return_started == 0U)
  {
    Combat_StartFinalUnload(now, "240S ENDGAME");
    return 1U;
  }
  return 0U;
}

static void PostUnloadPace_Start(uint32_t now)
{
  Motor_Stop();
  /* P-mode pacing is still an active competition phase: keep the front
   * collection blades running and retain automatic jam monitoring. */
  motor3_forward(COLLECTOR_RUN_PWM);
  BrushFeedback_ArmForCollection(now);
  greedy_active = 0U;
  combat_autonomy_enabled = 1U;
  combat_p_unload_completed = 1U;
  post_unload_pace_left_mm = 0.0f;
  post_unload_pace_right_mm = 0.0f;
  post_unload_pace_heading_rad = ImuNavigation_Fresh(&imu_navigation, now) ?
      imu_navigation.heading_rad : robot_pose.heading_rad;
  post_unload_pace_leg_tick = now;
  auto_state = AUTO_POST_UNLOAD_PACE_FORWARD;
  auto_state_start_tick = now;
  greedy_action = "PACE_FORWARD_300";
  BlackUnload_StartStraight(POST_UNLOAD_PACE_PERCENT,
                            post_unload_pace_heading_rad, now);
  Motor_SendText("P UNLOAD COMPLETE: BRUSH FORWARD; FORWARD/REVERSE 300MM PACING UNTIL 300S\r\n");
}

static void PostUnloadPace_Update(uint32_t now)
{
  uint8_t distance_done = (uint8_t)(
      post_unload_pace_left_mm >= POST_UNLOAD_PACE_DISTANCE_MM &&
      post_unload_pace_right_mm >= POST_UNLOAD_PACE_DISTANCE_MM);
  uint8_t time_fallback = (uint8_t)(
      (uint32_t)(now - post_unload_pace_leg_tick) >=
      POST_UNLOAD_PACE_LEG_TIMEOUT_MS);
  int16_t speed = auto_state == AUTO_POST_UNLOAD_PACE_FORWARD ?
      POST_UNLOAD_PACE_PERCENT : -POST_UNLOAD_PACE_PERCENT;

  if (distance_done != 0U || time_fallback != 0U)
  {
    Motor_Stop();
    post_unload_pace_left_mm = 0.0f;
    post_unload_pace_right_mm = 0.0f;
    post_unload_pace_leg_tick = now;
    if (auto_state == AUTO_POST_UNLOAD_PACE_FORWARD)
    {
      auto_state = AUTO_POST_UNLOAD_PACE_REVERSE;
      greedy_action = "PACE_REVERSE_300";
      speed = -POST_UNLOAD_PACE_PERCENT;
      Motor_SendText(time_fallback != 0U ?
          "P PACE FORWARD TIME FALLBACK: REVERSE 300MM\r\n" :
          "P PACE FORWARD 300MM COMPLETE: REVERSE 300MM\r\n");
    }
    else
    {
      auto_state = AUTO_POST_UNLOAD_PACE_FORWARD;
      greedy_action = "PACE_FORWARD_300";
      speed = POST_UNLOAD_PACE_PERCENT;
      Motor_SendText(time_fallback != 0U ?
          "P PACE REVERSE TIME FALLBACK: FORWARD 300MM\r\n" :
          "P PACE REVERSE 300MM COMPLETE: FORWARD 300MM\r\n");
    }
    auto_state_start_tick = now;
    BlackUnload_StartStraight(speed, post_unload_pace_heading_rad, now);
    return;
  }

  if (straight_hold_active == 0U ||
      straight_hold_base_percent != speed ||
      straight_hold_target_valid == 0U)
    BlackUnload_StartStraight(speed, post_unload_pace_heading_rad, now);
  else
    Motor_SetTarget(speed, speed);
}

static void Auto_BuildPlan(void)
{
  uint32_t now = HAL_GetTick();
  if (greedy_active)
  {
    memset(&planned_route, 0, sizeof(planned_route));
    last_plan_tick = now;
    plan_dirty = 0U;
    current_target_id = 0U;
    if (!GreedyLocal_Visible(now))
    {
      Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
      Motor_SendText("GREEDY EMPTY: ONE FULL OBJECT SCAN BEFORE DIRECT WALL RETURN\r\n");
      return;
    }
    current_target_id = 1U;
    auto_state = AUTO_NAVIGATE;
    auto_state_start_tick = now;
    Vision_SendMode('S', 0U);
    return;
  }
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
  if ((auto_state == AUTO_IDLE) || (auto_state == AUTO_COMPLETE) ||
      (auto_state == AUTO_UNLOAD_WAIT) ||
      (auto_state == AUTO_UNLOAD_ALIGN_ZERO) ||
      (auto_state == AUTO_UNLOAD_REVERSE_WALL) ||
      (auto_state == AUTO_UNLOAD_TURN_CCW_90) ||
      (auto_state == AUTO_UNLOAD_SEARCH_FORWARD) ||
      (auto_state == AUTO_UNLOAD_APPROACH) ||
      (auto_state == AUTO_UNLOAD_BACKUP_BEFORE_ALIGN) ||
      (auto_state == AUTO_UNLOAD_ALIGN_INITIAL) ||
      (auto_state == AUTO_UNLOAD_TURN_CCW_30) ||
      (auto_state == AUTO_UNLOAD_REVERSE))
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
      if (greedy_active) Greedy_ClearPending();
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

static void BlackHeadingPid_Reset(uint32_t now)
{
  black_heading_pid.integral = 0.0f;
  black_heading_pid.previous_error = 0.0f;
  black_heading_pid.previous_tick = now;
  black_heading_pid.valid = 0U;
}

static int16_t BlackHeadingPid_CommandWithKp(float error, uint32_t now,
                                             float kp)
{
  float dt = 0.005f;
  float derivative = 0.0f;
  float output;
  float magnitude;

  if (black_heading_pid.valid != 0U)
  {
    dt = (float)(uint32_t)(now - black_heading_pid.previous_tick) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.100f) dt = 0.100f;
    derivative = (error - black_heading_pid.previous_error) / dt;
  }

  /* If the heading error is below 3 degrees, remove motor drive and let the
   * chassis coast into the existing 2-degree completion band.  Clearing the
   * integral also prevents an accumulated kick if the error leaves this
   * deadband again. */
  if (Robot_AbsFloat(error) < 0.0524f) /* 3 degrees = 0.0524 rad. */
  {
    black_heading_pid.integral = 0.0f;
    return 0;
  }

  black_heading_pid.integral += error * dt;
  if (black_heading_pid.integral > BLACK_HEADING_PID_I_LIMIT)
    black_heading_pid.integral = BLACK_HEADING_PID_I_LIMIT;
  if (black_heading_pid.integral < -BLACK_HEADING_PID_I_LIMIT)
    black_heading_pid.integral = -BLACK_HEADING_PID_I_LIMIT;
  output = kp * error +
           BLACK_HEADING_PID_KI * black_heading_pid.integral +
           BLACK_HEADING_PID_KD * derivative;
  black_heading_pid.previous_error = error;
  black_heading_pid.previous_tick = now;
  black_heading_pid.valid = 1U;

  /* Derivative braking may reduce the command, but must never reverse the
   * requested turn close to the target. */
  if ((output > 0.0f) != (error > 0.0f)) output = 0.0f;
  magnitude = Robot_AbsFloat(output);
  if (magnitude < BLACK_HEADING_PID_MIN_PERCENT)
    magnitude = BLACK_HEADING_PID_MIN_PERCENT;
  if (magnitude > BLACK_HEADING_PID_MAX_PERCENT)
    magnitude = BLACK_HEADING_PID_MAX_PERCENT;
  return error >= 0.0f ? (int16_t)magnitude : (int16_t)-magnitude;
}

static void BlackAlignStable_Reset(void)
{
  black_align_input_ready = 0U;
  black_align_stable_samples = 0U;
  black_align_last_frame = imu_navigation.angle_frames;
  black_align_reject_count = imu_navigation.rejected_steps;
  black_align_last_heading_rad = imu_navigation.heading_rad;
}

static uint8_t BlackAlignStable_Ready(uint32_t now)
{
  float change;
  if (!ImuNavigation_Fresh(&imu_navigation, now))
  {
    Motor_Stop();
    black_align_input_ready = 0U;
    black_align_stable_samples = 0U;
    black_align_last_frame = imu_navigation.angle_frames;
    black_align_reject_count = imu_navigation.rejected_steps;
    greedy_action = "WAIT_IMU";
    return 0U;
  }
  if (black_align_input_ready != 0U) return 1U;

  Motor_Stop();
  greedy_action = "ZERO_SETTLE";
  if ((uint32_t)(now - auto_state_start_tick) < BLACK_ALIGN_PRESETTLE_MS)
    return 0U;
  if (imu_navigation.rejected_steps != black_align_reject_count)
  {
    black_align_reject_count = imu_navigation.rejected_steps;
    black_align_stable_samples = 0U;
    black_align_last_frame = imu_navigation.angle_frames;
    black_align_last_heading_rad = imu_navigation.heading_rad;
    greedy_action = "ZERO_REJECT_WAIT";
    return 0U;
  }
  if (imu_navigation.angle_frames == black_align_last_frame) return 0U;

  change = Planner_NormalizeAngle(imu_navigation.heading_rad -
                                  black_align_last_heading_rad);
  black_align_last_frame = imu_navigation.angle_frames;
  black_align_last_heading_rad = imu_navigation.heading_rad;
  if (Robot_AbsFloat(change) > BLACK_ALIGN_STABILITY_RAD)
    black_align_stable_samples = 1U;
  else if (black_align_stable_samples < BLACK_ALIGN_STABLE_FRAMES)
    black_align_stable_samples++;

  if (black_align_stable_samples < BLACK_ALIGN_STABLE_FRAMES)
    return 0U;
  black_align_input_ready = 1U;
  BlackHeadingPid_Reset(now);
  greedy_action = "ALIGN_ZERO";
  Motor_SendText("IMU QUIET 5/5: ALIGN INITIAL 0 DEG PID START\r\n");
  return 1U;
}

static uint8_t BlackUnload_AlignWithKp(float target_heading, uint32_t now,
                                       float kp)
{
  float error;
  float magnitude;
  int16_t command;

  if (!ImuNavigation_Fresh(&imu_navigation, now))
  {
    Motor_Stop();
    black_heading_settle_tick = 0U;
    BlackAlignStable_Reset();
    BlackHeadingPid_Reset(now);
    greedy_action = "WAIT_IMU";
    return 0U;
  }
  error = Planner_NormalizeAngle(target_heading -
                                 imu_navigation.heading_rad);
  magnitude = Robot_AbsFloat(error);
  if (magnitude <= BLACK_HEADING_TOLERANCE_RAD)
  {
    Motor_Stop();
    BlackHeadingPid_Reset(now);
    if (black_heading_settle_tick == 0U)
      black_heading_settle_tick = now;
    return (uint8_t)((uint32_t)(now - black_heading_settle_tick) >=
                     BLACK_HEADING_SETTLE_MS);
  }
  black_heading_settle_tick = 0U;
  /* Keep the IMU PID active all the way to the tolerance band.  The old
   * minimum-PWM pulse/coast window could leave the heavy chassis stationary;
   * a continuous command lets the higher-Kp loop cross that dead zone and
   * finish the normal 90-degree turn in one closed-loop action. */
  command = BlackHeadingPid_CommandWithKp(error, now, kp);
  greedy_action = error > 0.0f ? "IMU_CCW" : "IMU_CW";
  Motor_SetTarget((int16_t)-command, command);
  return 0U;
}

static uint8_t BlackUnload_Align(float target_heading, uint32_t now)
{
  return BlackUnload_AlignWithKp(target_heading, now, BLACK_HEADING_PID_KP);
}

static void BlackUnload_StartStraight(int16_t speed,
                                      float target_heading,
                                      uint32_t now)
{
  StraightHold_Reset();
  straight_hold_active = 1U;
  straight_hold_target_valid = 1U;
  /* Return/unload travel must stay in the immutable startup coordinate
   * system.  Local refreshes are for ordinary collection travel only. */
  straight_hold_use_start_reference = 1U;
  straight_hold_base_percent = speed;
  straight_hold_target_heading = target_heading;
  straight_hold_integral = 0.0f;
  straight_hold_tick = now;
  Motor_SetTarget(speed, speed);
}

static void BlackBump_Reset(uint8_t reset_attempts)
{
  black_bump_phase = BLACK_BUMP_IDLE;
  black_bump_phase_tick = 0U;
  black_bump_suspect_tick = 0U;
  black_bump_suspect_active = 0U;
  black_bump_left_mm = 0.0f;
  black_bump_right_mm = 0.0f;
  if (reset_attempts != 0U) black_bump_attempts = 0U;
}

static void BlackBump_RecordTravel(float left_mm, float right_mm)
{
  if (black_bump_phase == BLACK_BUMP_BACKUP)
  {
    if (left_mm < 0.0f) black_bump_left_mm -= left_mm;
    if (right_mm < 0.0f) black_bump_right_mm -= right_mm;
  }
  else if (black_bump_phase == BLACK_BUMP_ESCAPE)
  {
    if (left_mm > 0.0f) black_bump_left_mm += left_mm;
    if (right_mm > 0.0f) black_bump_right_mm += right_mm;
  }
}

/* Return 1 while a contact recovery owns the wheel command.  In this phase
 * either commanded-forward wheel remaining nearly stopped means that the car
 * is pinned or a drive feedback channel has failed.  Both cases must release
 * the wall first; persistence, rather than an IMU impact, rejects glitches. */
static uint8_t BlackBump_Update(uint32_t now)
{
  if (black_bump_phase == BLACK_BUMP_STOP)
  {
    Motor_Stop();
    greedy_action = "BUMP_STOP";
    if ((uint32_t)(now - black_bump_phase_tick) >= BLACK_BUMP_STOP_MS)
    {
      black_bump_phase = BLACK_BUMP_BACKUP;
      black_bump_phase_tick = now;
      black_bump_left_mm = black_bump_right_mm = 0.0f;
      BlackUnload_StartStraight(-BLACK_BUMP_BACKUP_PERCENT,
                                imu_navigation.heading_rad, now);
      greedy_action = "BUMP_BACKUP";
    }
    return 1U;
  }

  if (black_bump_phase == BLACK_BUMP_BACKUP)
  {
    uint8_t distance_done = (uint8_t)(
        black_bump_left_mm >= BLACK_BUMP_BACKUP_DISTANCE_MM &&
        black_bump_right_mm >= BLACK_BUMP_BACKUP_DISTANCE_MM);
    if (distance_done != 0U ||
        (uint32_t)(now - black_bump_phase_tick) >=
            BLACK_BUMP_BACKUP_TIMEOUT_MS)
    {
      Motor_Stop();
      black_search_heading_rad = Planner_NormalizeAngle(
          imu_navigation.heading_rad - BLACK_BUMP_RIGHT_RAD);
      black_bump_phase = BLACK_BUMP_TURN_RIGHT;
      black_bump_phase_tick = now;
      black_heading_settle_tick = 0U;
      BlackHeadingPid_Reset(now);
      greedy_action = "BUMP_RIGHT";
      return 1U;
    }
    if (straight_hold_active == 0U ||
        straight_hold_base_percent != -BLACK_BUMP_BACKUP_PERCENT)
      BlackUnload_StartStraight(-BLACK_BUMP_BACKUP_PERCENT,
                                imu_navigation.heading_rad, now);
    else
      Motor_SetTarget(-BLACK_BUMP_BACKUP_PERCENT,
                      -BLACK_BUMP_BACKUP_PERCENT);
    greedy_action = "BUMP_BACKUP";
    return 1U;
  }

  if (black_bump_phase == BLACK_BUMP_TURN_RIGHT)
  {
    if (BlackUnload_Align(black_search_heading_rad, now) != 0U)
    {
      black_bump_phase = BLACK_BUMP_ESCAPE;
      black_bump_phase_tick = now;
      black_bump_left_mm = black_bump_right_mm = 0.0f;
      BlackUnload_StartStraight(BLACK_RETURN_FORWARD_PERCENT,
                                black_search_heading_rad, now);
      greedy_action = "BUMP_ESCAPE";
    }
    return 1U;
  }

  if (black_bump_phase == BLACK_BUMP_ESCAPE)
  {
    uint8_t distance_done = (uint8_t)(
        black_bump_left_mm >= BLACK_BUMP_ESCAPE_DISTANCE_MM &&
        black_bump_right_mm >= BLACK_BUMP_ESCAPE_DISTANCE_MM);
    if (distance_done != 0U ||
        (uint32_t)(now - black_bump_phase_tick) >=
            BLACK_BUMP_ESCAPE_TIMEOUT_MS)
    {
      black_bump_phase = BLACK_BUMP_IDLE;
      black_bump_phase_tick = now;
      black_bump_suspect_active = 0U;
      greedy_action = "FORWARD_FIND_BLACK";
      Motor_SendText("LEFT WALL RECOVERY COMPLETE: SLOW BLACK SEARCH RESUMED\r\n");
      return 1U;
    }
    if (straight_hold_active == 0U ||
        straight_hold_base_percent != BLACK_RETURN_FORWARD_PERCENT)
      BlackUnload_StartStraight(BLACK_RETURN_FORWARD_PERCENT,
                                black_search_heading_rad, now);
    else
      Motor_SetTarget(BLACK_RETURN_FORWARD_PERCENT,
                      BLACK_RETURN_FORWARD_PERCENT);
    greedy_action = "BUMP_ESCAPE";
    return 1U;
  }

  if ((uint32_t)(now - auto_state_start_tick) < BLACK_BUMP_ARM_MS ||
      left_target_percent <= 0 || right_target_percent <= 0)
  {
    black_bump_suspect_active = 0U;
    return 0U;
  }

  {
    float left_target_rpm = ((float)left_target_percent * MOTOR_MAX_RPM) /
                            100.0f;
    float right_target_rpm = ((float)right_target_percent * MOTOR_MAX_RPM) /
                             100.0f;
    float left_limit = left_target_rpm * BLACK_BUMP_MAX_TARGET_RATIO;
    float right_limit = right_target_rpm * BLACK_BUMP_MAX_TARGET_RATIO;
    uint8_t drive_stalled;
    if (left_limit < BLACK_BUMP_MAX_STALL_RPM)
      left_limit = BLACK_BUMP_MAX_STALL_RPM;
    if (right_limit < BLACK_BUMP_MAX_STALL_RPM)
      right_limit = BLACK_BUMP_MAX_STALL_RPM;
    drive_stalled = (uint8_t)(left_speed_rpm <= left_limit ||
                              right_speed_rpm <= right_limit);

    if (drive_stalled == 0U)
    {
      black_bump_suspect_active = 0U;
      return 0U;
    }
  }

  if (black_bump_suspect_active == 0U)
  {
    black_bump_suspect_active = 1U;
    black_bump_suspect_tick = now;
    return 0U;
  }
  if ((uint32_t)(now - black_bump_suspect_tick) < BLACK_BUMP_CONFIRM_MS)
    return 0U;

  black_bump_suspect_active = 0U;
  if (black_bump_attempts >= BLACK_BUMP_MAX_ATTEMPTS)
  {
    Motor_Stop();
    Auto_Stop(0U);
    Motor_SendText("LEFT WALL RECOVERY LIMIT 5: TASK STOPPED; CHECK WHEEL ENCODERS/MECHANICS\r\n");
    return 1U;
  }

  black_bump_attempts++;
  black_bump_phase = BLACK_BUMP_STOP;
  black_bump_phase_tick = now;
  black_bump_left_mm = black_bump_right_mm = 0.0f;
  Motor_Stop();
  greedy_action = "BUMP_STOP";
  Motor_SendText("WALL-LINE DRIVE STALL: STOP, BACKUP 40MM, RIGHT 10DEG\r\n");
  return 1U;
}

static void BlackUnload_Update(uint32_t now)
{
  uint8_t fresh = (uint8_t)(latest_black_tick != 0U &&
      (uint32_t)(now - latest_black_tick) <= BLACK_VISION_STALE_MS);

  if (auto_state == AUTO_UNLOAD_WAIT)
  {
    /* Compatibility landing state only.  The v7.3 unload sequence never
     * scans for black here: it immediately returns to the initial heading. */
    Motor_Stop();
    black_return_initial_heading_rad = 0.0f;
    black_return_turn_phase = 0U;
    black_return_heading_valid = 1U;
    black_heading_settle_tick = 0U;
    BlackAlignStable_Reset();
    BlackHeadingPid_Reset(now);
    auto_state = AUTO_UNLOAD_ALIGN_ZERO;
    auto_state_start_tick = now;
    greedy_action = "ZERO_SETTLE";
    Motor_SendText("UNLOAD START: STOP/IMU QUIET 5 FRAMES, THEN ALIGN INITIAL 0 DEG\r\n");
    return;
  }

  if (auto_state == AUTO_UNLOAD_ALIGN_ZERO)
  {
    if (black_return_heading_valid == 0U)
    {
      Motor_Stop();
      Auto_Stop(0U);
      Motor_SendText("BLACK RETURN HAS NO START YAW REFERENCE: TASK STOPPED\r\n");
      return;
    }
    if (BlackAlignStable_Ready(now) == 0U)
    {
      if ((uint32_t)(now - auto_state_start_tick) >=
          BLACK_RETURN_ALIGN_TIMEOUT_MS)
      {
        Motor_Stop();
        Auto_Stop(0U);
        Motor_SendText("BLACK RETURN ALIGN 0 IMU SETTLE TIMEOUT: TASK STOPPED\r\n");
      }
      return;
    }
    if (BlackUnload_Align(black_return_initial_heading_rad, now))
    {
      Motor_Stop();
      /* Empty object scan path: first turn CCW 90 degrees from the saved
       * startup heading.  Only after that turn is complete do we reverse to
       * the rear wall; a second CCW90 follows the 180 mm sonar stop. */
      black_return_turn_phase = 1U;
      auto_state = AUTO_UNLOAD_TURN_CCW_90;
      auto_state_start_tick = now;
      black_heading_settle_tick = 0U;
      BlackHeadingPid_Reset(now);
      greedy_action = "TURN_CCW90_FIRST";
      Motor_SendText("IMU 0 DEG LOCKED: IMU PID CCW 90 FIRST, THEN REVERSE TO 180MM\r\n");
      return;
    }
    if ((uint32_t)(now - auto_state_start_tick) >=
        BLACK_RETURN_ALIGN_TIMEOUT_MS)
    {
      Motor_Stop();
      Auto_Stop(0U);
      Motor_SendText("BLACK RETURN ALIGN 0 TIMEOUT/IMU STALE: TASK STOPPED\r\n");
    }
    return;
  }

  if (auto_state == AUTO_UNLOAD_REVERSE_WALL)
  {
    if ((uint32_t)(now - auto_state_start_tick) >=
        BLACK_RETURN_REVERSE_TIMEOUT_MS)
    {
      Motor_Stop();
      Auto_Stop(0U);
      Motor_SendText("BLACK RETURN REVERSE TIMEOUT: CHECK REAR SONAR/IMU\r\n");
      return;
    }
    if (!ImuNavigation_Fresh(&imu_navigation, now))
    {
      Motor_Stop();
      greedy_action = "WAIT_IMU";
      return;
    }
    if (hc_last_sample_tick != black_rear_consumed_tick)
    {
      black_rear_consumed_tick = hc_last_sample_tick;
      if (hc_distance_mm != 0U &&
          (uint32_t)(now - hc_last_sample_tick) <= (HC_SAMPLE_MS * 3U) &&
          hc_distance_mm <= BLACK_RETURN_REAR_STOP_MM)
      {
        if (black_rear_close_packets < BLACK_RETURN_REAR_CONFIRM_PACKETS)
          black_rear_close_packets++;
      }
      else black_rear_close_packets = 0U;
    }
    if (black_rear_close_packets >= BLACK_RETURN_REAR_CONFIRM_PACKETS)
    {
      Motor_Stop();
      auto_state = AUTO_UNLOAD_TURN_CCW_90;
      black_return_turn_phase = 2U;
      auto_state_start_tick = now;
      black_heading_settle_tick = 0U;
      BlackHeadingPid_Reset(now);
      greedy_action = "TURN_CCW90_SECOND";
      Motor_SendText("REAR WALL 180MM CONFIRMED: IMU PID CCW 90 SECOND START\r\n");
      return;
    }
    if (straight_hold_active == 0U ||
        straight_hold_base_percent != -BLACK_RETURN_REVERSE_PERCENT ||
        straight_hold_target_valid == 0U)
      BlackUnload_StartStraight(-BLACK_RETURN_REVERSE_PERCENT,
                                black_return_initial_heading_rad, now);
    else
      Motor_SetTarget(-BLACK_RETURN_REVERSE_PERCENT,
                      -BLACK_RETURN_REVERSE_PERCENT);
    greedy_action = "REVERSE_WALL";
    return;
  }

  if (auto_state == AUTO_UNLOAD_TURN_CCW_90)
  {
    uint8_t first_turn = (uint8_t)(black_return_turn_phase == 1U);
    float ccw_target = Planner_NormalizeAngle(
        black_return_initial_heading_rad +
        (first_turn != 0U ? BLACK_RETURN_CCW_90_RAD :
                            (2.0f * BLACK_RETURN_CCW_90_RAD)));
    /* Use the IMU PID for the complete 90-degree move.  Do not accept an
     * early 12-degree band: the improved sensor can now drive all the way to
     * the normal heading tolerance without the old low-torque fine pulses. */
    if (BlackUnload_AlignWithKp(ccw_target, now, BLACK_TURN_PID_KP))
    {
      Motor_Stop();
      if (first_turn != 0U)
      {
        /* The first 90-degree turn is a completed sub-step.  Reverse while
         * holding the new heading, then enter this same state for turn 2. */
        auto_state = AUTO_UNLOAD_REVERSE_WALL;
        auto_state_start_tick = now;
        black_rear_consumed_tick = hc_last_sample_tick;
        black_rear_close_packets = 0U;
        BlackUnload_StartStraight(-BLACK_RETURN_REVERSE_PERCENT,
                                  ccw_target, now);
        greedy_action = "REVERSE_WALL_AFTER_CCW90";
        Motor_SendText("IMU PID CCW 90 FIRST LOCKED: REVERSE TO REAR WALL 180MM\r\n");
      }
      else
      {
        /* The second 90-degree turn completes the requested two-turn return
         * maneuver.  Search for the black unloading area from this heading. */
        auto_state = AUTO_UNLOAD_SEARCH_FORWARD;
        auto_state_start_tick = now;
        black_search_heading_rad = ccw_target;
        black_start_packets = 0U;
        black_start_consumed_tick = latest_black_tick;
        BlackBump_Reset(1U);
        BlackUnload_StartStraight(BLACK_RETURN_FORWARD_PERCENT,
                                  black_search_heading_rad, now);
        greedy_action = "FORWARD_FIND_BLACK";
        Motor_SendText("IMU PID CCW 90 SECOND LOCKED: SLOW FORWARD SEARCH BLACK; LEFT-WALL RECOVERY ARMED\r\n");
      }
      return;
    }
    if ((uint32_t)(now - auto_state_start_tick) >=
        BLACK_RETURN_ALIGN_TIMEOUT_MS)
    {
      Motor_Stop();
      Auto_Stop(0U);
      Motor_SendText("BLACK RETURN CCW90 TIMEOUT/IMU STALE: TASK STOPPED\r\n");
    }
    return;
  }

  if (auto_state == AUTO_UNLOAD_SEARCH_FORWARD)
  {
    if ((uint32_t)(now - auto_state_start_tick) >=
        BLACK_RETURN_FORWARD_TIMEOUT_MS)
    {
      Motor_Stop();
      Auto_Stop(0U);
      Motor_SendText("FORWARD BLACK SEARCH TIMEOUT: TASK STOPPED\r\n");
      return;
    }
    if (!ImuNavigation_Fresh(&imu_navigation, now) || fresh == 0U)
    {
      Motor_Stop();
      black_start_packets = 0U;
      black_start_consumed_tick = latest_black_tick;
      black_bump_suspect_active = 0U;
      greedy_action = !ImuNavigation_Fresh(&imu_navigation, now) ?
          "WAIT_IMU" : "WAIT_BLACK_LINK";
      return;
    }
    if (latest_black_tick != black_start_consumed_tick)
    {
      black_start_consumed_tick = latest_black_tick;
      if (latest_black_zone.seen != 0U &&
          latest_black_zone.coverage_x10 >= BLACK_VISION_START_X10)
      {
        if (black_start_packets < BLACK_VISION_CONFIRM_PACKETS)
          black_start_packets++;
      }
      else black_start_packets = 0U;
    }
    if (black_start_packets >= BLACK_VISION_CONFIRM_PACKETS)
    {
      Motor_Stop();
      BlackBump_Reset(0U);
      black_arrive_packets = 0U;
      black_arrive_consumed_tick = latest_black_tick;
      auto_state = AUTO_UNLOAD_APPROACH;
      auto_state_start_tick = now;
      greedy_action = "BLACK_FOLLOW";
      Motor_SendText("BLACK >=3% CONFIRMED ON WALL LINE: SLOW VISUAL-GUIDED FORWARD START\r\n");
      return;
    }
    if (BlackBump_Update(now) != 0U) return;
    /* Blue-wall packets are diagnostic only in v8.2.  Collision recovery is
     * based exclusively on the two wheel encoders during Greedy travel. */
    if (straight_hold_active == 0U ||
        straight_hold_base_percent != BLACK_RETURN_FORWARD_PERCENT ||
        straight_hold_target_valid == 0U)
      BlackUnload_StartStraight(BLACK_RETURN_FORWARD_PERCENT,
                                black_search_heading_rad, now);
    else
      Motor_SetTarget(BLACK_RETURN_FORWARD_PERCENT,
                      BLACK_RETURN_FORWARD_PERCENT);
    greedy_action = "FORWARD_FIND_BLACK";
    return;
  }

  if (auto_state == AUTO_UNLOAD_APPROACH)
  {
    int16_t turn;
    if ((uint32_t)(now - auto_state_start_tick) >=
        BLACK_VISION_APPROACH_TIMEOUT_MS)
    {
      Motor_Stop();
      Auto_Stop(0U);
      Motor_SendText("BLACK APPROACH TIMEOUT: TASK STOPPED\r\n");
      return;
    }
    if (fresh == 0U || latest_black_zone.seen == 0U)
    {
      Motor_Stop();
      black_arrive_packets = 0U;
      black_arrive_consumed_tick = latest_black_tick;
      greedy_action = "BLACK_LOST";
      return;
    }
    if (latest_black_tick != black_arrive_consumed_tick)
    {
      black_arrive_consumed_tick = latest_black_tick;
      if (latest_black_zone.coverage_x10 >= BLACK_VISION_ARRIVE_X10)
      {
        if (black_arrive_packets < BLACK_VISION_CONFIRM_PACKETS)
          black_arrive_packets++;
      }
      else black_arrive_packets = 0U;
    }
    if (black_arrive_packets >= BLACK_VISION_CONFIRM_PACKETS)
    {
      Motor_Stop();
      unload_pre_turn_left_mm = unload_pre_turn_right_mm = 0.0f;
      unload_turn_start_heading_rad =
          ImuNavigation_Fresh(&imu_navigation, now) ?
          imu_navigation.heading_rad : black_search_heading_rad;
      auto_state = AUTO_UNLOAD_BACKUP_BEFORE_ALIGN;
      auto_state_start_tick = now;
      greedy_action = "BACKUP_BEFORE_ALIGN";
      Motor_SendText("BLACK >=40% CONFIRMED: BACKUP 80MM, ALIGN INITIAL, THEN CCW30\r\n");
      return;
    }

    turn = (int16_t)((int32_t)latest_black_zone.center_x_milli *
                     BLACK_VISION_STEER_MAX / 1000);
    if (latest_black_zone.center_x_milli > -BLACK_VISION_CENTER_DEADBAND &&
        latest_black_zone.center_x_milli < BLACK_VISION_CENTER_DEADBAND)
      turn = 0;
    Motor_SetTarget((int16_t)(BLACK_VISION_APPROACH_PERCENT + turn),
                    (int16_t)(BLACK_VISION_APPROACH_PERCENT - turn));
    greedy_action = turn == 0 ? "BLACK_FORWARD" :
                    (turn > 0 ? "BLACK_RIGHT" : "BLACK_LEFT");
    return;
  }

  if (auto_state == AUTO_UNLOAD_BACKUP_BEFORE_ALIGN)
  {
    uint8_t distance_done = (uint8_t)(
        unload_pre_turn_left_mm >= BLACK_PRE_TURN_BACKUP_DISTANCE_MM &&
        unload_pre_turn_right_mm >= BLACK_PRE_TURN_BACKUP_DISTANCE_MM);
    if (distance_done != 0U && ImuNavigation_Fresh(&imu_navigation, now))
    {
      Motor_Stop();
      unload_turn_target_heading_rad = black_return_initial_heading_rad;
      unload_turn_angle_rad = 0.0f;
      black_heading_settle_tick = 0U;
      BlackHeadingPid_Reset(now);
      auto_state = AUTO_UNLOAD_ALIGN_INITIAL;
      auto_state_start_tick = now;
      greedy_action = "ALIGN_INITIAL";
      Motor_SendText("PRE-TURN BACKUP 80MM COMPLETE: TWO-WHEEL IMU ALIGN INITIAL START\r\n");
      return;
    }
    if ((uint32_t)(now - auto_state_start_tick) >=
        BLACK_PRE_TURN_BACKUP_TIMEOUT_MS)
    {
      Motor_Stop();
      Auto_Stop(0U);
      Motor_SendText("PRE-TURN BACKUP TIMEOUT: CHECK WHEEL ENCODERS\r\n");
      return;
    }
    if (distance_done != 0U)
    {
      Motor_Stop();
      greedy_action = "WAIT_IMU";
      return;
    }
    /* Back straight along the heading held when the 40% black threshold was
     * confirmed.  Initial-heading alignment starts only after 80 mm. */
    if (straight_hold_active == 0U ||
        straight_hold_base_percent != -BLACK_PRE_TURN_BACKUP_PERCENT ||
        straight_hold_target_valid == 0U)
      BlackUnload_StartStraight(-BLACK_PRE_TURN_BACKUP_PERCENT,
                                unload_turn_start_heading_rad, now);
    else
      Motor_SetTarget(-BLACK_PRE_TURN_BACKUP_PERCENT,
                      -BLACK_PRE_TURN_BACKUP_PERCENT);
    greedy_action = "BACKUP_BEFORE_ALIGN";
    return;
  }

  if (auto_state == AUTO_UNLOAD_ALIGN_INITIAL)
  {
    if (BlackUnload_AlignWithKp(black_return_initial_heading_rad, now,
                                BLACK_TURN_PID_KP))
    {
      Motor_Stop();
      unload_turn_start_heading_rad = imu_navigation.heading_rad;
      unload_turn_target_heading_rad = Planner_NormalizeAngle(
          black_return_initial_heading_rad + BLACK_UNLOAD_CCW_30_RAD);
      unload_turn_angle_rad = 0.0f;
      black_heading_settle_tick = 0U;
      BlackHeadingPid_Reset(now);
      auto_state = AUTO_UNLOAD_TURN_CCW_30;
      auto_state_start_tick = now;
      greedy_action = "TURN_CCW30";
      Motor_SendText("IMU INITIAL HEADING LOCKED: TWO-WHEEL CCW30 START\r\n");
      return;
    }
    if ((uint32_t)(now - auto_state_start_tick) >=
        BLACK_RETURN_ALIGN_TIMEOUT_MS)
    {
      Motor_Stop();
      Auto_Stop(0U);
      Motor_SendText("UNLOAD ALIGN INITIAL TIMEOUT: CHECK IMU/MECHANICS\r\n");
    }
    return;
  }

  if (auto_state == AUTO_UNLOAD_TURN_CCW_30)
  {
    if (ImuNavigation_Fresh(&imu_navigation, now))
      unload_turn_angle_rad = Planner_NormalizeAngle(
          imu_navigation.heading_rad - unload_turn_start_heading_rad);
    if (BlackUnload_AlignWithKp(unload_turn_target_heading_rad, now,
                                BLACK_TURN_PID_KP))
    {
      char report[96];
      unload_reverse_left_mm = unload_reverse_right_mm = 0.0f;
      unload_sonar_close_packets = 0U;
      auto_state = AUTO_UNLOAD_REVERSE;
      auto_state_start_tick = now;
      greedy_action = "REVERSE_IN";
      (void)snprintf(report, sizeof(report),
          "IMU TWO-WHEEL CCW TURN VERIFIED=%ld mdeg (30 TARGET): REVERSE INTO BLACK ZONE\r\n",
          (long)(Robot_AbsFloat(unload_turn_angle_rad) *
                 180000.0f / ROBOT_PI));
      Motor_SendText(report);
      return;
    }
    if ((uint32_t)(now - auto_state_start_tick) >= BLACK_TURN_TIMEOUT_MS)
    {
      Motor_Stop();
      Auto_Stop(0U);
      Motor_SendText("UNLOAD TWO-WHEEL CCW30 IMU PID TIMEOUT: CHECK IMU/MECHANICS\r\n");
    }
    return;
  }

  if (auto_state == AUTO_UNLOAD_REVERSE)
  {
    uint8_t distance_done = (uint8_t)(
        unload_reverse_left_mm >= BLACK_REVERSE_DISTANCE_MM &&
        unload_reverse_right_mm >= BLACK_REVERSE_DISTANCE_MM);
    uint8_t fallback_done = (uint8_t)(
        (uint32_t)(now - auto_state_start_tick) >=
        BLACK_REVERSE_FALLBACK_MS);
    if (hc_distance_mm != 0U &&
        (uint32_t)(now - hc_last_sample_tick) <= (HC_SAMPLE_MS * 3U) &&
        hc_distance_mm <= BLACK_REAR_SONAR_STOP_MM)
    {
      if (unload_sonar_close_packets < 2U) unload_sonar_close_packets++;
    }
    else unload_sonar_close_packets = 0U;

    if (distance_done != 0U || unload_sonar_close_packets >= 2U ||
        fallback_done != 0U)
    {
      Motor_Stop();
      MissionExtension_ForceStartDockedEject(now);
      auto_state = AUTO_COMPLETE;
      auto_state_start_tick = now;
      greedy_action = "EJECT";
      if (distance_done != 0U)
        Motor_SendText("UNLOAD REVERSE 300MM COMPLETE: HATCH/ SHAKE START\r\n");
      else if (unload_sonar_close_packets >= 2U)
        Motor_SendText("UNLOAD REAR SONAR 120MM: HATCH/ SHAKE START\r\n");
      else
        Motor_SendText("UNLOAD REVERSE 10S FALLBACK: HATCH/ SHAKE START\r\n");
      return;
    }
    Motor_SetTarget(-BLACK_REVERSE_PERCENT, -BLACK_REVERSE_PERCENT);
  }

}

static void Auto_Update(void)
{
  uint32_t now = HAL_GetTick();

  /* The match clock owns the 240/300-second transitions regardless of whether
   * Greedy is running, manually paused, or waiting for camera data. */
  if (CombatMission_Service(now) != 0U) return;

  if (auto_state == AUTO_DEBUG)
  {
    /* Stationary IMU test also works while the brush is in HOLD. */
    Debug_Update(now);
    return;
  }
  if (collector_recovery_state != COLLECTOR_IDLE) return;
  /* Startup yaw capture owns the stopped wheels and does not depend on an
   * OpenMV heartbeat.  Camera gating begins only after zero is locked. */
  if (greedy_imu_zero_pending != 0U)
  {
    GreedyImuZero_Update(now);
    return;
  }
  /* A local refresh is a short stationary gate.  It does not depend on the
   * camera heartbeat and cannot modify the saved startup reference. */
  if (greedy_imu_local_pending != 0U)
  {
    GreedyImuLocal_Update(now);
    return;
  }
  /* Run before the 20ms scheduling gate so a stale link cannot keep driving. */
  if (auto_state == AUTO_COLLISION_RECOVER)
  {
    CollisionRecovery_Update(now);
    return;
  }
  if (auto_state == AUTO_POST_UNLOAD_PACE_FORWARD ||
      auto_state == AUTO_POST_UNLOAD_PACE_REVERSE)
  {
    PostUnloadPace_Update(now);
    return;
  }
  if (Auto_CheckVision(now) == 0U) return;

  if ((now - last_auto_update_tick) < AUTO_UPDATE_PERIOD_MS)
  {
    return;
  }
  last_auto_update_tick = now;

  if (auto_state == AUTO_UNLOAD_WAIT ||
      auto_state == AUTO_UNLOAD_ALIGN_ZERO ||
      auto_state == AUTO_UNLOAD_REVERSE_WALL ||
      auto_state == AUTO_UNLOAD_TURN_CCW_90 ||
      auto_state == AUTO_UNLOAD_SEARCH_FORWARD ||
      auto_state == AUTO_UNLOAD_APPROACH ||
      auto_state == AUTO_UNLOAD_BACKUP_BEFORE_ALIGN ||
      auto_state == AUTO_UNLOAD_ALIGN_INITIAL ||
      auto_state == AUTO_UNLOAD_TURN_CCW_30 ||
      auto_state == AUTO_UNLOAD_REVERSE)
  {
    BlackUnload_Update(now);
    return;
  }

  if (greedy_active != 0U && auto_state == AUTO_COMBAT_PATROL)
  {
    CombatGreedy_UpdatePatrol(now);
    return;
  }

  if (greedy_active && auto_state != AUTO_COLLECT &&
      auto_state != AUTO_UNLOAD_WAIT &&
      auto_state != AUTO_UNLOAD_ALIGN_ZERO &&
      auto_state != AUTO_UNLOAD_REVERSE_WALL &&
      auto_state != AUTO_UNLOAD_TURN_CCW_90 &&
      auto_state != AUTO_UNLOAD_SEARCH_FORWARD &&
      auto_state != AUTO_UNLOAD_BACKUP_BEFORE_ALIGN &&
      auto_state != AUTO_COLLISION_RECOVER &&
      auto_state != AUTO_COMPLETE && auto_state != AUTO_IDLE)
  {
    Greedy_UpdateLocal(now);
    return;
  }

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
    case AUTO_UNLOAD_WAIT:
      break;

    case AUTO_SCAN:
      if (greedy_active)
      {
        Greedy_UpdateScan(now);
        break;
      }
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
      if (greedy_active &&
          (greedy_collect_left_mm < greedy_collect_goal_mm ||
           greedy_collect_right_mm < greedy_collect_goal_mm))
      {
        if ((uint32_t)(now - auto_state_start_tick) >= GREEDY_COLLECT_TIMEOUT_MS)
        {
          /* Elapsed time is a failure limit, never evidence of pickup. */
          Motor_Stop();
          BrushFeedback_SetEnabled(0U, now);
          Auto_Stop(0U);
          Motor_SendText("GREEDY PICKUP TIMEOUT: ACTION FAILED; TASK STOPPED, CHECK WHEEL ENCODERS\r\n");
        }
        else Motor_SetTarget(GREEDY_FORWARD_SPEED_PERCENT, GREEDY_FORWARD_SPEED_PERCENT);
        break;
      }
      Motor_SetTarget(AUTO_FINAL_SPEED_PERCENT, AUTO_FINAL_SPEED_PERCENT);
      if (greedy_active || (now - auto_state_start_tick) >= AUTO_COLLECT_DURATION_MS)
      {
        Motor_Stop();
        if (greedy_active)
        {
          MissionExtension_RecordCollected();
          if (CombatStrategy_IsActive() != 0U)
            CombatStrategy_RecordCollected(now);
          GreedyLocal_Reset(now, 0U);
          greedy_action = "DONE";
          Motor_SendText("GREEDY PICKUP ACTION COMPLETE: RESCAN\r\n");
        }
        else
        {
          TargetMap_MarkCollected(&target_map, current_target_id, now);
          MissionExtension_RecordCollected();
          CombatStrategy_RecordCollected(now);
        }
        current_target_id = 0U;
        plan_dirty = 1U;
        auto_state = AUTO_PLAN;
        auto_state_start_tick = now;
        Vision_SendMode('S', 0U);
        if (greedy_active)
        {
          /* Require a new observation after feeding; never re-use the last
           * close frame to trigger the same pickup repeatedly. */
          Auto_StartScan(AUTO_SCAN_TIMEOUT_MS);
          GreedyLocal_Reset(now, GREEDY_LOCAL_AFTER_PICK_MS);
          GreedyImuLocal_Start(now, 1U);
        }
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

    case AUTO_UNLOAD_ALIGN_ZERO:
    case AUTO_UNLOAD_REVERSE_WALL:
    case AUTO_UNLOAD_TURN_CCW_90:
    case AUTO_UNLOAD_SEARCH_FORWARD:
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

static uint8_t Auto_AvailableCount(void)
{
  return greedy_active ? GreedyLocal_Visible(HAL_GetTick()) :
                         TargetMap_CountAvailable(&target_map);
}

static void Greedy_SendStatus(uint8_t detailed)
{
  uint32_t now = HAL_GetTick();
  const GreedyLocalTarget_t *track = GreedyLocal_Get();
  uint8_t count = GreedyLocal_Visible(now);
  uint8_t black_fresh = (uint8_t)(latest_black_tick != 0U &&
      (uint32_t)(now - latest_black_tick) <= BLACK_VISION_STALE_MS);
  const char *scan_mode = greedy_imu_zero_pending != 0U ? "IMU_ZERO" :
      (auto_state == AUTO_UNLOAD_WAIT ? "BLACK_RETURN" :
      (auto_state != AUTO_SCAN ? "OFF" :
       (greedy_scan_turning ? "TURN" : "OBSERVE")));
  const char *navigation =
      greedy_imu_zero_pending != 0U ? "IMU_ZERO_WAIT" :
      ((auto_state == AUTO_UNLOAD_ALIGN_ZERO ||
       auto_state == AUTO_UNLOAD_TURN_CCW_90 ||
       auto_state == AUTO_UNLOAD_ALIGN_INITIAL ||
       auto_state == AUTO_UNLOAD_TURN_CCW_30) ? "IMU_TURN_PID" :
      (straight_hold_active == 0U ? "ENC_TURN" :
       (straight_hold_target_valid != 0U ? "IMU_STRAIGHT" :
                                           "ENC_STRAIGHT")));
  char line[256];
  (void)snprintf(line, sizeof(line),
      "GREEDY LIST=%u CURRENT=%u STATE=%s SCAN=%s ACTION=%s OBJ_STEP=%u/6 NAV=%s PAYLOAD=%u BLACK_SEEN=%u BLACK_COV_X10=%u BLACK_AGE=%ld BUMP=%u/%u\r\n",
      (unsigned)count, (unsigned)current_target_id, Auto_StateName(auto_state),
      scan_mode, greedy_action,
      (unsigned)greedy_scan_steps, navigation,
      (unsigned)MissionExtension_HasPayload(),
      (unsigned)(black_fresh != 0U ? latest_black_zone.seen : 0U),
      (unsigned)(black_fresh != 0U ? latest_black_zone.coverage_x10 : 0U),
      latest_black_tick != 0U ? (long)(now - latest_black_tick) : -1L,
      (unsigned)black_bump_phase, (unsigned)black_bump_attempts);
  Motor_SendText(line);
  if (greedy_imu_zero_pending != 0U)
  {
    (void)snprintf(line, sizeof(line),
        "IMU START ZERO SAMPLES=%u/%u ELAPSED=%lu ms; WHEELS=STOP\r\n",
        (unsigned)greedy_imu_zero_samples,
        (unsigned)GREEDY_IMU_ZERO_REQUIRED_FRAMES,
        (unsigned long)(now - greedy_imu_zero_start_tick));
    Motor_SendText(line);
  }
  if (auto_state == AUTO_COLLECT)
  {
    (void)snprintf(line, sizeof(line),
        "GREEDY PICKUP L=%ld R=%ld GOAL=%lu mm ELAPSED=%lu ASSUMED=1\r\n",
        (long)greedy_collect_left_mm, (long)greedy_collect_right_mm,
        (unsigned long)greedy_collect_goal_mm, (unsigned long)(now - auto_state_start_tick));
    Motor_SendText(line);
  }
  if (!detailed) return;
  if (track->locked)
  {
    (void)snprintf(line, sizeof(line),
        "LOCAL LOCK=1 VISIBLE=%u C=%u X=%d Y=%u AGE=%lu; NO GLOBAL LIST\r\n",
        (unsigned)count, (unsigned)track->target.color, (int)track->target.lateral_mm,
        (unsigned)track->target.forward_mm, (unsigned long)(now - track->last_seen_ms));
    Motor_SendText(line);
  }
}

static void Auto_SendMap(void)
{
  if (greedy_active) { Greedy_SendStatus(1U); return; }
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
  if (greedy_active) { Greedy_SendStatus(1U); return; }
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
  BrushFeedback_SetDrive(speed != 0U ? 1 : 0, HAL_GetTick());
}

static void motor3_stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  BrushFeedback_SetDrive(0, HAL_GetTick());
}

static void motor3_reverse(uint16_t speed)
{
  /* Called only after zero PWM and the direction-change pause. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, speed);
  BrushFeedback_SetDrive(speed != 0U ? -1 : 0, HAL_GetTick());
}

static const char *CollectorRecovery_StateName(void)
{
  switch (collector_recovery_state)
  {
    case COLLECTOR_STOPPING:  return "STOPPING";
    case COLLECTOR_REVERSING: return "REVERSING";
    case COLLECTOR_SETTLING:  return "SETTLING";
    case COLLECTOR_ADVANCING: return "ADVANCING";
    case COLLECTOR_HOLD:      return "HOLD";
    default:                 return "IDLE";
  }
}

static void CollectorRecovery_SendStatus(void)
{
  char line[192];
  const char *brush = (collector_recovery_state == COLLECTOR_IDLE ||
      collector_recovery_state == COLLECTOR_ADVANCING) ? "FORWARD" :
      (collector_recovery_state == COLLECTOR_REVERSING ? "REVERSE" : "STOP");
  (void)snprintf(line, sizeof(line),
      "UNJAM=%s BRUSH=%s BACK_L=%ld BACK_R=%ld mm BACK_STOP=%u REV_MS=%lu EST_TURNS=%u\r\n",
      CollectorRecovery_StateName(), brush, (long)collector_left_travel_mm,
      (long)collector_right_travel_mm, (unsigned int)collector_backup_done,
      (unsigned long)COLLECTOR_REVERSE_DURATION_MS,
      (unsigned int)COLLECTOR_REVERSE_TURNS);
  Motor_SendText(line);
  (void)snprintf(line, sizeof(line), "ADVANCE_L=%ld ADVANCE_R=%ld GOAL=50 mm\r\n",
      (long)collector_advance_left_mm, (long)collector_advance_right_mm);
  Motor_SendText(line);
  (void)snprintf(line, sizeof(line), "REV_CONTROL=%s REV_COUNT=%lu AUTO_CYCLE=%u ATTEMPTS=%u\r\n",
      collector_use_encoder ? "ENCODER" : "TIMED",
      (unsigned long)collector_reverse_progress, (unsigned int)collector_auto_cycle,
      (unsigned int)collector_auto_attempts);
  Motor_SendText(line);
}

static void CollectorFeedback_SendStatus(void)
{
  char line[272];
  BrushFeedbackSnapshot_t feedback = BrushFeedback_Snapshot(HAL_GetTick());
  (void)snprintf(line, sizeof(line),
      "BRUSH_FB COUNT=%ld CPR=%lu RPM_X10=%ld ENCERR=%lu ADC=%u MV=%u MA=%ld ADC_OK=%u AUTO=%u LAST_EVENT=%s WATCH=%s DELTA=%ld VERIFIED=%u SIGN=%d\r\n",
      (long)feedback.count, (unsigned long)feedback.counts_per_rev,
      (long)feedback.rpm_x10, (unsigned long)feedback.errors,
      (unsigned int)feedback.adc_raw, (unsigned int)feedback.adc_mv,
      (long)feedback.current_ma, (unsigned int)feedback.adc_fresh,
      (unsigned int)feedback.enabled, BrushFeedback_EventName(feedback.last_event),
      feedback.counts_per_rev >= 4U ? "RPM" : "PULSE",
      (long)feedback.delta_counts, (unsigned int)feedback.forward_verified, (int)BRUSH_ENCODER_SIGN);
  Motor_SendText(line);
  (void)snprintf(line, sizeof(line),
      "ENC_HEALTH=%s ERR_DELTA=%lu ERR_WIN=%lu ERR_STREAK=%lu WINDOW_MS=%lu LIMIT=%lu STREAK_LIMIT=%lu\r\n",
      feedback.encoder_fault ? "FAULT" : (feedback.error_window ? "WARN" : "OK"),
      (unsigned long)feedback.error_delta, (unsigned long)feedback.error_window,
      (unsigned long)feedback.error_streak, (unsigned long)BRUSH_ERROR_WINDOW_MS,
      (unsigned long)BRUSH_ERROR_WINDOW_LIMIT, (unsigned long)BRUSH_ERROR_CONSECUTIVE_LIMIT);
  Motor_SendText(line);
}

static void CollectorFeedback_Update(void)
{
  uint32_t now = HAL_GetTick();
  BrushEvent_t event = BrushFeedback_Update(now);
  BrushFeedbackSnapshot_t feedback = BrushFeedback_Snapshot(now);
  if (feedback.error_window && !feedback.encoder_fault &&
      (uint32_t)(now - brush_error_warn_tick) >= BRUSH_ERROR_WARN_MS)
  {
    char warning[112];
    brush_error_warn_tick = now;
    (void)snprintf(warning, sizeof(warning), "BRUSH ENCODER WARN: ERR_WIN=%lu TOTAL=%lu; MONITORING\r\n",
        (unsigned long)feedback.error_window, (unsigned long)feedback.errors);
    Motor_SendText(warning);
  }
  if (collector_recovery_state == COLLECTOR_HOLD &&
      (uint32_t)(now - collector_hold_report_tick) >= BRUSH_HOLD_REPORT_MS)
  {
    char report[144];
    collector_hold_report_tick = now;
    (void)snprintf(report, sizeof(report),
        "JAM_HOLD ALIVE MS=%lu LAST_EVENT=%s; V=STATUS, CHECK FAULT THEN K=BRUSH_RESTORE\r\n",
        (unsigned long)now, BrushFeedback_EventName(feedback.last_event));
    Motor_SendText(report);
  }
  uint8_t healthy = feedback.enabled && collector_recovery_state == COLLECTOR_IDLE &&
      feedback.drive > 0 && feedback.forward_motion &&
      event == BRUSH_EVENT_NONE;
#if BRUSH_CURRENT_ENABLE
  healthy = healthy && feedback.adc_fresh && feedback.current_ma < (int32_t)BRUSH_CURRENT_STALL_MA;
#endif
  if (healthy)
  {
    if (!collector_healthy_tracking) { collector_healthy_tracking = 1U; collector_healthy_tick = now; }
    if ((uint32_t)(now - collector_healthy_tick) >= BRUSH_HEALTHY_RESET_MS)
      collector_auto_attempts = 0U;
  }
  else collector_healthy_tracking = 0U;
  if (event == BRUSH_EVENT_NONE) return;
  if (event == BRUSH_EVENT_ENCODER_ERRORS)
  {
    char report[128];
    (void)snprintf(report, sizeof(report), "BRUSH ENCODER FAULT: DELTA=%lu WINDOW=%lu STREAK=%lu TOTAL=%lu\r\n",
        (unsigned long)feedback.error_delta, (unsigned long)feedback.error_window,
        (unsigned long)feedback.error_streak, (unsigned long)feedback.errors);
    Motor_SendText(report);
  }
  Motor_SendText("BRUSH EVENT: ");
  Motor_SendText(BrushFeedback_EventName(event));
  Motor_SendText("\r\n");
  if ((event == BRUSH_EVENT_LOW_RPM || event == BRUSH_EVENT_NO_PULSES ||
       event == BRUSH_EVENT_HIGH_CURRENT) &&
      collector_recovery_state == COLLECTOR_IDLE)
  {
    CollectorRecovery_Start();
    if (collector_recovery_state == COLLECTOR_STOPPING)
    {
      /* Automatic jam recovery has no retry budget. Saturate this telemetry
       * counter instead of wrapping after 255 successful retry starts. */
      if (collector_auto_attempts < UINT8_MAX) collector_auto_attempts++;
      collector_auto_cycle = 1U;
      return;
    }
  }
  CollectorRecovery_Abort();
  if (event == BRUSH_EVENT_WRONG_DIRECTION)
    Motor_SendText("BRUSH DIRECTION FAULT: CHECK PHYSICAL FORWARD AND BRUSH_ENCODER_SIGN; V=STATUS\r\n");
  Motor_SendText("AUTO_UNJAM HOLD: CHECK WIRING/JAM FIRST; K THEN Q=NEW TASK; H,N,T,K,E=CALIBRATE\r\n");
}

static void CollectorRecovery_Start(void)
{
  MissionUnloadState_t unload_state = MissionExtension_GetUnloadState();
  if (collector_recovery_state != COLLECTOR_IDLE &&
      collector_recovery_state != COLLECTOR_HOLD)
  {
    Motor_SendText("UNJAM BUSY: REQUEST NOT RESTARTED\r\n");
    return;
  }
  if (servo_test_active != 0U ||
      (auto_state == AUTO_COMPLETE && MissionExtension_HasPayload() != 0U) ||
      (unload_state != MISSION_UNLOAD_IDLE && unload_state != MISSION_UNLOAD_DONE))
  {
    Motor_SendText("UNJAM REJECTED: STOP UNLOADING/SERVO TEST FIRST\r\n");
    return;
  }
  collector_resume_state = auto_state;
  collector_use_encoder = BrushFeedback_Cpr() >= 4U;
  collector_auto_cycle = 0U;
  collector_reverse_progress = 0U;
  Motor_Stop();
  /* A jam invalidates an in-progress quiet sampling window.  The successful
   * recovery path starts a fresh local reference after all motion ends. */
  greedy_imu_local_pending = 0U;
  greedy_imu_local_samples = 0U;
  motor3_stop();
  MissionExtension_CancelMotion();
  current_target_id = 0U;
  memset(&planned_route, 0, sizeof(planned_route));
  auto_state = AUTO_UNJAM;
  collector_left_travel_mm = collector_right_travel_mm = 0.0f;
  collector_backup_done = 0U;
  collector_advance_pending = 1U;
  collector_advance_left_mm = collector_advance_right_mm = 0.0f;
  collector_recovery_state = COLLECTOR_STOPPING;
  collector_phase_tick = HAL_GetTick();
  Motor_SendText(collector_use_encoder ? "UNJAM START: ENCODER TWO TURNS, LIMITED BACKUP\r\n" :
                                        "UNJAM START: UNCALIBRATED TIMED REVERSE, LIMITED BACKUP\r\n");
}

static void CollectorRecovery_RecordTravel(float left_mm, float right_mm)
{
  if (collector_recovery_state == COLLECTOR_ADVANCING)
  {
    /* Signed wheel travel: reversed/missing feedback is not forward progress. */
    collector_advance_left_mm += left_mm;
    collector_advance_right_mm += right_mm;
  }
  if (collector_recovery_state == COLLECTOR_REVERSING && collector_backup_done == 0U)
  {
    /* Stop when EITHER wheel reaches the cap, including an encoder polarity
     * mismatch or a one-wheel stall. Wheel travel is not ground displacement. */
    collector_left_travel_mm += Robot_AbsFloat(left_mm);
    collector_right_travel_mm += Robot_AbsFloat(right_mm);
  }
}

static void CollectorRecovery_Abort(void)
{
  Motor_Stop();
  motor3_stop();
  BrushFeedback_SetEnabled(0U, HAL_GetTick());
  servo_test_active = 0U;
  Servo_Stop();
  collector_auto_cycle = collector_healthy_tracking = 0U;
  Auto_Stop(0U);
  combat_autonomy_enabled = 0U;
  collector_resume_state = AUTO_IDLE;
  collector_backup_done = 1U;
  collector_advance_pending = 0U;
  collector_recovery_state = COLLECTOR_HOLD;
  collector_phase_tick = HAL_GetTick();
  Motor_SendText("UNJAM ABORTED: WHEELS/BRUSH STOPPED; J=RETRY, K=RESTORE BRUSH\r\n");
  collector_hold_report_tick = collector_phase_tick;
}

static void CollectorRecovery_Restore(void)
{
  if (collector_recovery_state != COLLECTOR_HOLD)
  {
    Motor_SendText("K REQUIRES UNJAM HOLD\r\n");
    return;
  }
  Motor_Stop();
  motor3_stop();
  collector_resume_state = AUTO_IDLE;
  collector_recovery_state = COLLECTOR_SETTLING;
  collector_phase_tick = HAL_GetTick();
  Motor_SendText("UNJAM RESTORE: WAIT BEFORE FORWARD\r\n");
  collector_advance_pending = 0U; /* K restores only the brush, never drives the car. */
}

static void CollectorRecovery_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - collector_phase_tick;
  uint8_t index;
  switch (collector_recovery_state)
  {
    case COLLECTOR_STOPPING:
      if (elapsed < COLLECTOR_DIRECTION_PAUSE_MS) return;
      collector_recovery_state = COLLECTOR_REVERSING;
      collector_phase_tick = now;
      collector_reverse_start_count = BrushFeedback_Count();
      collector_reverse_start_errors = BrushFeedback_Errors();
      collector_reverse_progress = 0U;
      collector_reverse_progress_tick = now;
      motor3_reverse(COLLECTOR_REVERSE_PWM);
      Motor_SetTarget(-COLLECTOR_BACKUP_PERCENT, -COLLECTOR_BACKUP_PERCENT);
      Motor_SendText("UNJAM REVERSING\r\n");
      return;

    case COLLECTOR_REVERSING:
    {
      uint8_t finished = 0U;
      if (collector_use_encoder)
      {
        int32_t reverse_count = (int32_t)(collector_reverse_start_count - BrushFeedback_Count());
        if (BrushFeedback_Errors() != collector_reverse_start_errors)
        {
          CollectorRecovery_Abort();
          Motor_SendText("UNJAM FAILED: INVALID ENCODER TRANSITIONS\r\n");
          return;
        }
        if (reverse_count > 0 && (uint32_t)reverse_count > collector_reverse_progress)
        {
          collector_reverse_progress = (uint32_t)reverse_count;
          collector_reverse_progress_tick = now;
        }
        finished = reverse_count >= (int32_t)(BrushFeedback_Cpr() * COLLECTOR_REVERSE_TURNS);
        if (!finished && (elapsed >= BRUSH_REVERSE_MAX_MS ||
            (uint32_t)(now - collector_reverse_progress_tick) >= BRUSH_REVERSE_NO_PROGRESS_MS))
        {
          CollectorRecovery_Abort();
          Motor_SendText("UNJAM FAILED: REVERSE ENCODER TIMEOUT; BRUSH HELD OFF\r\n");
          return;
        }
      }
      else finished = elapsed >= COLLECTOR_REVERSE_DURATION_MS;
      if (finished)
      {
        Motor_Stop();
        motor3_stop();
        collector_backup_done = 1U;
        collector_recovery_state = COLLECTOR_SETTLING;
        collector_phase_tick = now;
        Motor_SendText("UNJAM SETTLING\r\n");
        return;
      }
      if (collector_backup_done == 0U)
      {
        uint8_t distance_limit = collector_left_travel_mm >= COLLECTOR_BACKUP_DISTANCE_MM ||
                                collector_right_travel_mm >= COLLECTOR_BACKUP_DISTANCE_MM;
        if (distance_limit != 0U || elapsed >= COLLECTOR_BACKUP_TIMEOUT_MS)
        {
          Motor_Stop();
          collector_backup_done = 1U;
          Motor_SendText(distance_limit != 0U ? "UNJAM BACKUP STOP: WHEEL DISTANCE\r\n" :
                                               "UNJAM BACKUP STOP: TIME LIMIT\r\n");
        }
        else
          Motor_SetTarget(-COLLECTOR_BACKUP_PERCENT, -COLLECTOR_BACKUP_PERCENT);
      }
      return;
    }

    case COLLECTOR_SETTLING:
      if (elapsed < COLLECTOR_DIRECTION_PAUSE_MS) return;
      Motor_Stop();
      if (collector_advance_pending)
      {
        collector_recovery_state = COLLECTOR_ADVANCING;
        collector_phase_tick = now;
        collector_advance_left_mm = collector_advance_right_mm = 0.0f;
        /* Keep wheels stopped until the guarded ADVANCING update. */
        motor3_forward(COLLECTOR_RUN_PWM);
        Motor_SendText("UNJAM ADVANCE: GOAL=50 mm; SIGNED WHEEL DISTANCE\r\n");
        return;
      }
      motor3_forward(COLLECTOR_RUN_PWM);
      collector_recovery_state = COLLECTOR_IDLE;
      /* Reverse motion may have displaced targets. Preserve odometry and
       * collected inventory, but require a NEW frame before autonomous motion. */
      for (index = 0U; index < TARGET_MAP_MAX_TARGETS; index++)
        if (target_map.targets[index].collected == 0U) target_map.targets[index].valid = 0U;
      if (greedy_active) Greedy_ClearPending();
      memset(&latest_vision_frame, 0, sizeof(latest_vision_frame));
      latest_vision_tick = 0U;
      accepted_vision_frame_count = 0U;
      vision_motion_hold = 0U;
      vision_report_due = 1U;
      plan_dirty = 1U;
      if (collector_resume_state == AUTO_DEBUG)
      {
        auto_state = AUTO_DEBUG;
        debug_action = "WAIT_FRAME";
        debug_target_index = -1;
        Vision_SendMode('S', 0U);
      }
      else if (collector_resume_state == AUTO_POST_UNLOAD_PACE_FORWARD ||
               collector_resume_state == AUTO_POST_UNLOAD_PACE_REVERSE)
      {
        int16_t pace_speed =
            collector_resume_state == AUTO_POST_UNLOAD_PACE_FORWARD ?
            POST_UNLOAD_PACE_PERCENT : -POST_UNLOAD_PACE_PERCENT;
        auto_state = collector_resume_state;
        post_unload_pace_left_mm = 0.0f;
        post_unload_pace_right_mm = 0.0f;
        post_unload_pace_leg_tick = now;
        BlackUnload_StartStraight(pace_speed,
                                  post_unload_pace_heading_rad, now);
        Motor_SendText("UNJAM DONE: RESUME P 300MM PACING; BRUSH FORWARD\r\n");
      }
      else if (collector_resume_state != AUTO_IDLE && collector_resume_state != AUTO_COMPLETE)
      {
        Auto_StartScan(CombatStrategy_IsActive() != 0U ? 3000U : AUTO_SCAN_TIMEOUT_MS);
        if (greedy_active != 0U)
          GreedyImuLocal_Start(now, 1U);
      }
      else
        auto_state = collector_resume_state;
      Motor_SendText("UNJAM CYCLE DONE: BRUSH FORWARD; JAM CLEARANCE NOT SENSED\r\n");
      return;

    case COLLECTOR_ADVANCING:
      if (collector_advance_left_mm < -5.0f || collector_advance_right_mm < -5.0f ||
          Robot_AbsFloat(collector_advance_left_mm - collector_advance_right_mm) > COLLECTOR_ADVANCE_SKEW_MM)
      {
        CollectorRecovery_Abort();
        Motor_SendText("UNJAM ADVANCE FAILED: WHEEL SIGN/SKEW; CHECK ENCODERS\r\n");
        return;
      }
      if (collector_advance_left_mm >= COLLECTOR_ADVANCE_DISTANCE_MM &&
          collector_advance_right_mm >= COLLECTOR_ADVANCE_DISTANCE_MM)
      {
        Motor_Stop();
        collector_advance_pending = 0U;
        collector_recovery_state = COLLECTOR_SETTLING;
        collector_phase_tick = now - COLLECTOR_DIRECTION_PAUSE_MS;
        Motor_SendText("UNJAM ADVANCE DONE: 50 mm WHEEL TRAVEL\r\n");
        return;
      }
      if (elapsed >= COLLECTOR_ADVANCE_TIMEOUT_MS)
      {
        CollectorRecovery_Abort();
        Motor_SendText("UNJAM ADVANCE FAILED: TIMEOUT\r\n");
        return;
      }
      Motor_SetTarget(collector_advance_left_mm >= COLLECTOR_ADVANCE_DISTANCE_MM ? 0 : COLLECTOR_ADVANCE_PERCENT,
                      collector_advance_right_mm >= COLLECTOR_ADVANCE_DISTANCE_MM ? 0 : COLLECTOR_ADVANCE_PERCENT);
      return;

    case COLLECTOR_HOLD:
    case COLLECTOR_IDLE:
    default:
      return;
  }
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
  else Servo_Update(now_ms);
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

  if (servo_test_active)
  {
    Motor_Stop();
    Mission_ServoUpdate(MISSION_UNLOAD_IDLE, now);
    return;
  }
  if (collector_recovery_state != COLLECTOR_IDLE) return;

  /* BlackUnload_Update owns wheel motion until it explicitly starts EJECT. */
  if (auto_state == AUTO_UNLOAD_WAIT ||
      auto_state == AUTO_UNLOAD_ALIGN_ZERO ||
      auto_state == AUTO_UNLOAD_REVERSE_WALL ||
      auto_state == AUTO_UNLOAD_TURN_CCW_90 ||
      auto_state == AUTO_UNLOAD_SEARCH_FORWARD ||
      auto_state == AUTO_UNLOAD_APPROACH ||
      auto_state == AUTO_UNLOAD_BACKUP_BEFORE_ALIGN ||
      auto_state == AUTO_UNLOAD_ALIGN_INITIAL ||
      auto_state == AUTO_UNLOAD_TURN_CCW_30 ||
      auto_state == AUTO_UNLOAD_REVERSE)
  {
    Mission_ServoUpdate(MISSION_UNLOAD_IDLE, now);
    return;
  }

  /* Debug has its own stop-only proximity check and no map-wall avoidance. */
  if (auto_state == AUTO_DEBUG) return;
  /* Greedy owns its local following and scan state machine. */
  if (greedy_active && auto_state != AUTO_COMPLETE) return;
  /* Do not let the reactive avoidance state overwrite the vision hold. */
  if (vision_motion_hold != 0U)
  {
    Motor_Stop();
    return;
  }

  /* 舵机只在 EJECT 阶段持续运动，其余状态保持中位 */
  Mission_ServoUpdate(unload_state, now);

  if ((unload_state != MISSION_UNLOAD_IDLE) &&
      (unload_state != MISSION_UNLOAD_DONE))
  {
    MissionDriveOutput_t output =
        MissionExtension_UpdateUnload(&robot_pose, hc_distance_mm, now);
    if (unload_state == MISSION_UNLOAD_GO_STAGE)
    {
      int16_t safe_left;
      int16_t safe_right;
      uint8_t request_replan;
      if (MissionExtension_ApplyForwardSafety(&robot_pose, 0U, now,
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
        if (combat_final_return_started != 0U)
        {
          motor3_stop();
          BrushFeedback_SetEnabled(0U, now);
          if (combat_p_unload_committed != 0U &&
              CombatStrategy_MatchFinished(now) == 0U)
          {
            PostUnloadPace_Start(now);
          }
          else
          {
            greedy_active = 0U;
            combat_autonomy_enabled = 0U;
            auto_state = AUTO_COMPLETE;
            Motor_SendText("COMBAT FINAL DEPOSIT COMPLETE: WAIT FOR 300S FINISH\r\n");
          }
        }
        else if (CombatStrategy_MatchFinished(now) == 0U)
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

  /* Rear HC-SR04 is never a forward obstacle input. Position-boundary safety
   * remains available here, but no ultrasonic front obstacle is claimed. */
  if ((auto_state == AUTO_NAVIGATE) ||
      (auto_state == AUTO_COMBAT_PATROL) ||
      ((auto_state == AUTO_IDLE) && (last_drive_command == 'F')))
  {
    int16_t safe_left;
    int16_t safe_right;
    uint8_t request_replan;
    if (MissionExtension_ApplyForwardSafety(&robot_pose, 0U, now,
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

static inline uint32_t HCSR04_Cycles(void)
{
  return DWT->CYCCNT; /* Subtract raw cycles BEFORE dividing across wrap. */
}

/* 测一次距离,返回毫米;超时/无回波返回 0 */
static uint32_t HCSR04_Measure(void)
{
  uint32_t start;
  uint32_t t0;
  uint32_t timeout;

  /* 发 10us 触发脉冲 */
  HAL_GPIO_WritePin(HC_TRIG_PORT, HC_TRIG_PIN, GPIO_PIN_SET);
  t0 = HCSR04_Cycles();
  while ((uint32_t)(HCSR04_Cycles() - t0) < 720U) { }
  HAL_GPIO_WritePin(HC_TRIG_PORT, HC_TRIG_PIN, GPIO_PIN_RESET);

  /* 等 ECHO 变高(超时 20ms) */
  timeout = HCSR04_Cycles();
  while (HAL_GPIO_ReadPin(HC_ECHO_PORT, HC_ECHO_PIN) == GPIO_PIN_RESET)
  {
    if ((uint32_t)(HCSR04_Cycles() - timeout) >= 20000U * 72U) return 0U;
  }
  start = HCSR04_Cycles();

  /* 等 ECHO 变低(超时 30ms) */
  timeout = HCSR04_Cycles();
  while (HAL_GPIO_ReadPin(HC_ECHO_PORT, HC_ECHO_PIN) != GPIO_PIN_RESET)
  {
    if ((uint32_t)(HCSR04_Cycles() - timeout) >= 30000U * 72U) return 0U;
  }

  /* 距离(mm) = 回波时间(us) × 343m/s ÷ 2 */
  return (((uint32_t)(HCSR04_Cycles() - start) / 72U) * 343U) / 2000U;
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
  JY901_Init();
  ImuNavigation_Reset(&imu_navigation);
  MX_USART1_UART_Init();
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
  if (HAL_UART_Receive_IT(&huart1, &imu_rx_byte, 1U) != HAL_OK)
  {
    imu_rx_retry = 1U;
    imu_rearm_errors++;
  }

  Motor_Stop();
  Buzzer_Init();
  Vision_SendMode('S', 0U);
  Motor_SendText("READY FW=v8.4.3+D_ALIGN_FIX+P_NO_IMU_RESET+UNLIMITED_UNJAM\r\n");
  Motor_SendText("MATCH CLOCK=FIRST Q; P=IMU_PRESERVED+EMPTY_UNLOAD; D=UNLOAD TEST/START_REF; AUTO_UNJAM=UNLIMITED\r\n");

  //对抗赛上电前刷待机；Q/P 启动正转，J 解卡及 HOLD 可反转/停刷//
  CollectorDirection_Init();
  BrushFeedback_HardwareInit();
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
#if COMPETITION_COMBAT_BLUETOOTH == 1U
  motor3_stop();
#else
  motor3_forward(COLLECTOR_RUN_PWM);
#endif

  //后舱门 MG995 舵机初始化：TIM1_CH1(PA8)，50Hz，回到中位//
  Servo_Init();

  //超声初始化//
  HCSR04_Init();
#if COMPETITION_COMBAT_BLUETOOTH == 1U
  Competition_InitializeStandby();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Bluetooth_ProcessPending();
    if (imu_rx_retry)
    {
      uint32_t mask = __get_PRIMASK();
      __disable_irq();
      if (HAL_UART_Receive_IT(&huart1, &imu_rx_byte, 1U) == HAL_OK) imu_rx_retry = 0U;
      __set_PRIMASK(mask);
    }

    Vision_ProcessIncoming();

    /* IMU heading feeds straight-line hold and the black return 0/90 PID. */
    IMU_NavigationUpdate(HAL_GetTick());

    if (motor_control_due != 0U)
    {
      motor_control_due = 0U;
      Motor_ControlUpdate();
    }

    BrushFeedback_HardwarePoll();
    CollectorFeedback_Update();
    CollectorRecovery_Update();
    Auto_Update();
    Mission_UpdateIntegration();
    StraightHold_Service(HAL_GetTick());
    Vision_ReportTelemetry(0U);

    if ((CombatStrategy_IsActive() != 0U) &&
        combat_autonomy_enabled != 0U &&
        combat_final_return_started == 0U &&
        ((auto_state == AUTO_NAVIGATE) ||
         (auto_state == AUTO_FINAL_ALIGN) ||
         (auto_state == AUTO_COLLECT) ||
         (auto_state == AUTO_COMBAT_PATROL)))
    {
      uint8_t forward_commanded =
          (left_target_percent > 10) && (right_target_percent > 10);
      uint32_t now = HAL_GetTick();
      if (CombatStrategy_ElapsedMs(now) < COMBAT_ENDGAME_RETURN_MS &&
          CollisionMonitor_Update(forward_commanded, now) != 0U)
      {
        CollisionRecovery_Start(now);
      }
    }
    else CollisionMonitor_Reset(HAL_GetTick());

    if ((auto_state == AUTO_IDLE) && motor_running &&
        ((HAL_GetTick() - last_motion_command_tick) > MOTOR_COMMAND_TIMEOUT_MS))
    {
      Motor_Stop();
      Motor_SendText("COMMAND TIMEOUT - MOTOR STOPPED\r\n");
    }
    /* HC-SR04 每 100ms 测一次;正在倒车且连续两次小于阈值 → 停车 */
        if ((ultrasonic_monitor_active != 0U ||
             collector_recovery_state == COLLECTOR_IDLE ||
             collector_recovery_state == COLLECTOR_SETTLING ||
             collector_recovery_state == COLLECTOR_ADVANCING) &&
            ((HAL_GetTick() - hc_last_sample_tick) >= HC_SAMPLE_MS))
        {
          hc_last_sample_tick = HAL_GetTick();
          hc_distance_mm = HCSR04_Measure();


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

          if ((REAR_HCSR04_MANUAL_STOP_ENABLE != 0U) &&
              (last_drive_command == 'B') && (hc_close_count >= 2U))
          {
            Motor_Stop();
            hc_close_count = 0U;
            Motor_SendText("REVERSE STOP - UNLOAD POSITION\r\n");
          }
        }
    Ultrasonic_MonitorUpdate(HAL_GetTick());
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
  if (huart->Instance == USART1)
  {
    JY901_RxByte(imu_rx_byte, HAL_GetTick());
    if (HAL_UART_Receive_IT(&huart1, &imu_rx_byte, 1U) != HAL_OK)
    {
      imu_rearm_errors++;
      imu_rx_retry = 1U;
    }
    return;
  }
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

      Bluetooth_QueueFromISR(byte);

      // 重新启动接收中断
      (void)HAL_UART_Receive_IT(&huart3, &bluetooth_rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    imu_uart_errors++;
    JY901_ResetStream();
    (void)HAL_UART_AbortReceive(&huart1);
    if (HAL_UART_Receive_IT(&huart1, &imu_rx_byte, 1U) != HAL_OK)
    {
      imu_rearm_errors++;
      imu_rx_retry = 1U;
    }
    return;
  }
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
