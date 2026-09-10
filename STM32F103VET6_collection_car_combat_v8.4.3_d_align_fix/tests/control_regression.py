"""Compile actual main.c control functions against fake GPIO/timers.

Extraction keeps this diagnostic out of the firmware build and avoids copying
the logic under test. Real planner, map, mission and combat modules are linked.
This tests decisions/outputs, NOT ARM interrupts, physical speed or UART timing.
"""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'Core/Src/main.c').read_text()

def section(name):
    return source.split('/* USER CODE BEGIN ' + name + ' */', 1)[1].split(
        '/* USER CODE END ' + name + ' */', 1)[0]

def function(name, source=source):
    match = re.search(r'^(?:static )?[\w *]+\b' + name +
                      r'\([^;]*?\)\s*\{', source, re.M)
    if not match:
        raise AssertionError('Function not found: ' + name)
    brace = source.index('{', match.start())
    level = 1
    end = brace + 1
    while level:
        level += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]

names = ['IMU_CopySnapshot', 'IMU_FormatAngle', 'IMU_StartRelativeReference',
         'IMU_TryCaptureRelativeReference',
         'IMU_SendStatus', 'IMU_NavigationUpdate',
         'GreedyImuZero_Start', 'GreedyImuZero_Update',
         'GreedyImuLocal_Start', 'GreedyImuLocal_Update',
         'Competition_InitializeStandby',
         'CombatGreedy_StartOrResume', 'CombatGreedy_StartPatrol',
         'CombatGreedy_UpdatePatrol', 'Combat_StartFinalUnload',
         'CombatMission_Service',
         'HAL_UART_RxCpltCallback', 'HAL_UART_ErrorCallback',
         'Motor_ClampPercent', 'Motor_SetOne', 'Motor_Set', 'Motor_SetTarget',
         'Greedy_UpdateLocal', 'Greedy_StartPickup',
         'Greedy_SendStatus', 'Greedy_UpdateScan',
         'Auto_StateName', 'Auto_SendMap', 'Auto_SendPlan', 'Auto_AvailableCount',
         'Motor_ResetPid', 'Motor_Stop', 'Robot_AbsFloat',
         'StraightHold_Reset', 'StraightHold_Service',
         'Auto_StartScan', 'Auto_Start', 'Auto_Stop',
         'BlackHeadingPid_Reset', 'BlackHeadingPid_CommandWithKp',
         'BlackAlignStable_Reset', 'BlackAlignStable_Ready',
         'BlackUnload_AlignWithKp', 'BlackUnload_Align',
         'BlackUnload_StartStraight',
         'BlackBump_Reset', 'BlackBump_RecordTravel', 'BlackBump_Update',
         'BlackUnload_Update', 'Auto_CheckVision',
         'Auto_Update', 'Auto_BuildPlan', 'Auto_FindCurrentVisionTarget',
         'Auto_DriveTowardLocal', 'Mission_UpdateIntegration',
         'motor3_forward', 'Vision_ApplyFrame', 'Mission_ServoUpdate',
         'Motor_ProcessCommand', 'Motor_ApplyDriveCommand', 'Debug_Start',
         'Debug_Update',
         'Vision_ReportTelemetry', 'Robot_ModeName',
         'Ultrasonic_SendStatus', 'Ultrasonic_MonitorUpdate',
         'Bluetooth_QueueFromISR', 'Bluetooth_GetCommand', 'Bluetooth_ProcessPending',
         'Vision_ProcessIncoming', 'Vision_WallFresh',
         'CollisionMonitor_Reset', 'CollisionMonitor_RecordTravel',
         'CollisionMonitor_Update', 'CollisionRecovery_Start',
         'CollisionRecovery_Update', 'PostUnloadPace_Start',
         'PostUnloadPace_Update',
         'Robot_UpdateOdometry',
         'motor3_stop', 'motor3_reverse', 'CollectorRecovery_Start',
         'CollectorRecovery_Update', 'CollectorRecovery_Abort',
         'CollectorRecovery_Restore', 'CollectorRecovery_RecordTravel',
         'CollectorRecovery_StateName', 'CollectorRecovery_SendStatus',
         'CollectorFeedback_Update', 'CollectorFeedback_SendStatus', 'HCSR04_Measure']
functions = [function(name).replace('Planner_BuildRoute(', 'CountedPlanner(') for name in names]
servo_source = (ROOT / 'Core/Src/servo.c').read_text()
servo_globals = servo_source[servo_source.index('static TIM_HandleTypeDef'):servo_source.index('static void Servo_SetPulseUs')].replace('TIM_HandleTypeDef', 'FakeTimer')
functions += [function(name, servo_source) for name in
              ['Servo_SetPulseUs', 'Servo_AngleToPulseUs', 'Servo_UpdateHatch',
               'Servo_StartEject', 'Servo_Stop', 'Servo_Update', 'Servo_IsRunning']]
functions = [f.replace('void Servo_Update(uint32_t now_ms)\n{',
                       'void Servo_Update(uint32_t now_ms)\n{\n servo_updates++;') for f in functions]
prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "drive_pwm.h"
#include "path_planner.h"
#include "target_map.h"
#include "vision_protocol.h"
#include "mission_extension.h"
#include "combat_strategy.h"
#include "brush_feedback.h"
#include "greedy_collection.h"
#include "greedy_local.h"
#include "coverage_path.h"
#include "jy901.h"
#include "imu_navigation.h"
#include "servo.h"
#include "jy901.h"
/* The large behavioural harness keeps immediate open-loop fake outputs. A
 * dedicated source/HAL syntax build verifies the production PID=ON branch. */
#define MOTOR_CLOSED_LOOP_ENABLE 0U
static unsigned planner_calls;
static void CountedPlanner(const RobotPose_t *p, const MapTarget_t *c,
                           uint8_t n, PlannerRoute_t *r)
{ planner_calls++; Planner_BuildRoute(p, c, n, r); }
typedef int GPIO_TypeDef;
typedef struct { uint32_t arr, ccr[4]; } FakeTimer;
static FakeTimer htim3 = {3599, {0}}, htim8 = {999, {0}};
static GPIO_TypeDef pins_b, pins_c;
#define GPIOB (&pins_b)
#define GPIOC (&pins_c)
#define GPIO_PIN_0 0
#define GPIO_PIN_1 1
#define GPIO_PIN_2 2
#define GPIO_PIN_3 3
#define GPIO_PIN_5 5
#define GPIO_PIN_6 6
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->arr)
#define __HAL_TIM_SET_COMPARE(t,c,d) FakeCompare(t,c,d)
static unsigned brush_writes, brush_in1, brush_in2, servo_updates;
static void FakeCompare(FakeTimer *timer, unsigned channel, unsigned duty)
{
  timer->ccr[channel] = duty;
  if (timer == &htim8) brush_writes++;
}
static uint32_t clock_ms, mode_requests;
typedef struct { unsigned Instance; } UART_HandleTypeDef;
enum { USART1=1, USART3=3, UART4=4, HAL_OK=0 };
static UART_HandleTypeDef huart1={USART1}, huart3={USART3}, huart4={UART4};
static unsigned uart_rearm_port, uart_abort_port;
static int uart_rearm_result;
static int HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *b, unsigned n)
{ (void)b; assert(n==1); uart_rearm_port=h->Instance; return uart_rearm_result; }
static int HAL_UART_AbortReceive(UART_HandleTypeDef *h)
{ uart_abort_port=h->Instance; return HAL_OK; }
static uint8_t feedback_test_ab;
static unsigned test_irq_mask;
static unsigned __get_PRIMASK(void) { return test_irq_mask; }
static void __disable_irq(void) { test_irq_mask = 1; }
static void __set_PRIMASK(unsigned mask) { test_irq_mask = mask; }
static uint32_t HAL_GetTick(void) { return clock_ms; }
static uint32_t test_cycles, cycle_calls, echo_reads;
static unsigned echo_mode;
static uint32_t HCSR04_Cycles(void)
{ cycle_calls++; test_cycles += 72; return test_cycles; }
static int HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
  (void)port; (void)pin;
  echo_reads++;
  if (echo_mode == 0) return 0;
  if (echo_mode == 1) return 1;
  return echo_reads > 4 && echo_reads < 1005;
}
static void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, int value)
{
  /* Changing H-bridge direction must happen with PWM already at zero. */
  if (port == GPIOB && pin == GPIO_PIN_0 && brush_in1 != (unsigned)value)
    assert(htim8.ccr[2] == 0);
  if (port == GPIOB && pin == GPIO_PIN_1 && brush_in2 != (unsigned)value)
    assert(htim8.ccr[2] == 0);
  if (port == GPIOB && pin == GPIO_PIN_0) brush_in1 = value;
  if (port == GPIOB && pin == GPIO_PIN_1) brush_in2 = value;
}
static void Vision_SendMode(char mode, uint8_t color)
{ (void)mode; (void)color; mode_requests++; }
static char test_log[16384];
static void Motor_SendText(const char *text)
{
  size_t used = strlen(test_log);
  snprintf(test_log + used, sizeof(test_log)-used, "%s", text);
}
static void Motor_SendStatus(void) {}
'''
checks = r'''
static void fresh(void)
{
  VisionFrame_t frame = {0}; /* zero targets is still a valid heartbeat */
  frame.sequence = 1;
  Vision_ApplyFrame(&frame);
}
static void assert_brush_continues(void)
{
  assert(htim8.ccr[2] == COLLECTOR_RUN_PWM);
  assert(brush_in1 == 1 && brush_in2 == 0);
}
static void reset_test(void)
{
  clock_ms = 100;
  motor_speed_percent = MOTOR_DEFAULT_SPEED;
  JY901_Init();
  imu_uart_errors = imu_rearm_errors = 0;
  imu_rx_retry = 0;
  imu_report_tick = imu_zero_after_frame = 0;
  uart_rearm_result = 0;
  imu_zero_valid = 0;
  ImuNavigation_Reset(&imu_navigation);
  ImuNavigation_Reset(&imu_local_navigation);
  greedy_imu_zero_pending = greedy_imu_zero_to_unload = 0;
  greedy_imu_zero_samples = 0;
  greedy_imu_zero_start_tick = greedy_imu_zero_last_frame = 0;
  greedy_imu_zero_anchor_raw = 0;
  greedy_imu_local_pending = greedy_imu_local_resume_scan = 0;
  greedy_imu_local_samples = greedy_imu_local_reference_valid = 0;
  greedy_imu_local_start_tick = greedy_imu_local_last_frame = 0;
  greedy_imu_local_last_refresh_tick = 0;
  greedy_imu_local_anchor_raw = 0;
  BrushFeedback_Reset(clock_ms, 0);
  feedback_test_ab = 0;
  collector_reverse_start_count = collector_reverse_start_errors = 0;
  collector_reverse_progress = collector_reverse_progress_tick = 0;
  collector_use_encoder = collector_auto_cycle = collector_auto_attempts = 0;
  collector_healthy_tracking = 0;
  collector_hold_report_tick = clock_ms;
  collector_recovery_state = COLLECTOR_IDLE;
  collector_resume_state = AUTO_IDLE;
  collector_phase_tick = 0;
  collector_left_travel_mm = collector_right_travel_mm = 0;
  collector_backup_done = 0;
  servo_test_active = 0;
  htim8.ccr[2] = brush_in1 = brush_in2 = 0;
  CombatStrategy_Reset();
  Greedy_Reset(); greedy_active = 0;
  combat_autonomy_enabled = combat_final_return_started = 0;
  combat_finish_reported = 0;
  combat_p_empty_unload_mode = combat_p_unload_committed = 0;
  combat_p_unload_completed = 0;
  post_unload_pace_left_mm = post_unload_pace_right_mm = 0;
  post_unload_pace_heading_rad = 0;
  post_unload_pace_leg_tick = 0;
  combat_patrol_left_mm = combat_patrol_right_mm = 0;
  GreedyLocal_Reset(clock_ms, 0);
  StraightHold_Reset();
  planner_calls = 0;
  MissionExtension_Reset();
  TargetMap_Reset(&target_map);
  memset(&planned_route, 0, sizeof(planned_route));
  memset(&latest_vision_frame, 0, sizeof(latest_vision_frame));
  robot_pose = (RobotPose_t){1500, 1000, 0};
  auto_state = AUTO_IDLE;
  vision_motion_hold = 0;
  vision_wall_valid = vision_wall_raw = vision_wall_confirmed = 0;
  vision_wall_hit_packets = vision_wall_clear_packets = 0;
  vision_wall_tick = 0;
  collision_recovery_phase = COLLISION_RECOVER_STOP;
  collision_recovery_phase_tick = 0;
  CollisionMonitor_Reset(clock_ms);
  collision_backup_left_mm = collision_backup_right_mm = 0;
  collision_turn_angle_rad = 0;
  accepted_vision_frame_count = 0;
  latest_vision_tick = 0;
  last_auto_update_tick = 0;
  auto_state_start_tick = clock_ms;
  hc_distance_mm = 0;
  hc_last_sample_tick = 0;
  hc_close_count = 0;
  ultrasonic_monitor_active = 0;
  ultrasonic_report_tick = 0;
  black_live_report_tick = 0;
  black_heading_settle_tick = 0;
  black_rear_consumed_tick = 0;
  black_rear_close_packets = 0;
  black_return_initial_heading_rad = 0;
  black_return_heading_valid = 0;
  black_align_input_ready = black_align_stable_samples = 0;
  black_align_last_frame = black_align_reject_count = 0;
  black_align_last_heading_rad = 0;
  black_start_packets = 0;
  black_arrive_packets = 0;
  black_start_consumed_tick = 0;
  black_arrive_consumed_tick = 0;
  black_search_heading_rad = 0;
  unload_turn_angle_rad = 0.0f;
  unload_turn_start_heading_rad = 0.0f;
  unload_turn_target_heading_rad = 0.0f;
  unload_pre_turn_left_mm = unload_pre_turn_right_mm = 0;
  BlackBump_Reset(1);
  memset(&black_heading_pid, 0, sizeof(black_heading_pid));
  empty_plan_retry_count = 0;
  debug_action = "OFF";
  debug_target_index = -1;
  vision_report_tick = 0;
  vision_report_due = 1;
  memset(vision_filter_reason, 0, sizeof(vision_filter_reason));
  test_log[0] = '\0';
  bluetooth_read_index = bluetooth_write_index = bluetooth_stop_pending = 0;
  bluetooth_drop_count = 0;
  VisionProtocol_Init();
  motor3_forward(COLLECTOR_RUN_PWM); /* same command as firmware startup */
  brush_writes = 0;
  servo_updates = 0;
  Servo_Stop();
  Motor_Stop();
}
static void test_pwm(void)
{
  int i, last = 0;
  for (i = 1; i <= 100; ++i)
  {
    int duty = Motor_MapDrivePercent((int16_t)i);
    assert(duty >= 60 && duty <= 100 && duty >= last);
    assert(Motor_MapDrivePercent((int16_t)-i) == -duty);
    last = duty;
  }
  assert(Motor_MapDrivePercent(0) == 0);
  assert(Motor_MapDrivePercent(32767) == 100);
  assert(Motor_MapDrivePercent(-32768) == -100);
  assert(Motor_MapDrivePercent(40) == 76);
  assert(Motor_MapDrivePercent(18) < Motor_MapDrivePercent(35));
  Motor_SetTarget(1, -1);
  assert(left_pwm_percent == 60 && right_pwm_percent == -60);
  /* Supplied v6 applies an 80% boost on start; it uses ARR, not ARR+1.
   * These checks describe the received wheel code, not a boost-timing fix. */
  assert(htim3.ccr[0] == 2879 && htim3.ccr[1] == 2879);
  clock_ms += MOTOR_BOOST_DURATION_MS + 1;
  Motor_SetTarget(100, 0);
  assert(htim3.ccr[0] == 3599 && htim3.ccr[1] == 0);
  Motor_Stop();
  assert(htim3.ccr[0] == 0 && htim3.ccr[1] == 0);
  puts("PASS: logical PWM mapping and supplied v6 boost output smoke check");
}
static void test_loss_and_recovery(void)
{
  AutoState_t states[] = {AUTO_SCAN, AUTO_PLAN, AUTO_NAVIGATE,
    AUTO_FINAL_ALIGN, AUTO_COLLECT, AUTO_RECOVER, AUTO_COMBAT_PATROL};
  unsigned i;
  for (i = 0; i < sizeof(states)/sizeof(states[0]); i++)
  {
    uint16_t kept, stale;
    reset_test();
    kept = TargetMap_Upsert(&target_map, 1, 1000, 1000, 90, clock_ms);
    TargetMap_MarkCollected(&target_map, kept, clock_ms);
    stale = TargetMap_Upsert(&target_map, 2, 2000, 1000, 90, clock_ms);
    MissionExtension_RecordCollected();
    fresh();
    auto_state = states[i];
    current_target_id = stale;
    planned_route.count = 1;
    Motor_SetTarget(40, 40);
    clock_ms += AUTO_VISION_TIMEOUT_MS + 1;
    last_auto_update_tick = clock_ms; /* watchdog precedes scheduling gate */
    Auto_Update();
    assert(vision_motion_hold == 1);
    assert(left_pwm_percent == 0 && right_pwm_percent == 0);
    assert_brush_continues();
    assert(current_target_id == 0 && planned_route.count == 0);
    assert(TargetMap_FindById(&target_map, stale) == NULL);
    assert(TargetMap_FindById(&target_map, kept)->collected != 0);
    assert(MissionExtension_HasPayload() != 0);
    /* Even an avoidance state already in STOP/TURN must not overwrite hold. */
    Mission_UpdateIntegration();
    assert(left_pwm_percent == 0 && right_pwm_percent == 0);
    assert_brush_continues();
    clock_ms += 500;
    fresh();
    Auto_Update();
    assert(vision_motion_hold == 0 && auto_state == AUTO_SCAN);
    assert_brush_continues();
    assert(left_pwm_percent <= -60 && right_pwm_percent >= 60);
    assert(scan_accumulated_angle == 0);
  }
  reset_test();
  Auto_Start();
  assert_brush_continues();
  assert(left_pwm_percent == 0);
  Auto_Update();
  assert(vision_motion_hold == 1 && left_pwm_percent == 0);
  clock_ms += 20;
  fresh();
  Auto_Update();
  assert(auto_state == AUTO_SCAN && htim8.ccr[2] == COLLECTOR_RUN_PWM);
  /* Zero timestamp and uint32 tick wrap must be valid. */
  reset_test();
  clock_ms = 0; fresh(); auto_state = AUTO_SCAN;
  assert(Auto_CheckVision(0) == 1);
  latest_vision_tick = UINT32_MAX - 500;
  assert(Auto_CheckVision(100) == 1);
  assert(Auto_CheckVision(1000) == 0);
  assert_brush_continues();
  puts("PASS: vision loss/recovery stops wheels, preserves payload and continuous brush");
}
static void test_missing_target(void)
{
  reset_test();
  fresh();
  auto_state = AUTO_FINAL_ALIGN;
  Motor_SetTarget(20, 20);
  Auto_Update();
  assert(vision_motion_hold == 0); /* fresh empty frame, not a link failure */
  assert(left_pwm_percent == 0 && right_pwm_percent == 0);
  assert_brush_continues();
  puts("PASS: missing target stops wheels without changing brush");
}
static void test_deposit_next_batch(void)
{
  reset_test();
  CombatStrategy_Start(clock_ms);
  MissionExtension_RecordCollected();
  CombatStrategy_RecordCollected(clock_ms);
  auto_state = AUTO_COMPLETE;
  MissionExtension_StartUnload(clock_ms);
  robot_pose.x_mm = MISSION_UNLOAD_STAGE_X_MM;
  robot_pose.y_mm = MISSION_UNLOAD_STAGE_Y_MM;
  robot_pose.heading_rad = MISSION_UNLOAD_HEADING_RAD;
  Mission_UpdateIntegration(); /* GO_STAGE -> ALIGN */
  assert_brush_continues();
  Mission_UpdateIntegration(); /* ALIGN -> REVERSE */
  assert_brush_continues();
  robot_pose.x_mm = MISSION_UNLOAD_STOP_X_MM;
  Mission_UpdateIntegration(); /* REVERSE -> EJECT */
  assert_brush_continues();
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_EJECT);
  Mission_UpdateIntegration();
  assert_brush_continues();
  assert(servo_updates > 0);
  assert(MissionExtension_UpdateUnload(&robot_pose, 0, clock_ms).collector_pwm == 0);
  clock_ms += 5001; /* current mission module uses a 5000ms eject phase */
  Mission_UpdateIntegration(); /* DONE -> next scan */
  assert(auto_state == AUTO_SCAN);
  assert_brush_continues();
  Auto_Update(); /* no frame: must not blindly restart */
  assert(vision_motion_hold == 1);
  assert_brush_continues();
  clock_ms += 20;
  fresh();
  Auto_Update();
  assert(htim8.ccr[2] == COLLECTOR_RUN_PWM);
  auto_state = AUTO_COLLECT;
  auto_state_start_tick = clock_ms;
  clock_ms += 20; fresh(); Auto_Update();
  assert(htim8.ccr[2] == COLLECTOR_RUN_PWM);
  assert_brush_continues();
  puts("PASS: unload drives rear servo, brush stays forward through deposit and next batch");
}
static void test_scope_and_override(void)
{
  int16_t l, r; uint8_t replan;
  reset_test();
  Motor_SetTarget(40, 40); /* manual is independent of camera */
  assert(Auto_CheckVision(clock_ms) == 1 && left_pwm_percent >= 60);
  auto_state = AUTO_NAVIGATE;
  MissionExtension_ApplyForwardSafety(&robot_pose, 100, clock_ms, 40, 40, &l, &r, &replan);
  MissionExtension_ApplyForwardSafety(&robot_pose, 100, clock_ms, 40, 40, &l, &r, &replan);
  assert(Auto_CheckVision(clock_ms) == 0);
  clock_ms += 1000;
  Mission_UpdateIntegration();
  assert(left_pwm_percent == 0 && right_pwm_percent == 0);
  assert_brush_continues();
  puts("PASS: manual unaffected; obstacle recovery cannot overwrite vision stop");
}
static void test_manual_and_fault(void)
{
  const char commands[] = "FBLRSXAC0G";
  unsigned i;
  reset_test();
  for (i = 0; i < sizeof(commands)-1; ++i)
  {
    Motor_ProcessCommand(commands[i]);
    /* Combat mode commands may re-arm/rewrite the same forward brush output,
     * but no ordinary mode switch may reverse it. */
    assert(htim8.ccr[2] == COLLECTOR_RUN_PWM);
    assert(brush_in1 == 1 && brush_in2 == 0);
  }
  reset_test();
  MissionExtension_RecordCollected();
  auto_state = AUTO_COMPLETE;
  MissionExtension_StartUnload(clock_ms);
  clock_ms += 20001;
  Mission_UpdateIntegration(); /* stage timeout -> unload fault */
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_FAULT);
  assert_brush_continues();
  puts("PASS: Bluetooth start/stop/reset and unload fault do not change front brush");
}

static void imu_angle(int16_t roll, int16_t pitch, int16_t yaw)
{
  int16_t xyz[3] = {roll, pitch, yaw};
  uint8_t p[11] = {0x55,0x53,0,0,0,0,0,0,0,0,0};
  unsigned i;
  for (i=0;i<3;i++) { p[2+2*i]=(uint16_t)xyz[i]&255; p[3+2*i]=(uint16_t)xyz[i]>>8; }
  for (i=0;i<10;i++) p[10]=(uint8_t)(p[10]+p[i]);
  for (i=0;i<11;i++) { imu_rx_byte=p[i]; HAL_UART_RxCpltCallback(&huart1); }
  assert(uart_rearm_port==USART1);
}
static void lock_task_imu_zero(void)
{
  unsigned i;
  assert(greedy_imu_zero_pending && !motor_running);
  for (i = 0; i < GREEDY_IMU_ZERO_REQUIRED_FRAMES; i++)
  {
    clock_ms += 100U;
    imu_angle(0, 0, 5000);
    IMU_NavigationUpdate(clock_ms);
    Auto_Update();
    assert(!motor_running);
  }
  assert(!greedy_imu_zero_pending);
  assert(imu_navigation.reference_valid);
  assert(imu_navigation.reference_yaw_raw == 5000);
  assert(imu_navigation.heading_rad == 0.0f);
  assert(greedy_imu_local_reference_valid);
  assert(imu_local_navigation.reference_yaw_raw == 5000);
}
static void lock_local_imu_reference(int16_t yaw)
{
  unsigned i;
  assert(greedy_imu_local_pending && !motor_running);
  clock_ms += GREEDY_IMU_LOCAL_SETTLE_MS;
  for (i = 0; i < GREEDY_IMU_LOCAL_REQUIRED_FRAMES; i++)
  {
    clock_ms += 20U;
    imu_angle(0, 0, yaw);
    IMU_NavigationUpdate(clock_ms);
    Auto_Update();
    assert(!motor_running);
  }
  assert(!greedy_imu_local_pending);
  assert(greedy_imu_local_reference_valid);
  assert(imu_local_navigation.reference_yaw_raw == yaw);
  assert(imu_navigation.reference_yaw_raw == 5000);
}
static void test_debug_direct_follow(void)
{
  VisionFrame_t f = {0};
  reset_test(); Motor_ProcessCommand('F'); assert(motor_running);
  MissionExtension_RecordCollected();
  Motor_ProcessCommand('I');
  assert(auto_state == AUTO_DEBUG && !motor_running && MissionExtension_HasPayload());
  assert(!BrushFeedback_Snapshot(clock_ms).enabled);
  Auto_Update(); assert(strstr(test_log,"IMU NO_DATA"));
  test_log[0]=0; Auto_Update(); assert(!test_log[0]);
  f.target_count=1; f.targets[0]=(VisionTarget_t){1,200,500,100,200};
  Vision_ApplyFrame(&f); Auto_Update(); Mission_UpdateIntegration();
  assert(!motor_running); /* Even a visible off-centre block cannot turn I. */
  imu_angle(8192,-16384,32700);
  Motor_ProcessCommand('V');
  assert(strstr(test_log,"IMU OK BAUD=9600"));
  assert(strstr(test_log,"ROLL=45.00 PITCH=-90.00"));
  assert(strstr(test_log,"REL_YAW=0.00 ZERO=1"));
  assert(strstr(test_log,"\r\n") && !strstr(test_log,"\\r\\n"));
  test_log[0]=0; clock_ms += 100; imu_angle(0,0,-32700);
  Motor_ProcessCommand('V');
  assert(strstr(test_log,"REL_YAW=0.74")); /* yaw crosses +/-180 */
  clock_ms += JY901_STALE_MS+1; test_log[0]=0; Auto_Update();
  assert(strstr(test_log,"IMU STALE") && !motor_running && !vision_motion_hold);
  imu_angle(0,0,1000); Motor_ProcessCommand('V'); assert(!strcmp(debug_action,"OK"));
  Motor_ProcessCommand('I'); Motor_ProcessCommand('V'); assert(!imu_zero_valid);
  imu_angle(0,0,1000); Motor_ProcessCommand('V'); assert(imu_zero_valid);
  assert_brush_continues();

  Motor_ProcessCommand('S'); assert(auto_state==AUTO_IDLE && !motor_running);
  CollectorRecovery_Abort();
  Motor_ProcessCommand('I'); assert(collector_recovery_state==COLLECTOR_HOLD);
  clock_ms += 500; test_log[0]=0; Auto_Update(); Motor_ProcessCommand('V');
  assert(strstr(test_log,"IMU ") && !motor_running && !htim8.ccr[2]);
  Motor_ProcessCommand('S');
  assert(auto_state!=AUTO_DEBUG && !motor_running);

  reset_test(); clock_ms=UINT32_MAX-200; Motor_ProcessCommand('I');
  imu_angle(0,0,0); Auto_Update(); assert(imu_zero_valid);
  clock_ms += 500; imu_angle(0,0,16384); test_log[0]=0; Auto_Update();
  assert(strstr(test_log,"REL_YAW=90.00") && !motor_running);
  uart_rearm_result=1; imu_rx_byte=0;
  HAL_UART_RxCpltCallback(&huart1);
  assert(imu_rx_retry && imu_rearm_errors==1);
  uart_rearm_result=0; HAL_UART_ErrorCallback(&huart1);
  assert(imu_uart_errors==1 && uart_abort_port==USART1 && uart_rearm_port==USART1);
  imu_angle(0,0,0); Motor_ProcessCommand('V'); assert(!strcmp(debug_action,"OK"));
  assert(bluetooth_read_index==bluetooth_write_index); /* Binary IMU never becomes a command. */
  puts("PASS: stationary I IMU, fresh yaw reference, +/-180 wrap, stale/recovery, no-camera/vision/HOLD, S exit");
}

static void test_mode_switch_and_stop(void)
{
  reset_test();
  MissionExtension_RecordCollected();
  MissionExtension_StartUnload(clock_ms);
  Motor_ProcessCommand('I');
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_IDLE);
  assert(strcmp(Robot_ModeName(), "DEBUG_IMU") == 0);
  Motor_ProcessCommand('C');
  assert(!CombatStrategy_IsActive() && auto_state == AUTO_DEBUG);
  assert(strstr(test_log, "Q MUST START"));
  Motor_ProcessCommand('Q');
  assert(CombatStrategy_IsActive() && auto_state == AUTO_SCAN);
  assert(strcmp(Robot_ModeName(), "COMBAT_GREEDY") == 0);
  assert(auto_state != AUTO_DEBUG);
  Motor_ProcessCommand('A');
  assert(CombatStrategy_IsActive() && auto_state == AUTO_SCAN);
  assert(strcmp(Robot_ModeName(), "COMBAT_GREEDY") == 0);
  Motor_ProcessCommand('D');
  assert(greedy_active && auto_state == AUTO_UNLOAD_ALIGN_ZERO && !motor_running);
  assert(!BrushFeedback_Snapshot(clock_ms).enabled);
  Motor_ProcessCommand('S');
  assert(auto_state == AUTO_IDLE && strcmp(Robot_ModeName(), "COMBAT_STANDBY") == 0);
  fresh(); Auto_Update(); Mission_UpdateIntegration();
  assert(left_target_percent == 0 && right_target_percent == 0);
  MissionExtension_RecordCollected();
  MissionExtension_StartUnload(clock_ms);
  auto_state = AUTO_COMPLETE;
  Motor_ProcessCommand('S');
  Mission_UpdateIntegration();
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_IDLE);
  assert(MissionExtension_HasPayload()); /* stop preserves inventory */
  Motor_ProcessCommand('D'); Motor_ProcessCommand('0');
  assert(auto_state == AUTO_IDLE && left_target_percent == 0);
  assert(htim8.ccr[2] == 0); /* final return owns and stops the intake */
  hc_distance_mm = 432; hc_last_sample_tick = clock_ms;
  Motor_ProcessCommand('U');
  assert(ultrasonic_monitor_active && !motor_running);
  assert(!strcmp(Robot_ModeName(), "REAR_SONAR"));
  test_log[0] = 0; Motor_ProcessCommand('V');
  assert(strstr(test_log, "SONAR REAR=432 mm VALID=1"));
  Motor_ProcessCommand('S'); assert(!ultrasonic_monitor_active);
  puts("PASS: A/C/I/D/U modes; D enters IMU-zero unload return, U monitors rear sonar, stop cancels motion");
}

static void test_d_cancels_local_refresh_and_uses_start_reference(void)
{
  reset_test();
  CombatStrategy_Start(clock_ms);
  greedy_active = 1U;
  auto_state = AUTO_SCAN;
  imu_navigation.valid = 1U;
  imu_navigation.reference_valid = 1U;
  imu_navigation.reference_yaw_raw = 1234;
  imu_navigation.reference_heading_rad = 0.35f;
  imu_navigation.heading_rad = 0.15f;
  imu_navigation.last_tick = clock_ms;
  greedy_imu_local_pending = 1U;
  greedy_imu_local_resume_scan = 1U;
  greedy_imu_local_samples = 3U;

  Motor_ProcessCommand('D');
  assert(auto_state == AUTO_UNLOAD_ALIGN_ZERO);
  assert(combat_final_return_started && black_return_heading_valid);
  assert(greedy_imu_local_pending == 0U &&
         greedy_imu_local_resume_scan == 0U &&
         greedy_imu_local_samples == 0U);
  assert(Robot_AbsFloat(black_return_initial_heading_rad - 0.35f) < 0.0001f);
  /* A later local-refresh service cannot overwrite D's unload state. */
  GreedyImuLocal_Update(clock_ms + 1000U);
  assert(auto_state == AUTO_UNLOAD_ALIGN_ZERO);
  puts("PASS: D cancels an in-flight Q local refresh and aligns to the stored startup reference");
}

static void test_greedy_relative_imu_telemetry(void)
{
  reset_test();
  imu_angle(0, 0, 5000); IMU_NavigationUpdate(clock_ms);
  Motor_SetTarget(70, 70);
  assert(straight_hold_active && straight_hold_target_valid);
  clock_ms += AUTO_UPDATE_PERIOD_MS;
  imu_angle(0, 0, 1820); IMU_NavigationUpdate(clock_ms);
  StraightHold_Service(clock_ms);
  assert(left_target_percent != right_target_percent);
  Motor_SetTarget(-35, 35);
  assert(!straight_hold_active);
  puts("PASS: IMU assists straight driving only and is reset by a turn");
}

static void test_imu_pid_turn_commands(void)
{
  int16_t command;
  reset_test();
  Motor_ProcessCommand('L');
  assert(left_target_percent < 0 && right_target_percent > 0);
  assert(auto_state == AUTO_IDLE && !straight_hold_active);
  Motor_ProcessCommand('R');
  assert(left_target_percent > 0 && right_target_percent < 0);
  Motor_ProcessCommand('S');
  assert(auto_state == AUTO_IDLE && !motor_running);

  /* The unload heading controller must stop driving below 3 degrees and
   * discard integral wind-up, while errors outside that deadband still get a
   * usable command above the motor minimum. */
  BlackHeadingPid_Reset(clock_ms);
  black_heading_pid.integral = 0.25f;
  command = BlackHeadingPid_CommandWithKp(2.5f * ROBOT_PI / 180.0f,
                                          clock_ms,
                                          BLACK_HEADING_PID_KP);
  assert(command == 0);
  assert(black_heading_pid.integral == 0.0f);
  command = BlackHeadingPid_CommandWithKp(4.0f * ROBOT_PI / 180.0f,
                                          clock_ms,
                                          BLACK_HEADING_PID_KP);
  assert(command >= BLACK_HEADING_PID_MIN_PERCENT);

  /* Automatic unload alignment keeps the IMU PID active through the final
   * degrees; there is no low-torque pulse/coast window. */
  imu_navigation.valid = 1U;
  imu_navigation.heading_rad = 10.0f * ROBOT_PI / 180.0f;
  imu_navigation.last_tick = clock_ms;
  BlackHeadingPid_Reset(clock_ms);
  assert(!BlackUnload_Align(0.0f, clock_ms));
  assert(left_target_percent > 0 && right_target_percent < 0);
  clock_ms += AUTO_UPDATE_PERIOD_MS;
  imu_navigation.last_tick = clock_ms;
  imu_navigation.heading_rad = 5.0f * ROBOT_PI / 180.0f;
  assert(!BlackUnload_Align(0.0f, clock_ms) && motor_running);
  clock_ms += AUTO_UPDATE_PERIOD_MS;
  imu_navigation.last_tick = clock_ms;
  imu_navigation.heading_rad = -5.0f * ROBOT_PI / 180.0f;
  assert(!BlackUnload_Align(0.0f, clock_ms));
  assert(left_target_percent < 0 && right_target_percent > 0);
  clock_ms += AUTO_UPDATE_PERIOD_MS;
  imu_navigation.heading_rad = 0.0f;
  imu_navigation.last_tick = clock_ms;
  assert(!BlackUnload_Align(0.0f, clock_ms) && !motor_running);
  clock_ms += BLACK_HEADING_SETTLE_MS;
  imu_navigation.last_tick = clock_ms;
  assert(BlackUnload_Align(0.0f, clock_ms));
  puts("PASS: L/R stay manual; IMU PID uses a 3-degree zero-output deadband and settles");
}

static void test_bluetooth_queue(void)
{
  uint8_t command;
  unsigned i;
  reset_test();
  Bluetooth_QueueFromISR('a'); Bluetooth_QueueFromISR('c'); Bluetooth_QueueFromISR('d');
  assert(Bluetooth_GetCommand(&command) && command == 'A');
  assert(Bluetooth_GetCommand(&command) && command == 'C');
  assert(Bluetooth_GetCommand(&command) && command == 'D');
  assert(!Bluetooth_GetCommand(&command) && test_irq_mask == 0);
  Bluetooth_QueueFromISR('F'); Bluetooth_QueueFromISR('S'); Bluetooth_QueueFromISR('D');
  assert(Bluetooth_GetCommand(&command) && command == 'S');
  assert(!Bluetooth_GetCommand(&command));
  for (i = 0; i < BLUETOOTH_QUEUE_SIZE + 4; i++) Bluetooth_QueueFromISR('F');
  assert(bluetooth_drop_count == 1);
  Bluetooth_ProcessPending();
  assert(auto_state == AUTO_IDLE && left_target_percent == 0);
  Bluetooth_QueueFromISR('D'); Bluetooth_ProcessPending();
  assert(auto_state == AUTO_UNLOAD_ALIGN_ZERO && greedy_active);
  Bluetooth_QueueFromISR('X'); Bluetooth_QueueFromISR('S');
  assert(Bluetooth_GetCommand(&command) && command == 'X');
  puts("PASS: Bluetooth FIFO preserves order; stop/reset priority and overflow stop");
}

static void feed_payload(const char *payload)
{
  char packet[160];
  unsigned checksum = 0, i;
  for (i = 0; payload[i] != '\0'; i++) checksum ^= (unsigned char)payload[i];
  snprintf(packet, sizeof(packet), "$%s*%02X\r\n", payload, checksum);
  for (i = 0; packet[i] != '\0'; i++) VisionProtocol_RxByteFromISR(packet[i]);
}

static void test_vision_telemetry(void)
{
  VisionFrame_t frame = {0};
  reset_test();
  Vision_ReportTelemetry(1);
  assert(strstr(test_log, "VISION NO_FRAME") != NULL);
  test_log[0] = '\0';
  feed_payload("F,20,1"); feed_payload("T,20,0,1,-50,500,5,150"); feed_payload("E,20");
  Vision_ProcessIncoming();
  Vision_ReportTelemetry(1);
  assert(strstr(test_log, "RAW=1") && strstr(test_log, "Q=5"));
  assert(strstr(test_log, "FILTER=LOW_Q") && TargetMap_CountAvailable(&target_map) == 0);
  test_log[0] = '\0';
  frame.target_count = 1;
  frame.targets[0] = (VisionTarget_t){1, 0, 500, 30, 200};
  Vision_ApplyFrame(&frame); Vision_ReportTelemetry(1);
  assert(strstr(test_log, "RAW=1 MAP=0") && strstr(test_log, "FILTER=OK"));
  assert(TargetMap_CountAvailable(&target_map) == 0);
  Vision_ApplyFrame(&frame);
  assert(TargetMap_CountAvailable(&target_map) == 1); /* A/C policy preserved */
  robot_pose.x_mm = -5000;
  Vision_ApplyFrame(&frame); test_log[0] = '\0'; Vision_ReportTelemetry(1);
  assert(strstr(test_log, "FILTER=OUTSIDE"));
  Motor_ProcessCommand('I');
  feed_payload("F,21,1"); feed_payload("T,21,0,1,0,500,0,150"); feed_payload("E,21");
  Vision_ProcessIncoming(); Auto_Update();
  test_log[0] = '\0'; Vision_ReportTelemetry(1);
  assert(test_log[0] == '\0'); /* Camera stays quiet throughout IMU test. */
  Motor_ProcessCommand('V'); assert(strstr(test_log, "IMU NO_DATA"));
  Motor_ProcessCommand('S');
  clock_ms += AUTO_VISION_TIMEOUT_MS + 1;
  test_log[0] = '\0'; Vision_ReportTelemetry(1);
  assert(strstr(test_log, "VISION STALE") && !strstr(test_log, "SEEN"));
  feed_payload("F,22,0"); feed_payload("E,22");
  Vision_ProcessIncoming(); Auto_Update();
  test_log[0] = '\0'; Vision_ReportTelemetry(1);
  assert(strstr(test_log, "VISION OK") && strstr(test_log, "RAW=0"));
  assert(strstr(test_log, "DBG=OFF"));
  test_log[0] = '\0'; Vision_ReportTelemetry(0);
  assert(test_log[0] == '\0'); /* periodic output is rate limited */
  puts("PASS: actual UART parser to telemetry: raw Q, filter causes, confirmations, no-frame/empty/stale distinction");
}
static void assert_recovery_stopped(void)
{
  assert(left_target_percent == 0 && right_target_percent == 0);
  assert(htim3.ccr[0] == 0 && htim3.ccr[1] == 0);
  assert(htim8.ccr[2] == 0 && brush_in1 == 0 && brush_in2 == 0);
}
static void enter_recovery_reverse(void)
{
  Motor_ProcessCommand('J');
  assert(collector_recovery_state == COLLECTOR_STOPPING);
  assert_recovery_stopped();
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS - 1;
  CollectorRecovery_Update(); assert_recovery_stopped();
  clock_ms++;
  CollectorRecovery_Update();
  assert(collector_recovery_state == COLLECTOR_REVERSING);
  assert(brush_in1 == 0 && brush_in2 == 1 && htim8.ccr[2] == COLLECTOR_REVERSE_PWM);
  assert(left_target_percent < 0 && right_target_percent < 0);
}
static void finish_advance(void)
{
  assert(collector_recovery_state == COLLECTOR_ADVANCING);
  CollectorRecovery_Update();
  assert(left_target_percent > 0 && right_target_percent > 0);
  CollectorRecovery_RecordTravel(25, 25); clock_ms += 100; CollectorRecovery_Update();
  assert(collector_recovery_state == COLLECTOR_ADVANCING);
  CollectorRecovery_RecordTravel(25, 24); clock_ms += 100; CollectorRecovery_Update();
  assert(left_target_percent == 0 && right_target_percent > 0);
  CollectorRecovery_RecordTravel(0, 1); CollectorRecovery_Update();
  assert(collector_recovery_state == COLLECTOR_SETTLING && !motor_running);
  CollectorRecovery_Update();
  assert(collector_recovery_state == COLLECTOR_IDLE);
}
static void finish_recovery(void)
{
  clock_ms = collector_phase_tick + COLLECTOR_REVERSE_DURATION_MS;
  CollectorRecovery_Update();
  assert(collector_recovery_state == COLLECTOR_SETTLING);
  assert_recovery_stopped();
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS - 1;
  CollectorRecovery_Update(); assert_recovery_stopped();
  clock_ms++;
  CollectorRecovery_Update();
  finish_advance();
  assert(collector_recovery_state == COLLECTOR_IDLE);
  assert(htim8.ccr[2] == COLLECTOR_RUN_PWM && brush_in1 == 1 && brush_in2 == 0);
  assert(left_target_percent == 0 && right_target_percent == 0);
}
static void test_recovery_distance_and_time(void)
{
  uint32_t phase;
  const char blocked[] = "FACDGBLR9?";
  unsigned i;
  reset_test();
  Motor_ProcessCommand('F');
  enter_recovery_reverse();
  phase = collector_phase_tick;
  for (i = 0; i < sizeof(blocked)-1; i++) Motor_ProcessCommand(blocked[i]);
  Motor_ProcessCommand('J'); /* no repeated command can extend the reversal */
  assert(collector_phase_tick == phase && auto_state == AUTO_UNJAM);
  assert(left_target_percent < 0 && right_target_percent < 0);
  Auto_Update(); Mission_UpdateIntegration();
  assert(left_target_percent < 0 && right_target_percent < 0);
  test_log[0] = '\0'; Motor_ProcessCommand('V');
  assert(strstr(test_log, "UNJAM=REVERSING BRUSH=REVERSE"));
  Vision_ReportTelemetry(1);
  assert(!strstr(test_log, "VISION"));
  /* Real wheel-count conversion feeds the cap; one rotating wheel is enough
   * to stop backing instead of turning twice as far with the other blocked. */
  Robot_UpdateOdometry(-200, 0);
  CollectorRecovery_Update();
  assert(left_target_percent < 0);
  Robot_UpdateOdometry(-300, 0);
  CollectorRecovery_Update();
  assert(collector_left_travel_mm >= 80 && collector_backup_done);
  assert(left_target_percent == 0 && right_target_percent == 0);
  assert(htim8.ccr[2] == COLLECTOR_REVERSE_PWM);
  finish_recovery();
  assert(auto_state == AUTO_IDLE); /* no repeat of pre-J manual F */

  reset_test(); enter_recovery_reverse();
  clock_ms += COLLECTOR_BACKUP_TIMEOUT_MS - 1;
  CollectorRecovery_Update(); assert(left_target_percent < 0);
  clock_ms++;
  CollectorRecovery_Update();
  assert(collector_backup_done && left_target_percent == 0);
  assert(strstr(test_log, "BACKUP STOP: TIME LIMIT"));
  assert(collector_left_travel_mm == 0); /* missing encoder cannot back forever */
  finish_recovery();

  reset_test(); clock_ms = UINT32_MAX - 100U;
  enter_recovery_reverse(); /* HAL tick wrap is handled by unsigned subtraction */
  clock_ms += COLLECTOR_REVERSE_DURATION_MS + 1000;
  CollectorRecovery_Update(); /* delayed loop must stop both before any new output */
  assert_recovery_stopped();
  puts("PASS: unjam dead time, reverse polarity, per-wheel distance cap, encoder timeout, command ownership and tick wrap");
}
static void test_recovery_stop_and_restore(void)
{
  const char stops[] = "SX0";
  unsigned s, phase;
  for (s = 0; s < sizeof(stops)-1; s++)
    for (phase = 0; phase < 4; phase++)
    {
      reset_test();
      if (phase == 0) Motor_ProcessCommand('J');
      else enter_recovery_reverse();
      if (phase >= 2)
      {
        clock_ms += COLLECTOR_REVERSE_DURATION_MS;
        CollectorRecovery_Update();
      }
      if (phase == 3)
      {
        clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update();
        CollectorRecovery_Update(); assert(motor_running);
      }
      Bluetooth_QueueFromISR(stops[s]);
      Bluetooth_QueueFromISR('J');
      Bluetooth_ProcessPending();
      assert(collector_recovery_state == COLLECTOR_HOLD && auto_state == AUTO_IDLE);
      assert_recovery_stopped();
      clock_ms += 10000;
      fresh(); CollectorRecovery_Update(); Auto_Update(); Mission_UpdateIntegration();
      Motor_ProcessCommand('I');
      assert_recovery_stopped();
      assert(collector_recovery_state == COLLECTOR_HOLD);
      Motor_ProcessCommand('K');
      assert(collector_recovery_state == COLLECTOR_SETTLING);
      clock_ms += COLLECTOR_DIRECTION_PAUSE_MS - 1;
      CollectorRecovery_Update(); assert_recovery_stopped();
      clock_ms++;
      CollectorRecovery_Update();
      assert(collector_recovery_state == COLLECTOR_IDLE && auto_state == AUTO_IDLE);
      assert(htim8.ccr[2] == COLLECTOR_RUN_PWM && left_target_percent == 0);
    }
  reset_test(); enter_recovery_reverse(); Motor_ProcessCommand('S');
  enter_recovery_reverse(); finish_recovery();
  assert(auto_state == AUTO_IDLE);
  puts("PASS: S/X/0 cancel every recovery phase and retain brush-off HOLD; K restores only after pause; J retries explicitly");
}
static void test_recovery_resume_and_servo_exclusion(void)
{
  AutoState_t states[] = {AUTO_DEBUG, AUTO_COLLECT, AUTO_COMBAT_PATROL};
  unsigned i;
  for (i = 0; i < sizeof(states)/sizeof(states[0]); i++)
  {
    uint16_t kept, stale;
    uint32_t combat_elapsed;
    reset_test();
    auto_state = states[i];
    if (auto_state == AUTO_COMBAT_PATROL)
    {
      CombatStrategy_Start(clock_ms);
      CombatStrategy_RecordCollected(clock_ms);
    }
    MissionExtension_RecordCollected();
    kept = TargetMap_Upsert(&target_map, 1, 1000, 1000, 90, clock_ms);
    TargetMap_MarkCollected(&target_map, kept, clock_ms);
    stale = TargetMap_Upsert(&target_map, 2, 1500, 1500, 90, clock_ms);
    current_target_id = stale;
    fresh();
    enter_recovery_reverse();
    Robot_UpdateOdometry(-500, -500);
    CollectorRecovery_Update();
    fresh();
    finish_recovery();
    combat_elapsed = CombatStrategy_ElapsedMs(clock_ms);
    assert(MissionExtension_HasPayload());
    assert(TargetMap_FindById(&target_map, kept)->collected);
    assert(TargetMap_FindById(&target_map, stale) == NULL);
    assert(robot_pose.x_mm < 1500 && robot_pose.x_mm > 1400);
    assert(accepted_vision_frame_count == 0 && current_target_id == 0);
    assert(servo_updates == 0);
    if (states[i] == AUTO_DEBUG) assert(auto_state == AUTO_DEBUG);
    else assert(auto_state == AUTO_SCAN);
    if (states[i] == AUTO_COMBAT_PATROL)
    {
      assert(CombatStrategy_IsActive() && CombatStrategy_GetPayloadCount() == 1);
      assert(combat_elapsed >= COLLECTOR_REVERSE_DURATION_MS);
    }
    Auto_Update(); Mission_UpdateIntegration();
    assert(left_target_percent == 0 && right_target_percent == 0);
    fresh(); Auto_Update();
    if (states[i] != AUTO_DEBUG) assert(left_target_percent < 0 && right_target_percent > 0);
  }
  reset_test(); servo_test_active = 1;
  Motor_ProcessCommand('J'); assert(collector_recovery_state == COLLECTOR_IDLE);
  assert_brush_continues();
  reset_test(); MissionExtension_RecordCollected(); MissionExtension_StartUnload(clock_ms);
  Motor_ProcessCommand('J'); assert(collector_recovery_state == COLLECTOR_IDLE);
  assert_brush_continues();
  reset_test(); MissionExtension_RecordCollected(); auto_state = AUTO_COMPLETE;
  Motor_ProcessCommand('J'); assert(collector_recovery_state == COLLECTOR_IDLE);
  assert_brush_continues(); /* pending unload cannot resume blind after J */
  puts("PASS: unjam preserves pose/inventory/combat time, invalidates old targets, requires new vision and excludes active unloading/servo tests");
}
static void feedback_raw_steps(int count)
{
  const uint8_t next[4] = {1,3,0,2}, prev[4] = {2,0,3,1};
  int i, n = count < 0 ? -count : count;
  for (i = 0; i < n; i++)
  {
    feedback_test_ab = count < 0 ? prev[feedback_test_ab] : next[feedback_test_ab];
    BrushFeedback_EncoderEdge(feedback_test_ab);
  }
}
static void feedback_steps(int count)
{
  feedback_raw_steps(count * BRUSH_ENCODER_SIGN);
}
static void calibrate_and_run(void)
{
  Motor_ProcessCommand('H');
  assert_recovery_stopped();
  Motor_ProcessCommand('N');
  Motor_ProcessCommand('T'); /* no counts: cannot invent a CPR */
  assert(BrushFeedback_Cpr() == 0);
  feedback_steps(40);
  Motor_ProcessCommand('T');
  assert(BrushFeedback_Cpr() == 40);
  Motor_ProcessCommand('K');
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS;
  CollectorRecovery_Update();
  feedback_steps(4); clock_ms += 100; CollectorFeedback_Update();
  assert(BrushFeedback_Snapshot(clock_ms).rpm_x10 == 600);
}
static void await_auto_stall(void)
{
  unsigned i;
  for (i = 0; i < 20 && collector_recovery_state == COLLECTOR_IDLE; i++)
  {
    clock_ms += 100;
    CollectorFeedback_Update();
  }
  assert(collector_recovery_state != COLLECTOR_IDLE);
}
static void finish_encoder_cycle(void)
{
  unsigned i;
  assert(collector_recovery_state == COLLECTOR_STOPPING);
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS;
  CollectorRecovery_Update();
  assert(collector_recovery_state == COLLECTOR_REVERSING);
  for (i = 0; i < 3; i++)
  {
    feedback_steps(-20); clock_ms += 100;
    CollectorFeedback_Update(); CollectorRecovery_Update();
    assert(collector_recovery_state == COLLECTOR_REVERSING);
  }
  feedback_steps(-19); clock_ms += 100;
  CollectorFeedback_Update(); CollectorRecovery_Update();
  assert(collector_reverse_progress == 79 && collector_recovery_state == COLLECTOR_REVERSING);
  feedback_steps(-1); CollectorRecovery_Update();
  assert(collector_reverse_progress == 80 && collector_recovery_state == COLLECTOR_SETTLING);
  assert_recovery_stopped(); /* exactly two calibrated revolutions, no timer fallback */
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS;
  CollectorRecovery_Update();
  finish_advance();
  assert(collector_recovery_state == COLLECTOR_IDLE && htim8.ccr[2] == COLLECTOR_RUN_PWM);
}
static void test_feedback_commands_and_auto(void)
{
  unsigned i, attempt;
  reset_test();
  Motor_ProcessCommand('E');
  assert(!BrushFeedback_Snapshot(clock_ms).enabled);
  calibrate_and_run();
  Motor_ProcessCommand('E');
  assert(BrushFeedback_Snapshot(clock_ms).enabled);
  auto_state = AUTO_DEBUG;
  MissionExtension_RecordCollected();
  await_auto_stall();
  assert(collector_auto_cycle && collector_auto_attempts == 1);
  assert(collector_use_encoder);
  finish_encoder_cycle();
  assert(auto_state == AUTO_DEBUG && MissionExtension_HasPayload());
  Auto_Update(); assert(left_target_percent == 0 && right_target_percent == 0);
  await_auto_stall();
  assert(collector_auto_attempts == 2);
  finish_encoder_cycle();
  for (attempt = 3; attempt <= 8; attempt++)
  {
    await_auto_stall();
    assert(collector_auto_attempts == attempt);
    finish_encoder_cycle();
  }
  assert(collector_recovery_state == COLLECTOR_IDLE);
  assert(BrushFeedback_Snapshot(clock_ms).enabled);

  reset_test(); calibrate_and_run(); Motor_ProcessCommand('E');
  await_auto_stall(); finish_encoder_cycle();
  for (i = 0; i < 110; i++)
  {
    feedback_steps(4); clock_ms += 100; CollectorFeedback_Update();
  }
  assert(collector_auto_attempts == 0); /* sustained success resets the budget */
  Motor_ProcessCommand('S');
  assert(!BrushFeedback_Snapshot(clock_ms).enabled);
  assert(htim8.ccr[2] == COLLECTOR_RUN_PWM); /* normal S still keeps front brush */
  for (i = 0; i < 20; i++) { clock_ms += 100; CollectorFeedback_Update(); }
  assert(collector_recovery_state == COLLECTOR_IDLE);
}
static void test_q_auto_unjam(void)
{
  unsigned i;
  reset_test();
  assert(!BrushFeedback_Snapshot(clock_ms).enabled);
  Motor_ProcessCommand('Q'); /* The only command after reset. No E, N, T or K. */
  assert(greedy_active && BrushFeedback_Snapshot(clock_ms).enabled);
  assert(BrushFeedback_Cpr() == 0);
  assert(strstr(test_log, "COMBAT GREEDY"));
  for (i = 0; i < 11; i++)
  {
    feedback_steps(4); clock_ms += 100; CollectorFeedback_Update();
  }
  assert(BrushFeedback_Snapshot(clock_ms).forward_verified);
  CollectorFeedback_SendStatus();
  assert(strstr(test_log, "WATCH=PULSE DELTA=4 VERIFIED=1"));
  Greedy_Observe(1, 900, 1000, 0, clock_ms);
  assert(Greedy_MarkCollected(Greedy_Nearest(&robot_pose, clock_ms)->id));
  MissionExtension_RecordCollected();
  await_auto_stall();
  assert(collector_recovery_state == COLLECTOR_STOPPING);
  assert(collector_auto_cycle && collector_auto_attempts == 1 && !collector_use_encoder);
  assert(strstr(test_log, "BRUSH EVENT: NO_PULSES"));
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update();
  assert(collector_recovery_state == COLLECTOR_REVERSING);
  assert(left_target_percent < 0 && right_target_percent < 0);
  for (i = 0; i < COLLECTOR_REVERSE_DURATION_MS / 100; i++)
  {
    feedback_steps(-4); clock_ms += 100;
    CollectorFeedback_Update(); CollectorRecovery_Update();
    if ((i + 1) * 100 >= COLLECTOR_BACKUP_TIMEOUT_MS)
      assert(left_target_percent == 0 && right_target_percent == 0);
  }
  assert(collector_recovery_state == COLLECTOR_SETTLING);
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update();
  finish_advance();
  assert(greedy_active && auto_state == AUTO_SCAN && !greedy_scan_turning);
  assert(Greedy_CountCollected() == 1 && MissionExtension_HasPayload());
  assert(BrushFeedback_Snapshot(clock_ms).enabled && htim8.ccr[2] == COLLECTOR_RUN_PWM);
  await_auto_stall();
  assert(collector_auto_attempts == 2);
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update();
  finish_recovery();
  for (i = 3; i <= 8; i++)
  {
    await_auto_stall();
    assert(collector_auto_attempts == i);
    clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update();
    finish_recovery();
  }
  assert(collector_recovery_state == COLLECTOR_IDLE);
  assert(BrushFeedback_Snapshot(clock_ms).enabled);

  reset_test(); Motor_ProcessCommand('Q'); await_auto_stall();
  assert(collector_recovery_state == COLLECTOR_HOLD && collector_auto_attempts == 0);
  assert(strstr(test_log, "NO_FEEDBACK")); assert_recovery_stopped();

  reset_test(); Motor_ProcessCommand('Q');
  for (i = 0; i < 110; i++)
  {
    feedback_steps(4); clock_ms += 100; CollectorFeedback_Update();
    if (i == 0) collector_auto_attempts = 1;
  }
  assert(collector_auto_attempts == 0); /* Pulse mode also resets retry budget. */
  Motor_ProcessCommand('O'); assert(!BrushFeedback_Snapshot(clock_ms).enabled);
  Motor_ProcessCommand('Q'); assert(BrushFeedback_Snapshot(clock_ms).enabled);
  brush_writes = 0;
  Motor_ProcessCommand('S'); assert(!BrushFeedback_Snapshot(clock_ms).enabled);
  assert_brush_continues();

  reset_test(); calibrate_and_run(); Motor_ProcessCommand('Q');
  assert(BrushFeedback_Snapshot(clock_ms).enabled);
  assert(BrushFeedback_Cpr() >= 4U && strstr(test_log, "COMBAT GREEDY"));
  feedback_steps(4); clock_ms += 100; CollectorFeedback_Update();
  await_auto_stall(); assert(collector_use_encoder); finish_encoder_cycle();
  assert(greedy_active && auto_state == AUTO_SCAN);
  puts("PASS: Q arms pulse/calibrated monitoring; automatic repeated stalls recover without a retry cap; missing feedback still enters safe HOLD");
}
static void test_feedback_fault_and_manual_counting(void)
{
  reset_test(); calibrate_and_run();
  Motor_ProcessCommand('J'); finish_encoder_cycle(); /* E not required for manual counting */

  Motor_ProcessCommand('J');
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update();
  clock_ms += BRUSH_REVERSE_NO_PROGRESS_MS; CollectorRecovery_Update();
  assert(collector_recovery_state == COLLECTOR_HOLD); assert_recovery_stopped();

  reset_test(); calibrate_and_run(); Motor_ProcessCommand('J');
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update();
  BrushFeedback_EncoderEdge(feedback_test_ab ^ 3U);
  CollectorRecovery_Update();
  assert(collector_recovery_state == COLLECTOR_HOLD); assert_recovery_stopped();

  reset_test(); calibrate_and_run(); Motor_ProcessCommand('E');
  await_auto_stall();
  Motor_ProcessCommand('O');
  assert(!BrushFeedback_Snapshot(clock_ms).enabled);
  assert(collector_recovery_state == COLLECTOR_HOLD); assert_recovery_stopped();

  reset_test(); calibrate_and_run(); Motor_ProcessCommand('E');
  MissionExtension_RecordCollected(); MissionExtension_StartUnload(clock_ms);
  await_auto_stall(); /* must stop a stalled brush rather than reverse during unloading */
  assert(collector_recovery_state == COLLECTOR_HOLD && !BrushFeedback_Snapshot(clock_ms).enabled);
  assert(MissionExtension_HasPayload()); assert_recovery_stopped();
  puts("PASS: feedback calibration/arming, unlimited repeated auto recovery, counted two turns, reverse feedback fault, disarm and unload interaction");
}
static void test_brush_polarity_and_hold_response(void)
{
  unsigned i;
  reset_test(); Motor_ProcessCommand('Q');
  for (i = 0; i < 20; i++)
  {
    /* Physical edge sequence from this car, independent of the test helper. */
    feedback_raw_steps(-4); clock_ms += 100; fresh();
    if (greedy_scan_turning) Robot_UpdateOdometry(-120, 120);
    CollectorFeedback_Update(); Auto_Update();
    assert(collector_recovery_state == COLLECTOR_IDLE && greedy_active);
  }
  assert(BrushFeedback_Snapshot(clock_ms).enabled);
  assert(BrushFeedback_Snapshot(clock_ms).forward_verified);
  CollectorFeedback_SendStatus(); assert(strstr(test_log, "SIGN=-1"));
  feedback_steps(-4); clock_ms += 100; CollectorFeedback_Update();
  assert(collector_recovery_state == COLLECTOR_IDLE); /* brief reverse sample */
  feedback_steps(4); clock_ms += 100; CollectorFeedback_Update();
  for (i = 0; i < 4; i++)
  { feedback_steps(-4); clock_ms += 100; CollectorFeedback_Update(); }
  assert(collector_recovery_state == COLLECTOR_HOLD);
  assert(BrushFeedback_Snapshot(clock_ms).last_event == BRUSH_EVENT_WRONG_DIRECTION);
  assert_recovery_stopped();
  test_log[0] = '\0';
  clock_ms += BRUSH_HOLD_REPORT_MS; CollectorFeedback_Update();
  assert(strstr(test_log, "JAM_HOLD ALIVE") && strstr(test_log, "EVENT=ENCODER_SIGN"));
  test_log[0] = '\0'; clock_ms += 100; CollectorFeedback_Update();
  assert(test_log[0] == '\0'); /* periodic report is bounded */
  Bluetooth_QueueFromISR('V'); Bluetooth_ProcessPending();
  assert(strstr(test_log, "UNJAM=HOLD") && strstr(test_log, "BRUSH_FB"));
  Bluetooth_QueueFromISR('Q'); Bluetooth_ProcessPending();
  assert(strstr(test_log, "JAM_HOLD: COMMAND RECEIVED") && collector_recovery_state == COLLECTOR_HOLD);
  Bluetooth_QueueFromISR('K'); Bluetooth_ProcessPending();
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update();
  assert(collector_recovery_state == COLLECTOR_IDLE && htim8.ccr[2] == COLLECTOR_RUN_PWM);
  Bluetooth_QueueFromISR('Q'); Bluetooth_ProcessPending();
  for (i = 0; i < 20; i++)
  { feedback_raw_steps(-4); clock_ms += 100; CollectorFeedback_Update(); }
  assert(greedy_active && collector_recovery_state == COLLECTOR_IDLE);
  puts("PASS: raw negative encoder under Q, transient/persistent sign checks, HOLD heartbeat, Bluetooth V/Q/K responses and restart");
}
static void test_ultrasound_wrap(void)
{
  unsigned mode;
  for (mode = 0; mode < 3; mode++)
  {
    uint32_t normal, wrapped;
    echo_mode = mode; echo_reads = cycle_calls = 0; test_cycles = 50000;
    normal = HCSR04_Measure();
    assert(cycle_calls < 31000);
    echo_reads = cycle_calls = 0; test_cycles = UINT32_MAX - 100;
    wrapped = HCSR04_Measure();
    assert(cycle_calls < 31000 && normal == wrapped);
    if (mode < 2) assert(wrapped == 0);
    else assert(wrapped >= 170 && wrapped <= 175);
  }
  puts("PASS: ultrasonic low/high echo timeouts and valid pulse width across DWT cycle wrap");
}
static void test_greedy_memory(void)
{
  RobotPose_t pose = {0, 0, 0};
  uint16_t ids[GREEDY_CAPACITY], near_id, far_id;
  unsigned i;
  Greedy_Reset();
  Greedy_Observe(1, 1000, 0, 100, 100);
  Greedy_Observe(2, 600, 0, 0, 100);
  assert(Greedy_List(&pose, 100, ids, GREEDY_CAPACITY) == 2);
  near_id = ids[0]; far_id = ids[1];
  assert(Greedy_Find(near_id, 100)->quality == 0);
  pose.x_mm = 1100;
  assert(Greedy_Nearest(&pose, 100)->id == far_id); /* distance changes with motion */
  Greedy_Observe(1, 1050, 0, 0, 200);
  assert(Greedy_List(&pose, 200, ids, GREEDY_CAPACITY) == 2);
  assert(Greedy_Find(far_id, 200)->x_mm == 1050);
  assert(Greedy_MarkCollected(far_id));
  assert(!Greedy_MarkCollected(far_id));
  Greedy_ClearPending();
  Greedy_Observe(1, 1050, 0, 100, 5000);
  assert(!Greedy_List(&pose, 5000, ids, GREEDY_CAPACITY));
  assert(Greedy_CountCollected() == 1);
  Greedy_Observe(2, 300, 0, 0, UINT32_MAX - 100);
  assert(Greedy_List(&pose, 100, ids, GREEDY_CAPACITY) == 1);
  assert(!Greedy_List(&pose, GREEDY_TARGET_TTL_MS + 100, ids, GREEDY_CAPACITY));
  /* Full memory evicts uncollected entries only. */
  for (i = 0; i < GREEDY_CAPACITY + 4; i++) Greedy_Observe(2, i * 200.0f, 900, 0, 30000+i);
  assert(Greedy_List(&pose, 30100, ids, GREEDY_CAPACITY) == GREEDY_CAPACITY-1);
  Greedy_ClearPending(); Greedy_Observe(1, 1050, 0, 100, 30200);
  assert(!Greedy_Nearest(&pose, 30200));
  puts("PASS: greedy distance ordering/moving pose, single low-Q frame, dedup, expiry/wrap, collected tombstones and capacity");
}

static void greedy_frame(uint16_t forward)
{
  VisionFrame_t frame = {0};
  frame.target_count = 1;
  frame.targets[0].color = 1;
  frame.targets[0].forward_mm = forward;
  frame.targets[0].quality = 0;
  Vision_ApplyFrame(&frame);
}

static void greedy_drive_mm(float mm)
{
  int32_t counts = (int32_t)(mm * ENCODER_COUNTS_PER_WHEEL_REV /
                             (ROBOT_PI * WHEEL_DIAMETER_MM)) + 1;
  Robot_UpdateOdometry(counts, counts);
}

static void begin_greedy_pickup(uint16_t mm)
{
  Motor_ProcessCommand('Q');
  lock_task_imu_zero();
  greedy_frame(mm);
  clock_ms += 20; Auto_Update(); assert(auto_state == AUTO_NAVIGATE);
  clock_ms += 20; Auto_Update(); assert(auto_state == AUTO_COLLECT);
}

static void test_greedy_blind_feed(void)
{
  unsigned i;
  uint32_t started;
  reset_test(); begin_greedy_pickup(200);
  assert(greedy_collect_goal_mm == 350.0f && Greedy_CountCollected() == 0);
  /* A block leaves view at 20 cm, but valid empty frames keep arriving. */
  clock_ms += AUTO_COLLECT_DURATION_MS; fresh(); Auto_Update();
  assert(auto_state == AUTO_COLLECT && Greedy_CountCollected() == 0 && motor_running);
  greedy_drive_mm(200); clock_ms += 20; fresh(); Auto_Update();
  assert(auto_state == AUTO_COLLECT && Greedy_CountCollected() == 0);
  Greedy_SendStatus(0); assert(strstr(test_log, "GOAL=350"));
  greedy_drive_mm(151); clock_ms += 20; fresh(); Auto_Update();
  assert(auto_state == AUTO_SCAN && Greedy_CountCollected() == 0 && !motor_running);
  assert(MissionExtension_HasPayload()); assert_brush_continues();
  clock_ms += GREEDY_LOCAL_AFTER_PICK_MS; Auto_Update();
  assert(!GreedyLocal_Get()->locked && Greedy_CountCollected() == 0);

  /* No wheel feedback or a stuck wheel never turns elapsed time into success. */
  for (i = 0; i < 2; i++)
  {
    reset_test(); begin_greedy_pickup(200); started = auto_state_start_tick;
    if (i) Robot_UpdateOdometry(4000, 0);
    while ((uint32_t)(clock_ms - started) < GREEDY_COLLECT_TIMEOUT_MS)
    { clock_ms += 100; fresh(); Auto_Update(); }
    assert(auto_state == AUTO_IDLE && !motor_running && !greedy_active);
    assert(Greedy_CountCollected() == 0 && !MissionExtension_HasPayload());
    assert(!BrushFeedback_Snapshot(clock_ms).enabled);
  assert(strstr(test_log, "PICKUP TIMEOUT: ACTION FAILED"));
    assert_brush_continues();
  }

  reset_test(); begin_greedy_pickup(200);
  Robot_UpdateOdometry(-3000, 3000); /* Pivot does not count as forward travel. */
  clock_ms += 20; fresh(); Auto_Update();
  assert(auto_state == AUTO_COLLECT && Greedy_CountCollected() == 0);
  clock_ms += AUTO_VISION_TIMEOUT_MS + 1; Auto_Update();
  assert(vision_motion_hold && !motor_running && Greedy_CountCollected() == 0);
  fresh(); clock_ms += 20; Auto_Update(); assert(auto_state == AUTO_SCAN);

  reset_test(); begin_greedy_pickup(200); greedy_drive_mm(100);
  Motor_ProcessCommand('S'); clock_ms += 20; fresh(); Auto_Update();
  assert(auto_state == AUTO_IDLE && !motor_running && Greedy_CountCollected() == 0);
  reset_test(); begin_greedy_pickup(200); greedy_drive_mm(100);
  Motor_ProcessCommand('J');
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update(); finish_recovery();
  assert(auto_state == AUTO_SCAN && Greedy_CountCollected() == 0);
  assert(greedy_imu_local_pending);
  lock_local_imu_reference(5100);
  greedy_frame(100); clock_ms += 20; Auto_Update();
  clock_ms += 20; Auto_Update(); clock_ms += 20; Auto_Update(); clock_ms += 20; Auto_Update();
  assert(auto_state == AUTO_COLLECT && greedy_collect_left_mm == 0 && greedy_collect_right_mm == 0);

  reset_test(); clock_ms = UINT32_MAX - 500; begin_greedy_pickup(200);
  clock_ms += 800; fresh(); greedy_drive_mm(351); Auto_Update();
  assert(Greedy_CountCollected() == 0 && auto_state == AUTO_SCAN);

  reset_test(); Motor_ProcessCommand('Q'); lock_task_imu_zero(); greedy_frame(200);
  clock_ms += 20; Auto_Update();
  latest_vision_frame.targets[0].lateral_mm = 50;
  Vision_ApplyFrame(&latest_vision_frame);
  clock_ms += 20; Auto_Update(); assert(auto_state == AUTO_FINAL_ALIGN);
  fresh(); clock_ms += 20; Auto_Update();
  assert(!motor_running && Greedy_CountCollected() == 0); /* no blind commit without alignment */
  puts("PASS: Q blind-zone feed continues on empty frames, requires both signed wheel distances, never counts timeout, respects link/stop/unjam, alignment and tick wrap");
}

static void test_greedy_transitions(void)
{
  VisionFrame_t frame = {0};
  unsigned i;
  uint16_t ids[GREEDY_CAPACITY];
  reset_test(); Motor_ProcessCommand('Q');
  assert(greedy_active && auto_state == AUTO_SCAN);
  assert(greedy_imu_zero_pending && !motor_running);
  lock_task_imu_zero();
  Auto_Update(); assert(vision_motion_hold && !motor_running);
  fresh(); clock_ms += 20; Auto_Update();
  frame.target_count = 2;
  frame.targets[0] = (VisionTarget_t){1, 120, 700, 0, 150};
  frame.targets[1] = (VisionTarget_t){2, -100, 900, 0, 150};
  Vision_ApplyFrame(&frame); clock_ms += 20; Auto_Update();
  assert(auto_state == AUTO_NAVIGATE);
  clock_ms += 20; Auto_Update();
  assert(current_target_id == 1 && Auto_AvailableCount() == 1);
  assert(left_target_percent > right_target_percent && right_target_percent > 0);
  frame.targets[1].forward_mm = 100; /* A new closer yellow block cannot steal the lock. */
  for (i = 0; i < 40; i++)
  {
    robot_pose = (RobotPose_t){-10000 + i*1000, 20000, (float)i};
    Vision_ApplyFrame(&frame); clock_ms += 100; Auto_Update(); Mission_UpdateIntegration();
    assert(GreedyLocal_Get()->target.color == 1 && Auto_AvailableCount() == 1);
    assert(left_target_percent > right_target_percent && right_target_percent > 0);
    assert(TargetMap_CountAvailable(&target_map) == 0);
    assert(Greedy_List(&robot_pose, clock_ms, ids, GREEDY_CAPACITY) == 0);
  }
  assert(planner_calls == 0 && planned_route.count == 0);
  frame.targets[1].color = 1; /* Same-color nearer distractor also cannot steal it. */
  Vision_ApplyFrame(&frame); clock_ms += 20; Auto_Update();
  assert(GreedyLocal_Get()->target.forward_mm == 700);
  Motor_ProcessCommand('M');
  assert(strstr(test_log, "NO GLOBAL LIST") && !strstr(test_log, "DIST RANK"));
  frame.targets[0].lateral_mm = 280;
  Vision_ApplyFrame(&frame); clock_ms += 20; Auto_Update();
  assert(left_target_percent > right_target_percent && right_target_percent > 0);
  assert(!strcmp(greedy_action, "DIFF_RIGHT"));
  frame.targets[0].lateral_mm = 0;
  Vision_ApplyFrame(&frame); clock_ms += 20; Auto_Update();
  frame.targets[0].lateral_mm = -280;
  Vision_ApplyFrame(&frame); clock_ms += 20; Auto_Update();
  assert(right_target_percent > left_target_percent && left_target_percent > 0);
  assert(!strcmp(greedy_action, "DIFF_LEFT")); /* preserve existing both-forward correction */
  /* The only ultrasonic sensor faces rearward, so it must not stop forward following. */
  hc_distance_mm = 100; clock_ms += 20; Auto_Update(); assert(motor_running);
  hc_distance_mm = 0;
  fresh(); clock_ms += 20; Auto_Update();
  assert(!motor_running && Auto_AvailableCount() == 0);
  assert(!strcmp(greedy_action, "LOST"));
  clock_ms += GREEDY_LOCAL_LOST_MS; fresh(); Auto_Update();
  assert(auto_state == AUTO_SCAN && !GreedyLocal_Get()->locked);
  frame.target_count = 1; frame.targets[0] = (VisionTarget_t){2, 0, 500, 0, 150};
  Vision_ApplyFrame(&frame); clock_ms += 20; Auto_Update(); clock_ms += 20; Auto_Update();
  assert(GreedyLocal_Get()->target.color == 2 && motor_running);
  clock_ms += GREEDY_LOCAL_FOLLOW_MAX_MS;
  Vision_ApplyFrame(&frame); Auto_Update();
  assert(auto_state == AUTO_SCAN && !motor_running && !GreedyLocal_Get()->locked);
  /* A frame with no target cannot be reused to keep driving or accumulate MAP. */
  fresh(); clock_ms += 20; Auto_Update(); assert(!motor_running && Auto_AvailableCount() == 0);
  Motor_ProcessCommand('S'); assert(!greedy_active && !GreedyLocal_Get()->locked);
  Motor_ProcessCommand('Q'); Motor_ProcessCommand('A'); assert(greedy_active && CombatStrategy_IsActive());
  Motor_ProcessCommand('Q'); Motor_ProcessCommand('C'); assert(greedy_active && CombatStrategy_IsActive());
  Motor_ProcessCommand('Q'); Motor_ProcessCommand('D'); assert(greedy_active && auto_state == AUTO_UNLOAD_ALIGN_ZERO);
  puts("PASS: Q one-target lock, no global map, forward steering, arcs, rear-sonar isolation, loss/timeout and mode isolation");
}

static void scan_tick(uint32_t delta)
{
  clock_ms += delta;
  fresh();
  Auto_Update();
}

static void test_greedy_step_scan(void)
{
  unsigned i;
  reset_test(); Motor_ProcessCommand('Q');
  lock_task_imu_zero();
  for (i = 0; i < 6; i++)
  {
    scan_tick(GREEDY_SCAN_OBSERVE_MS);
    assert(motor_running && greedy_scan_turning);
    assert(left_target_percent < 0 && right_target_percent > 0);
    Robot_UpdateOdometry(-650, 650);
    scan_tick(AUTO_UPDATE_PERIOD_MS);
    assert(!motor_running && greedy_scan_steps == i + 1U);
  }
  scan_tick(GREEDY_SCAN_OBSERVE_MS);
  assert(auto_state == AUTO_COMBAT_PATROL && !motor_running);
  assert(strstr(test_log, "COMBAT EMPTY SCAN: LOCAL 1000MM PATROL"));

  /* D and an empty object scan align to the saved initial heading, then make
   * the first CCW90 turn before the rear-sonar reverse leg. */
  reset_test(); Motor_ProcessCommand('D');
  assert(auto_state == AUTO_UNLOAD_ALIGN_ZERO && !motor_running);
  lock_task_imu_zero();
  for (i = 0; i < BLACK_ALIGN_STABLE_FRAMES; i++)
  {
    clock_ms += BLACK_ALIGN_PRESETTLE_MS + AUTO_UPDATE_PERIOD_MS;
    imu_angle(0, 0, 5000); IMU_NavigationUpdate(clock_ms); fresh(); Auto_Update();
  }
  clock_ms += BLACK_HEADING_SETTLE_MS + AUTO_UPDATE_PERIOD_MS;
  imu_angle(0, 0, 5000); IMU_NavigationUpdate(clock_ms); fresh(); Auto_Update();
  assert(auto_state == AUTO_UNLOAD_TURN_CCW_90 && !motor_running);
  assert(!strcmp(greedy_action, "TURN_CCW90_FIRST"));

  /* The first IMU-PID CCW90 must finish before the rear-wall reverse starts. */
  clock_ms += 20; imu_angle(0, 0, 13192); IMU_NavigationUpdate(clock_ms);
  clock_ms += BLACK_HEADING_SETTLE_MS + AUTO_UPDATE_PERIOD_MS;
  fresh(); Auto_Update();
  assert(auto_state == AUTO_UNLOAD_TURN_CCW_90);
  clock_ms += 20; imu_angle(0, 0, 19564); IMU_NavigationUpdate(clock_ms);
  fresh(); Auto_Update();
  assert(auto_state == AUTO_UNLOAD_TURN_CCW_90);
  assert(left_target_percent < 0 && right_target_percent > 0);
  clock_ms += 20; imu_angle(0, 0, 21384); IMU_NavigationUpdate(clock_ms);
  fresh(); Auto_Update();
  assert(auto_state == AUTO_UNLOAD_TURN_CCW_90 && !motor_running);
  clock_ms += BLACK_HEADING_SETTLE_MS + AUTO_UPDATE_PERIOD_MS;
  imu_navigation.last_tick = clock_ms;
  fresh(); Auto_Update();
  assert(auto_state == AUTO_UNLOAD_REVERSE_WALL);
  assert(left_target_percent < 0 && right_target_percent < 0);
  assert(strstr(test_log, "IMU PID CCW 90 FIRST LOCKED"));

  /* Rear HC-SR04 is consumed as distinct samples, not as a level. */
  for (i = 0; i < BLACK_RETURN_REAR_CONFIRM_PACKETS; i++)
  {
    clock_ms += HC_SAMPLE_MS;
    hc_distance_mm = BLACK_RETURN_REAR_STOP_MM - 10U;
    hc_last_sample_tick = clock_ms;
    imu_angle(0, 0, 5000); IMU_NavigationUpdate(clock_ms);
    fresh(); Auto_Update();
  }
  assert(auto_state == AUTO_UNLOAD_TURN_CCW_90 && !motor_running);
  assert(!strcmp(greedy_action, "TURN_CCW90_SECOND"));

  /* The second IMU-PID CCW90 targets the heading opposite the saved startup
   * direction, then hands off to the slow forward black search. */
  clock_ms += 20; imu_angle(0, 0, 24576); IMU_NavigationUpdate(clock_ms);
  clock_ms += BLACK_HEADING_SETTLE_MS + AUTO_UPDATE_PERIOD_MS;
  fresh(); Auto_Update();
  assert(auto_state == AUTO_UNLOAD_TURN_CCW_90);
  clock_ms += 20; imu_angle(0, 0, 29564); IMU_NavigationUpdate(clock_ms);
  fresh(); Auto_Update();
  assert(auto_state == AUTO_UNLOAD_TURN_CCW_90);
  assert(left_target_percent < 0 && right_target_percent > 0);
  clock_ms += 20; imu_angle(0, 0, 32700); IMU_NavigationUpdate(clock_ms);
  fresh(); Auto_Update();
  assert(auto_state == AUTO_UNLOAD_TURN_CCW_90);
  clock_ms += 20; imu_angle(0, 0, -27768); IMU_NavigationUpdate(clock_ms);
  fresh(); Auto_Update();
  assert(auto_state == AUTO_UNLOAD_TURN_CCW_90 && !motor_running);
  clock_ms += BLACK_HEADING_SETTLE_MS + AUTO_UPDATE_PERIOD_MS;
  imu_navigation.last_tick = clock_ms;
  fresh(); Auto_Update();
  assert(auto_state == AUTO_UNLOAD_SEARCH_FORWARD);
  assert(left_target_percent > 0 && right_target_percent > 0);
  assert(strstr(test_log, "IMU PID CCW 90 SECOND LOCKED"));

  /* The new OpenMV decimal packet is converted to fixed-point internally.
   * Three stable 3.0% reports switch from wall-line travel to visual follow. */
  for (i = 0; i < BLACK_VISION_CONFIRM_PACKETS; i++)
  {
    clock_ms += BLACK_LIVE_REPORT_MS;
    feed_payload("B,1,0.000,0.300,310,230,61000,3.0");
    Vision_ProcessIncoming();
    imu_angle(0, 0, 21384); IMU_NavigationUpdate(clock_ms);
    fresh(); Auto_Update();
  }
  assert(auto_state == AUTO_UNLOAD_APPROACH);
  assert(latest_black_zone.center_x_milli == 0);
  assert(latest_black_zone.coverage_x10 == 30);
  assert(strstr(test_log, "BLACK LIVE") && strstr(test_log, "COVER_X10=30"));

  reset_test(); Motor_ProcessCommand('Q'); lock_task_imu_zero();
  scan_tick(GREEDY_SCAN_OBSERVE_MS);
  scan_tick(GREEDY_SCAN_TURN_MAX_MS);
  assert(!motor_running && !greedy_active && auto_state == AUTO_IDLE);
  assert(strstr(test_log, "CHECK WHEEL ENCODERS"));
  puts("PASS: Q stepped object scan uses CCW90/reverse180/CCW90 return; new float black 3% starts visual follow");
}

static void test_greedy_imu_navigation(void)
{
  float heading_before;
  reset_test(); Motor_ProcessCommand('Q');
  assert(greedy_imu_zero_pending && !motor_running);
  lock_task_imu_zero();
  heading_before = robot_pose.heading_rad;
  clock_ms += 20; imu_angle(0, 0, 13192); IMU_NavigationUpdate(clock_ms);
  assert(robot_pose.heading_rad == heading_before);
  scan_tick(GREEDY_SCAN_OBSERVE_MS);
  assert(greedy_scan_turning && motor_running && !straight_hold_active);
  puts("PASS: IMU does not alter encoder scan pose; it remains available for straight hold and black-return PID");
}

static void test_greedy_start_zero_gate(void)
{
  unsigned i;
  reset_test(); Motor_ProcessCommand('Q');
  assert(greedy_imu_zero_pending && !motor_running);
  for (i = 0; i < 3; i++)
  {
    clock_ms += 100U; imu_angle(0, 0, 5000);
    IMU_NavigationUpdate(clock_ms); Auto_Update();
  }
  assert(greedy_imu_zero_samples == 3U);
  clock_ms += 100U; imu_angle(0, 0, 6000); /* About 5.5 degrees: moved. */
  IMU_NavigationUpdate(clock_ms); Auto_Update();
  assert(greedy_imu_zero_samples == 1U && greedy_imu_zero_pending);
  for (i = 1; i < GREEDY_IMU_ZERO_REQUIRED_FRAMES; i++)
  {
    clock_ms += 100U; imu_angle(0, 0, 6000);
    IMU_NavigationUpdate(clock_ms); Auto_Update();
  }
  assert(!greedy_imu_zero_pending && imu_navigation.reference_valid);
  assert(imu_navigation.reference_yaw_raw == 6000);
  assert(black_return_heading_valid && black_return_initial_heading_rad == 0.0f);
  assert(auto_state == AUTO_SCAN && !motor_running);

  reset_test(); Motor_ProcessCommand('Q');
  clock_ms += GREEDY_IMU_ZERO_TIMEOUT_MS;
  Auto_Update();
  assert(auto_state == AUTO_IDLE && !greedy_active && !motor_running);
  assert(strstr(test_log, "IMU START ZERO TIMEOUT"));
  puts("PASS: Q waits for ten stationary IMU frames, restarts on movement, reports reference and safely times out");
}
static void test_greedy_dual_imu_reference(void)
{
  reset_test(); Motor_ProcessCommand('Q'); lock_task_imu_zero();
  assert(imu_navigation.reference_yaw_raw == 5000);
  assert(imu_local_navigation.reference_yaw_raw == 5000);

  GreedyImuLocal_Start(clock_ms, 1U);
  lock_local_imu_reference(6200);
  assert(auto_state == AUTO_SCAN && !motor_running);
  assert(imu_navigation.reference_yaw_raw == 5000);
  assert(imu_local_navigation.reference_yaw_raw == 6200);

  clock_ms += 20U;
  imu_angle(0, 0, 6500);
  IMU_NavigationUpdate(clock_ms);
  assert(imu_navigation.heading_rad > imu_local_navigation.heading_rad);
  Motor_SetTarget(20, 20);
  assert(straight_hold_target_valid &&
         Robot_AbsFloat(straight_hold_target_heading -
                        imu_local_navigation.heading_rad) < 0.001f);

  Motor_Stop();
  GreedyImuLocal_Start(clock_ms, 1U);
  clock_ms += GREEDY_IMU_LOCAL_TIMEOUT_MS;
  Auto_Update();
  assert(!greedy_imu_local_pending && greedy_active && auto_state == AUTO_SCAN);
  assert(imu_navigation.reference_yaw_raw == 5000);
  assert(imu_local_navigation.reference_yaw_raw == 6200);
  assert(strstr(test_log, "KEEP OLD LOCAL REF"));
  puts("PASS: local IMU refresh preserves startup zero, feeds straight hold and times out without aborting mission");
}
static void test_manual_servo_pwm(void)
{
  reset_test(); Motor_ProcessCommand('F'); assert(motor_running);
  Motor_ProcessCommand('G');
  assert(!motor_running && servo_test_active && Servo_IsRunning());
  assert(hservo_tim.ccr[0] == 1500);
  clock_ms += 250; Mission_UpdateIntegration(); assert(hservo_tim.ccr[0] == 1000);
  clock_ms += 250; Mission_UpdateIntegration(); assert(hservo_tim.ccr[0] == 500);
  clock_ms += 1000; Mission_UpdateIntegration(); assert(hservo_tim.ccr[0] == 2500);
  Motor_ProcessCommand('G');
  assert(!servo_test_active && !Servo_IsRunning() && hservo_tim.ccr[0] == 1500);
  CollectorRecovery_Abort();
  assert(collector_recovery_state == COLLECTOR_HOLD);
  Motor_ProcessCommand('G'); clock_ms += 250; Mission_UpdateIntegration();
  assert(servo_test_active && hservo_tim.ccr[0] == 1000);
  assert(collector_recovery_state == COLLECTOR_HOLD && !motor_running && htim8.ccr[2] == 0);
  Motor_ProcessCommand('S'); assert(!servo_test_active && hservo_tim.ccr[0] == 1500);
  puts("PASS: G real servo PWM interpolation, manual drive cancellation and HOLD isolation");
}

static void test_coverage_to_rear_unload(void)
{
  unsigned i;
  reset_test(); Motor_ProcessCommand('D');
  lock_task_imu_zero();
  MissionExtension_RecordCollected();
  assert(auto_state == AUTO_UNLOAD_ALIGN_ZERO && !motor_running);
  auto_state = AUTO_UNLOAD_SEARCH_FORWARD;
  auto_state_start_tick = clock_ms;
  BlackUnload_StartStraight(BLACK_RETURN_FORWARD_PERCENT, 0.0f, clock_ms);

  for (i = 0; i < BLACK_VISION_CONFIRM_PACKETS; i++)
  {
    feed_payload("B,1,0.300,0.100,200,150,22000,2.9");
    Vision_ProcessIncoming(); clock_ms += 20;
    imu_angle(0, 0, 5000); IMU_NavigationUpdate(clock_ms); Auto_Update();
  }
  assert(auto_state == AUTO_UNLOAD_SEARCH_FORWARD);
  for (i = 0; i < 3; i++)
  {
    feed_payload("B,1,0.300,0.100,200,150,22000,3.0");
    Vision_ProcessIncoming(); clock_ms += 20;
    imu_angle(0, 0, 5000); IMU_NavigationUpdate(clock_ms); Auto_Update();
  }
  assert(auto_state == AUTO_UNLOAD_APPROACH);
  feed_payload("B,1,0.400,0.100,260,180,30000,20.0");
  Vision_ProcessIncoming(); clock_ms += 20; Auto_Update();
  assert(left_target_percent > right_target_percent && right_target_percent > 0);
  assert(!straight_hold_active); /* visual is the only steering reference */
  for (i = 0; i < 3; i++)
  {
    feed_payload("B,1,0.000,0.300,310,230,61000,40.0");
    Vision_ProcessIncoming(); clock_ms += 20; Auto_Update();
  }
  /* Black confirmation first backs up 80 mm, then two-wheel IMU control
   * returns to the startup heading and performs a CCW 30-degree turn. */
  assert(auto_state == AUTO_UNLOAD_BACKUP_BEFORE_ALIGN && !motor_running);
  clock_ms += AUTO_UPDATE_PERIOD_MS; Auto_Update();
  assert(left_target_percent < 0 && right_target_percent < 0);
  Robot_UpdateOdometry(-500, -500); /* About 80.5 mm at the calibrated wheel. */
  imu_navigation.heading_rad = 0.30f;
  imu_navigation.last_tick = clock_ms + AUTO_UPDATE_PERIOD_MS;
  clock_ms += AUTO_UPDATE_PERIOD_MS; Auto_Update();
  assert(auto_state == AUTO_UNLOAD_ALIGN_INITIAL && !motor_running);

  clock_ms += AUTO_UPDATE_PERIOD_MS;
  imu_navigation.last_tick = clock_ms;
  BlackUnload_Update(clock_ms);
  assert(left_target_percent > 0 && right_target_percent < 0);

  imu_navigation.heading_rad = black_return_initial_heading_rad;
  clock_ms += AUTO_UPDATE_PERIOD_MS;
  imu_navigation.last_tick = clock_ms;
  BlackUnload_Update(clock_ms);
  assert(auto_state == AUTO_UNLOAD_ALIGN_INITIAL && !motor_running);
  clock_ms += BLACK_HEADING_SETTLE_MS;
  imu_navigation.last_tick = clock_ms;
  BlackUnload_Update(clock_ms);
  assert(auto_state == AUTO_UNLOAD_TURN_CCW_30 && !motor_running);

  clock_ms += AUTO_UPDATE_PERIOD_MS;
  imu_navigation.last_tick = clock_ms;
  BlackUnload_Update(clock_ms);
  assert(left_target_percent < 0 && right_target_percent > 0);

  imu_navigation.heading_rad = unload_turn_target_heading_rad;
  clock_ms += AUTO_UPDATE_PERIOD_MS;
  imu_navigation.last_tick = clock_ms;
  BlackUnload_Update(clock_ms);
  assert(auto_state == AUTO_UNLOAD_TURN_CCW_30 && !motor_running);
  clock_ms += BLACK_HEADING_SETTLE_MS;
  imu_navigation.last_tick = clock_ms;
  BlackUnload_Update(clock_ms);
  assert(auto_state == AUTO_UNLOAD_REVERSE);
  clock_ms += 20; Auto_Update();
  assert(left_target_percent < 0 && right_target_percent < 0);
  Robot_UpdateOdometry(-2000, -2000); /* About 322 mm at the calibrated wheel. */
  clock_ms += 20; Auto_Update();
  assert(auto_state == AUTO_COMPLETE && !motor_running);
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_EJECT);
  Mission_UpdateIntegration();
  assert(Servo_IsRunning());
  assert(strstr(test_log, "30 TARGET"));
  puts("PASS: black 3/40 thresholds, 80mm backup, two-wheel IMU initial align/CCW30, 300mm reverse/hatch unload");
}

static void test_unload_reverse_fallback(void)
{
  reset_test();
  auto_state = AUTO_UNLOAD_REVERSE;
  auto_state_start_tick = clock_ms;
  unload_reverse_left_mm = unload_reverse_right_mm = 0.0f;
  unload_sonar_close_packets = 0U;
  hc_distance_mm = 0U; /* No echo: exercise the timed unload fallback. */
  hc_last_sample_tick = clock_ms;

  clock_ms += BLACK_REVERSE_FALLBACK_MS - 1U;
  BlackUnload_Update(clock_ms);
  assert(auto_state == AUTO_UNLOAD_REVERSE && motor_running);

  clock_ms += 1U;
  BlackUnload_Update(clock_ms);
  assert(auto_state == AUTO_COMPLETE && !motor_running);
  assert(strstr(test_log, "UNLOAD REVERSE 10S FALLBACK"));
  Mission_UpdateIntegration();
  assert(Servo_IsRunning());
  puts("PASS: unload reverse uses sonar/encoder first and enters hatch eject after 10s without timeout stop");
}

static void test_black_left_wall_bump_recovery(void)
{
  unsigned i;
  reset_test(); Motor_ProcessCommand('D');
  auto_state = AUTO_UNLOAD_SEARCH_FORWARD;
  auto_state_start_tick = clock_ms;
  black_search_heading_rad = 0.0f;
  BlackBump_Reset(1U);
  imu_navigation.valid = 1U;
  imu_navigation.last_tick = clock_ms;
  imu_navigation.heading_rad = 0.0f;
  feed_payload("B,0,0.000,0.000,0,0,0,0.0"); Vision_ProcessIncoming();
  BlackUnload_StartStraight(BLACK_RETURN_FORWARD_PERCENT, 0.0f, clock_ms);

  /* Healthy low-speed travel does not trigger.  A persistent stall of either
   * wheel does trigger without requiring an IMU impact. */
  clock_ms += BLACK_BUMP_ARM_MS;
  imu_navigation.last_tick = clock_ms;
  left_speed_rpm = 10.0f; right_speed_rpm = 10.0f;
  BlackUnload_Update(clock_ms);
  assert(black_bump_phase == BLACK_BUMP_IDLE);

  left_speed_rpm = 1.0f; right_speed_rpm = 24.0f;
  for (i = 0; i <= BLACK_BUMP_CONFIRM_MS / AUTO_UPDATE_PERIOD_MS + 1U; i++)
  {
    clock_ms += AUTO_UPDATE_PERIOD_MS;
    imu_navigation.last_tick = clock_ms;
    feed_payload("B,0,0.000,0.000,0,0,0,0.0"); Vision_ProcessIncoming();
    BlackUnload_Update(clock_ms);
  }
  assert(black_bump_phase == BLACK_BUMP_STOP && !motor_running);
  assert(black_bump_attempts == 1U && strstr(test_log, "WALL-LINE DRIVE STALL"));

  clock_ms += BLACK_BUMP_STOP_MS;
  imu_navigation.last_tick = clock_ms;
  BlackUnload_Update(clock_ms);
  assert(black_bump_phase == BLACK_BUMP_BACKUP);
  assert(left_target_percent < 0 && right_target_percent < 0);

  BlackBump_RecordTravel(-BLACK_BUMP_BACKUP_DISTANCE_MM,
                         -BLACK_BUMP_BACKUP_DISTANCE_MM);
  clock_ms += AUTO_UPDATE_PERIOD_MS;
  imu_navigation.last_tick = clock_ms;
  BlackUnload_Update(clock_ms);
  assert(black_bump_phase == BLACK_BUMP_TURN_RIGHT && !motor_running);
  assert(black_search_heading_rad < imu_navigation.heading_rad);

  imu_navigation.heading_rad = black_search_heading_rad;
  imu_navigation.last_tick = clock_ms;
  BlackUnload_Update(clock_ms);
  clock_ms += BLACK_HEADING_SETTLE_MS + AUTO_UPDATE_PERIOD_MS;
  imu_navigation.last_tick = clock_ms;
  BlackUnload_Update(clock_ms);
  assert(black_bump_phase == BLACK_BUMP_ESCAPE);
  assert(left_target_percent > 0 && right_target_percent > 0);

  BlackBump_RecordTravel(BLACK_BUMP_ESCAPE_DISTANCE_MM,
                         BLACK_BUMP_ESCAPE_DISTANCE_MM);
  clock_ms += AUTO_UPDATE_PERIOD_MS;
  imu_navigation.last_tick = clock_ms;
  BlackUnload_Update(clock_ms);
  assert(black_bump_phase == BLACK_BUMP_IDLE);
  assert(strstr(test_log, "RECOVERY COMPLETE"));
  puts("PASS: either-wheel wall-line stall triggers recovery, backs 40mm, turns right 10deg and escapes 100mm");
}

static void test_greedy_no_count_limit(void)
{
  unsigned i;
  reset_test(); Motor_ProcessCommand('Q'); lock_task_imu_zero(); fresh();
  /* More than the old ten-action limit must not trigger unloading. */
  for (i = 0; i < 12U; i++)
  {
    robot_pose = (RobotPose_t){500 + i*200.0f, 1000, 0};
    clock_ms += GREEDY_LOCAL_AFTER_PICK_MS;
    greedy_frame(100); /* one frame/quality zero is enough */
    clock_ms += 20; Auto_Update(); assert(auto_state == AUTO_NAVIGATE);
    clock_ms += 20; Auto_Update(); assert(auto_state == AUTO_COLLECT);
    greedy_drive_mm(greedy_collect_goal_mm);
    clock_ms += AUTO_COLLECT_DURATION_MS; fresh(); Auto_Update();
    assert(Greedy_CountCollected() == 0);
    assert(auto_state == AUTO_SCAN);
    Mission_UpdateIntegration();
    assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_IDLE);
    assert(greedy_imu_local_pending);
    lock_local_imu_reference((int16_t)(5050 + i));
  }
  assert(planner_calls == 0 && MissionExtension_HasPayload());
  assert(strstr(test_log, "PICKUP ACTION COMPLETE: RESCAN"));
  assert(!strstr(test_log, "GREEDY BATCH="));
  puts("PASS: Q has no pickup-count limit; twelve pickup actions still rescan and do not unload before coverage completes");
}

static void test_advance_faults(void)
{
  unsigned fault;
  for (fault = 0; fault < 4; fault++)
  {
    reset_test(); enter_recovery_reverse();
    clock_ms += COLLECTOR_REVERSE_DURATION_MS; CollectorRecovery_Update();
    clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update();
    assert(collector_recovery_state == COLLECTOR_ADVANCING);
    if (fault == 0) clock_ms += COLLECTOR_ADVANCE_TIMEOUT_MS;
    if (fault == 1) CollectorRecovery_RecordTravel(-6, 0);
    if (fault == 2) CollectorRecovery_RecordTravel(0, 31);
    if (fault == 3) { CollectorRecovery_RecordTravel(20,20); Motor_ProcessCommand('H'); }
    CollectorRecovery_Update();
    assert(collector_recovery_state == COLLECTOR_HOLD);
    assert_recovery_stopped();
  }
  puts("PASS: forward 50 mm signed both-wheel goal, timeout, wrong sign, missing wheel/skew, H/S/X/0 and K no-motion; rear sonar ignored");
}
static void test_encoder_glitch_tolerance(void)
{
  unsigned i;
  reset_test(); Motor_ProcessCommand('Q');
  brush_error_warn_tick = clock_ms - BRUSH_ERROR_WARN_MS;
  for (i=0;i<11;i++) { feedback_steps(4); clock_ms+=100; CollectorFeedback_Update(); }
  feedback_test_ab ^= 3; BrushFeedback_EncoderEdge(feedback_test_ab);
  feedback_steps(4); clock_ms+=100; CollectorFeedback_Update();
  assert(collector_recovery_state == COLLECTOR_IDLE && greedy_active);
  assert(strstr(test_log,"BRUSH ENCODER WARN"));
  CollectorFeedback_SendStatus();
  assert(strstr(test_log,"ENC_HEALTH=WARN") && strstr(test_log,"LAST_EVENT=NONE"));
  clock_ms+=100; feedback_steps(4); CollectorFeedback_Update();
  for(i=0;i<2;i++) { feedback_test_ab^=3; BrushFeedback_EncoderEdge(feedback_test_ab); }
  clock_ms+=100; feedback_steps(4); CollectorFeedback_Update();
  assert(collector_recovery_state == COLLECTOR_IDLE);
  for(i=0;i<BRUSH_ERROR_WINDOW_LIMIT;i++) { feedback_test_ab^=3; BrushFeedback_EncoderEdge(feedback_test_ab); }
  clock_ms+=100; CollectorFeedback_Update();
  assert(collector_recovery_state == COLLECTOR_HOLD);
  assert_recovery_stopped();
  assert(strstr(test_log,"BRUSH ENCODER FAULT: DELTA="));
  CollectorFeedback_SendStatus(); assert(strstr(test_log,"ENC_HEALTH=FAULT"));
  clock_ms+=1500; CollectorFeedback_Update(); CollectorFeedback_SendStatus();
  assert(!BrushFeedback_Snapshot(clock_ms).encoder_fault);
  assert(BrushFeedback_Snapshot(clock_ms).last_event==BRUSH_EVENT_ENCODER_ERRORS);
  assert(collector_recovery_state==COLLECTOR_HOLD); /* healthy window never auto-releases HOLD */
  puts("PASS: actual Q isolates 1/2 glitches as warnings; dense errors stop both, diagnostics retain history, HOLD never auto-releases");
}
static void feed_wall(unsigned hit)
{
  char payload[8], packet[20];
  unsigned checksum = 0, i;
  snprintf(payload, sizeof(payload), "W,%u", hit);
  for (i = 0; payload[i] != '\0'; i++) checksum ^= (unsigned char)payload[i];
  snprintf(packet, sizeof(packet), "$%s*%02X\r\n", payload, checksum);
  for (i = 0; packet[i] != '\0'; i++)
    VisionProtocol_RxByteFromISR((uint8_t)packet[i]);
  Vision_ProcessIncoming();
}
static void test_encoder_collision_recovery(void)
{
  reset_test();
  Motor_ProcessCommand('Q');
  lock_task_imu_zero();
  fresh();
  auto_state = AUTO_NAVIGATE;
  Motor_SetTarget(70, 70);
  /* Blue-wall telemetry is accepted but never owns vehicle motion. */
  feed_wall(1); assert(!vision_wall_confirmed);
  feed_wall(1); assert(vision_wall_confirmed);
  assert(auto_state == AUTO_NAVIGATE);

  CollisionMonitor_Reset(clock_ms);
  assert(!CollisionMonitor_Update(1, clock_ms));
  CollisionMonitor_RecordTravel(5.0f, 5.0f);
  clock_ms += COLLISION_MONITOR_WINDOW_MS;
  assert(CollisionMonitor_Update(1, clock_ms));
  CollisionRecovery_Start(clock_ms);
  assert(auto_state == AUTO_COLLISION_RECOVER &&
         collision_recovery_phase == COLLISION_RECOVER_STOP);
  assert(left_pwm_percent == 0 && right_pwm_percent == 0);
  clock_ms += COLLISION_RECOVER_STOP_MS;
  CollisionRecovery_Update(clock_ms);
  assert(collision_recovery_phase == COLLISION_RECOVER_BACKUP);
  clock_ms += 20; CollisionRecovery_Update(clock_ms);
  assert(left_pwm_percent < 0 && right_pwm_percent < 0);
  CollisionMonitor_RecordTravel(-50.0f, -50.0f);
  CollisionRecovery_Update(clock_ms);
  assert(collision_recovery_phase == COLLISION_RECOVER_TURN);
  clock_ms += 20; CollisionRecovery_Update(clock_ms);
  assert(left_pwm_percent < 0 && right_pwm_percent > 0);
  CollisionMonitor_RecordTravel(-WHEEL_TRACK_MM * COLLISION_RECOVER_TURN_RAD * 0.5f,
                                  WHEEL_TRACK_MM * COLLISION_RECOVER_TURN_RAD * 0.5f);
  CollisionRecovery_Update(clock_ms);
  assert(auto_state == AUTO_SCAN && left_pwm_percent == 0 && right_pwm_percent == 0);
  assert(strstr(test_log, "BACK50 CCW110"));

  /* If either wheel travels 10 mm, the 3 s collision window restarts. */
  CollisionMonitor_Reset(clock_ms);
  assert(!CollisionMonitor_Update(1, clock_ms));
  CollisionMonitor_RecordTravel(11.0f, 0.0f);
  clock_ms += COLLISION_MONITOR_WINDOW_MS;
  assert(!CollisionMonitor_Update(1, clock_ms));
  puts("PASS: blue wall is diagnostic-only; encoder collision performs back50, dual-wheel CCW110 and Greedy rescan");
}

static void test_independent_wheel_and_brush_stop(void)
{
  reset_test();
  Motor_ProcessCommand('F');
  assert(motor_running && htim8.ccr[2] == COLLECTOR_RUN_PWM);
  Motor_ProcessCommand('Z');
  assert(motor_running && htim8.ccr[2] == 0);
  Motor_ProcessCommand('Z');
  assert(motor_running && htim8.ccr[2] == COLLECTOR_RUN_PWM);
  Motor_ProcessCommand('S');
  assert(!motor_running && htim8.ccr[2] == COLLECTOR_RUN_PWM);
  assert(strstr(test_log, "WHEELS STOPPED BY S"));
  puts("PASS: S stops wheels only and Z toggles brush without changing wheel output");
}

static void test_combat_boot_clock_and_endgame(void)
{
  uint32_t match_start;
  uint32_t preserved_elapsed;
  int16_t saved_zero;

  reset_test();
  Competition_InitializeStandby();
  assert(!CombatStrategy_IsActive());
  assert(!greedy_active && !combat_autonomy_enabled && auto_state == AUTO_IDLE);
  assert(!motor_running && htim8.ccr[2] == 0 && greedy_imu_zero_pending);
  assert(greedy_imu_zero_to_unload == IMU_ZERO_DEST_STANDBY);

  /* Motion is rejected while the independent stationary boot zero is being
   * captured, but C can arm Greedy without resetting the match clock. */
  Motor_ProcessCommand('F');
  assert(!motor_running && strstr(test_log, "BOOT IMU ZERO BUSY"));
  lock_task_imu_zero();
  saved_zero = imu_navigation.reference_yaw_raw;
  assert(!greedy_active && auto_state == AUTO_IDLE && htim8.ccr[2] == 0);
  clock_ms += 5000U;
  Motor_ProcessCommand('P');
  assert(!CombatStrategy_IsActive() && !greedy_active);
  Motor_ProcessCommand('C');
  assert(!CombatStrategy_IsActive() && !greedy_active);
  match_start = clock_ms;
  Motor_ProcessCommand('Q');
  assert(greedy_active && combat_autonomy_enabled);
  assert(CombatStrategy_ElapsedMs(clock_ms) == 0U);
  assert(htim8.ccr[2] == COLLECTOR_RUN_PWM);

  clock_ms += 1000U;
  Motor_ProcessCommand('P');
  assert(combat_p_empty_unload_mode && !greedy_imu_zero_pending);
  assert(CombatStrategy_ElapsedMs(clock_ms) == 1000U);
  assert(imu_navigation.reference_yaw_raw == saved_zero);
  preserved_elapsed = CombatStrategy_ElapsedMs(clock_ms);
  Motor_ProcessCommand('Q');
  assert(!combat_p_empty_unload_mode &&
         CombatStrategy_ElapsedMs(clock_ms) == preserved_elapsed);
  Motor_ProcessCommand('F');
  assert(!greedy_active && !combat_autonomy_enabled && motor_running);
  assert(CombatStrategy_ElapsedMs(clock_ms) == clock_ms - match_start);
  assert(imu_navigation.reference_yaw_raw == saved_zero);
  Motor_ProcessCommand('S');

  /* The final minute is clock-owned: it starts from manual standby too. */
  clock_ms = match_start + COMBAT_ENDGAME_RETURN_MS;
  Auto_Update();
  assert(combat_final_return_started && greedy_active && combat_autonomy_enabled);
  assert(auto_state == AUTO_UNLOAD_ALIGN_ZERO && htim8.ccr[2] == 0);

  /* Five minutes is an unconditional all-output stop. */
  clock_ms = match_start + COMBAT_MATCH_DURATION_MS;
  Auto_Update();
  assert(auto_state == AUTO_COMPLETE && !greedy_active && !motor_running);
  assert(htim8.ccr[2] == 0 && !Servo_IsRunning());
  puts("PASS: boot waits for Q; first Q owns the persistent clock, P/Q/manual preserve it, 240s unload and 300s hard stop");
}

static void test_p_empty_unload_and_post_unload_pacing(void)
{
  uint32_t match_start;

  reset_test();
  imu_navigation.reference_valid = 1U;
  imu_navigation.reference_yaw_raw = 0;
  imu_navigation.heading_rad = 0.0f;
  imu_navigation.valid = 1U;
  imu_navigation.last_tick = clock_ms;
  match_start = clock_ms;
  Motor_ProcessCommand('Q');
  assert(CombatStrategy_IsActive() && !combat_p_empty_unload_mode);
  clock_ms += 1234U;
  Motor_ProcessCommand('P');
  assert(combat_p_empty_unload_mode && greedy_active && !greedy_imu_zero_pending);
  assert(CombatStrategy_ElapsedMs(clock_ms) == 1234U);
  assert(imu_navigation.reference_yaw_raw == 0);

  /* P commits to unloading after the first complete empty 6x60 scan. */
  auto_state = AUTO_SCAN;
  greedy_scan_turning = 0U;
  greedy_scan_steps = GREEDY_SCAN_STEPS;
  greedy_scan_phase_tick = clock_ms - GREEDY_SCAN_OBSERVE_MS;
  latest_vision_frame.target_count = 0U;
  latest_vision_tick = clock_ms;
  Greedy_UpdateScan(clock_ms);
  assert(combat_p_unload_committed && combat_final_return_started);
  assert(auto_state == AUTO_UNLOAD_ALIGN_ZERO);
  assert(imu_navigation.reference_yaw_raw == 0);

  /* Existing hatch/shake completion hands ownership to 300 mm pacing. */
  MissionExtension_ForceStartDockedEject(clock_ms);
  auto_state = AUTO_COMPLETE;
  clock_ms += 5001U;
  Mission_UpdateIntegration();
  assert(combat_p_unload_completed);
  assert(auto_state == AUTO_POST_UNLOAD_PACE_FORWARD);
  assert(left_target_percent > 0 && right_target_percent > 0);
  assert(htim8.ccr[2] == COLLECTOR_RUN_PWM);
  assert(BrushFeedback_Snapshot(clock_ms).enabled);

  post_unload_pace_left_mm = POST_UNLOAD_PACE_DISTANCE_MM;
  post_unload_pace_right_mm = POST_UNLOAD_PACE_DISTANCE_MM;
  PostUnloadPace_Update(clock_ms);
  assert(auto_state == AUTO_POST_UNLOAD_PACE_REVERSE);
  assert(left_target_percent < 0 && right_target_percent < 0);
  post_unload_pace_left_mm = POST_UNLOAD_PACE_DISTANCE_MM;
  post_unload_pace_right_mm = POST_UNLOAD_PACE_DISTANCE_MM;
  PostUnloadPace_Update(clock_ms);
  assert(auto_state == AUTO_POST_UNLOAD_PACE_FORWARD);

  clock_ms = match_start + COMBAT_MATCH_DURATION_MS;
  Auto_Update();
  assert(auto_state == AUTO_COMPLETE && !motor_running);
  assert(htim8.ccr[2] == 0U && !Servo_IsRunning());
  puts("PASS: P preserves the startup IMU zero and empty-scan unloads; pacing keeps the brush forward, alternates 300mm legs, then hard-stops at the original Q+300s");
}
int main(void)
{
  test_manual_servo_pwm();
  reset_test(); test_pwm(); test_loss_and_recovery(); test_missing_target();
  test_deposit_next_batch(); test_scope_and_override(); test_manual_and_fault();
  test_debug_direct_follow(); test_mode_switch_and_stop();
  test_d_cancels_local_refresh_and_uses_start_reference();
  test_greedy_relative_imu_telemetry();
  test_imu_pid_turn_commands();
  test_bluetooth_queue(); test_vision_telemetry();
  test_recovery_distance_and_time(); test_recovery_stop_and_restore();
  test_advance_faults();
  test_encoder_glitch_tolerance();
  test_recovery_resume_and_servo_exclusion();
  test_feedback_commands_and_auto(); test_feedback_fault_and_manual_counting();
  test_q_auto_unjam();
  test_brush_polarity_and_hold_response();
  test_ultrasound_wrap();
  test_greedy_memory(); test_greedy_blind_feed(); test_greedy_transitions(); test_greedy_step_scan(); test_greedy_imu_navigation();
  test_greedy_start_zero_gate();
  test_greedy_dual_imu_reference();
  test_coverage_to_rear_unload(); test_unload_reverse_fallback(); test_black_left_wall_bump_recovery();
  test_greedy_no_count_limit();
  test_encoder_collision_recovery();
  test_independent_wheel_and_brush_stop();
  test_combat_boot_clock_and_endgame();
  test_p_empty_unload_and_post_unload_pacing();
  puts("control regression tests passed");
  return 0;
}
'''
program = (prefix + servo_globals + section('PTD') + section('PD') + section('PV') + '\n' +
           '\n'.join(f[:f.index('{')].strip() + ';' for f in functions) + '\n' +
           '\n'.join(functions) + checks)
with tempfile.TemporaryDirectory(prefix='car-control-test-') as tmp:
    test_c = Path(tmp) / 'control_test.c'
    test_bin = Path(tmp) / 'control_test'
    test_c.write_text(program)
    sources = ['jy901', 'imu_navigation', 'path_planner', 'target_map', 'vision_protocol', 'brush_feedback', 'greedy_collection', 'greedy_local', 'coverage_path',
               'mission_extension', 'combat_strategy']
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) +
                   ['-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-function', '-Wno-unused-variable',
                    '-DCOMPETITION_GREEDY_ONLY=0',
                    '-I' + str(ROOT/'Core/Inc'), str(test_c)] +
                   [str(ROOT/'Core/Src'/f'{name}.c') for name in sources] +
                   ['-o', str(test_bin)], check=True)
    subprocess.run([str(test_bin)], check=True)
