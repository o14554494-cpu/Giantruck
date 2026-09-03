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

def function(name):
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

names = ['Motor_ClampPercent', 'Motor_SetOne', 'Motor_Set', 'Motor_SetTarget',
         'Greedy_SendStatus', 'Auto_StateName', 'Auto_SendMap', 'Auto_SendPlan', 'Auto_AvailableCount',
         'Motor_ResetPid', 'Motor_Stop', 'Robot_AbsFloat',
         'Auto_StartScan', 'Auto_Start', 'Auto_Stop', 'Auto_CheckVision',
         'Auto_Update', 'Auto_BuildPlan', 'Auto_FindCurrentVisionTarget',
         'Auto_DriveTowardLocal', 'Mission_UpdateIntegration',
         'motor3_forward', 'Vision_ApplyFrame', 'Mission_ServoUpdate',
         'Motor_ProcessCommand', 'Motor_ApplyDriveCommand', 'Debug_Start',
         'Debug_Update', 'Vision_ReportTelemetry', 'Robot_ModeName',
         'Bluetooth_QueueFromISR', 'Bluetooth_GetCommand', 'Bluetooth_ProcessPending',
         'Vision_ProcessIncoming', 'Robot_UpdateOdometry',
         'motor3_stop', 'motor3_reverse', 'CollectorRecovery_Start',
         'CollectorRecovery_Update', 'CollectorRecovery_Abort',
         'CollectorRecovery_Restore', 'CollectorRecovery_RecordTravel',
         'CollectorRecovery_StateName', 'CollectorRecovery_SendStatus',
         'CollectorFeedback_Update', 'CollectorFeedback_SendStatus', 'HCSR04_Measure']
functions = [function(name).replace('Planner_BuildRoute(', 'CountedPlanner(') for name in names]
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
static void Servo_StartEject(void) {}
static void Servo_Stop(void) {}
static void Servo_Update(uint32_t now) { (void)now; servo_updates++; }
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
  assert(brush_writes == 0); /* no task state may rewrite brush PWM */
}
static void reset_test(void)
{
  clock_ms = 100;
  BrushFeedback_Reset(clock_ms, 0);
  feedback_test_ab = 0;
  collector_reverse_start_count = collector_reverse_start_errors = 0;
  collector_reverse_progress = collector_reverse_progress_tick = 0;
  collector_use_encoder = collector_auto_cycle = collector_auto_attempts = 0;
  collector_healthy_tracking = 0;
  collector_recovery_state = COLLECTOR_IDLE;
  collector_resume_state = AUTO_IDLE;
  collector_phase_tick = 0;
  collector_left_travel_mm = collector_right_travel_mm = 0;
  collector_backup_done = 0;
  servo_test_active = 0;
  htim8.ccr[2] = brush_in1 = brush_in2 = 0;
  CombatStrategy_Reset();
  Greedy_Reset(); greedy_active = 0;
  planner_calls = 0;
  MissionExtension_Reset();
  TargetMap_Reset(&target_map);
  memset(&planned_route, 0, sizeof(planned_route));
  memset(&latest_vision_frame, 0, sizeof(latest_vision_frame));
  robot_pose = (RobotPose_t){1500, 1000, 0};
  auto_state = AUTO_IDLE;
  vision_motion_hold = 0;
  accepted_vision_frame_count = 0;
  latest_vision_tick = 0;
  last_auto_update_tick = 0;
  auto_state_start_tick = clock_ms;
  hc_distance_mm = 0;
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
    assert_brush_continues();
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

static void test_debug_direct_follow(void)
{
  VisionFrame_t frame = {0};
  reset_test();
  Motor_ProcessCommand('d');
  assert(auto_state == AUTO_DEBUG && CombatStrategy_IsActive() == 0);
  Auto_Update();
  assert(vision_motion_hold == 1 && left_target_percent == 0);
  assert(strcmp(debug_action, "WAIT_FRAME") == 0);

  /* Deliberately bogus global pose: debug uses camera-relative coordinates. */
  robot_pose = (RobotPose_t){-5000, -5000, 0};
  frame.target_count = 1;
  frame.targets[0] = (VisionTarget_t){1, 0, 500, 0, 150};
  Vision_ApplyFrame(&frame);
  Auto_Update();
  assert(strcmp(debug_action, "FOLLOW") == 0);
  assert(left_target_percent > 0 && right_target_percent == left_target_percent);
  assert(TargetMap_CountAvailable(&target_map) == 0 && planned_route.count == 0);
  assert(strcmp(vision_filter_reason[0], "DEBUG_BYPASS") == 0);
  assert_brush_continues();

  frame.targets[0].lateral_mm = 200;
  Vision_ApplyFrame(&frame); Auto_Update();
  assert(left_target_percent > 0 && right_target_percent < 0);
  assert(strcmp(debug_action, "TURN_RIGHT") == 0);
  frame.targets[0].lateral_mm = -200;
  Vision_ApplyFrame(&frame); Auto_Update();
  assert(left_target_percent < 0 && right_target_percent > 0);
  assert(strcmp(debug_action, "TURN_LEFT") == 0);

  frame.targets[0].lateral_mm = 0;
  frame.targets[0].forward_mm = 100;
  Vision_ApplyFrame(&frame); Auto_Update();
  assert(left_target_percent == 0 && strcmp(debug_action, "NEAR") == 0);
  frame.targets[0].forward_mm = 500;
  hc_distance_mm = 100;
  Vision_ApplyFrame(&frame); Auto_Update();
  assert(left_target_percent == 0 && strcmp(debug_action, "OBSTACLE") == 0);
  hc_distance_mm = 0;

  frame.target_count = 2;
  frame.targets[0] = (VisionTarget_t){1, 0, 500, 99, 1000};
  frame.targets[1] = (VisionTarget_t){2, -100, 200, 0, 150};
  Vision_ApplyFrame(&frame); Auto_Update();
  assert(debug_target_index == 1); /* nearer target wins even with quality 0 */
  frame.target_count = 0;
  Vision_ApplyFrame(&frame); Auto_Update();
  assert(left_target_percent == 0 && right_target_percent == 0);
  assert(vision_motion_hold == 0 && strcmp(debug_action, "NO_TARGET") == 0);

  frame.target_count = 1;
  frame.targets[0] = (VisionTarget_t){1, 0, 500, 1, 150};
  Vision_ApplyFrame(&frame); Auto_Update();
  clock_ms += DEBUG_VISION_TIMEOUT_MS + 1;
  Auto_Update(); Mission_UpdateIntegration();
  assert(auto_state == AUTO_DEBUG && vision_motion_hold == 1);
  assert(left_target_percent == 0 && right_target_percent == 0);
  assert(strcmp(debug_action, "LOST") == 0);
  Vision_ApplyFrame(&frame); Auto_Update();
  assert(auto_state == AUTO_DEBUG && strcmp(debug_action, "FOLLOW") == 0);
  assert(planned_route.count == 0 && !MissionExtension_HasPayload());
  frame.targets[0].forward_mm = 0;
  Vision_ApplyFrame(&frame); Auto_Update();
  assert(left_target_percent == 0 && debug_target_index == -1);
  assert_brush_continues();
  puts("PASS: D follows one quality-0 observation without map/planning; left/right, empty, near, obstacle and stale handling");
}

static void test_mode_switch_and_stop(void)
{
  reset_test();
  MissionExtension_RecordCollected();
  MissionExtension_StartUnload(clock_ms);
  Motor_ProcessCommand('D');
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_IDLE);
  assert(strcmp(Robot_ModeName(), "DEBUG") == 0);
  Motor_ProcessCommand('C');
  assert(CombatStrategy_IsActive() && auto_state == AUTO_SCAN);
  assert(strcmp(Robot_ModeName(), "COMBAT") == 0);
  assert(strcmp(debug_action, "OFF") == 0);
  Motor_ProcessCommand('A');
  assert(!CombatStrategy_IsActive() && auto_state == AUTO_SCAN);
  assert(strcmp(Robot_ModeName(), "TECH") == 0);
  Motor_ProcessCommand('D');
  Motor_ProcessCommand('S');
  assert(auto_state == AUTO_IDLE && strcmp(Robot_ModeName(), "MANUAL") == 0);
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
  assert_brush_continues();
  puts("PASS: A/C/D mode switches, stop cancels debug/unload and late frames cannot restart stopped wheels");
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
  assert(auto_state == AUTO_DEBUG);
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
  Motor_ProcessCommand('D');
  feed_payload("F,21,1"); feed_payload("T,21,0,1,0,500,0,150"); feed_payload("E,21");
  Vision_ProcessIncoming(); Auto_Update();
  test_log[0] = '\0'; Vision_ReportTelemetry(1);
  assert(strstr(test_log, "Q=0") && strstr(test_log, "FILTER=DEBUG_BYPASS"));
  assert(strstr(test_log, "MODE=DEBUG DBG=FOLLOW"));
  clock_ms += DEBUG_VISION_TIMEOUT_MS + 1;
  test_log[0] = '\0'; Vision_ReportTelemetry(1);
  assert(strstr(test_log, "VISION STALE") && !strstr(test_log, "SEEN"));
  feed_payload("F,22,0"); feed_payload("E,22");
  Vision_ProcessIncoming(); Auto_Update();
  test_log[0] = '\0'; Vision_ReportTelemetry(1);
  assert(strstr(test_log, "VISION OK") && strstr(test_log, "RAW=0"));
  assert(strstr(test_log, "DBG=NO_TARGET"));
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
    for (phase = 0; phase < 3; phase++)
    {
      reset_test();
      if (phase == 0) Motor_ProcessCommand('J');
      else enter_recovery_reverse();
      if (phase == 2)
      {
        clock_ms += COLLECTOR_REVERSE_DURATION_MS;
        CollectorRecovery_Update();
      }
      Bluetooth_QueueFromISR(stops[s]);
      Bluetooth_QueueFromISR('J');
      Bluetooth_ProcessPending();
      assert(collector_recovery_state == COLLECTOR_HOLD && auto_state == AUTO_IDLE);
      assert_recovery_stopped();
      clock_ms += 10000;
      fresh(); CollectorRecovery_Update(); Auto_Update(); Mission_UpdateIntegration();
      Motor_ProcessCommand('D');
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
static void feedback_steps(int count)
{
  const uint8_t next[4] = {1,3,0,2}, prev[4] = {2,0,3,1};
  int i, n = count < 0 ? -count : count;
  for (i = 0; i < n; i++)
  {
    feedback_test_ab = count < 0 ? prev[feedback_test_ab] : next[feedback_test_ab];
    BrushFeedback_EncoderEdge(feedback_test_ab);
  }
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
  assert(collector_recovery_state == COLLECTOR_IDLE && htim8.ccr[2] == COLLECTOR_RUN_PWM);
}
static void test_feedback_commands_and_auto(void)
{
  unsigned i;
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
  await_auto_stall();
  assert(collector_recovery_state == COLLECTOR_HOLD);
  assert(!BrushFeedback_Snapshot(clock_ms).enabled);
  assert_recovery_stopped();

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
  puts("PASS: feedback calibration/arming, automatic retry limit, counted two turns, reverse feedback fault, disarm and unload interaction");
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

static void test_greedy_transitions(void)
{
  uint16_t first, nearer;
  reset_test(); Motor_ProcessCommand('Q');
  assert(greedy_active && auto_state == AUTO_SCAN);
  Auto_Update(); assert(vision_motion_hold && !motor_running);
  fresh(); clock_ms += 20; Auto_Update();
  greedy_frame(800); clock_ms += 20; Auto_Update();
  assert(auto_state == AUTO_PLAN);
  clock_ms += 20; Auto_Update();
  first = current_target_id;
  assert(first && auto_state == AUTO_NAVIGATE && Auto_AvailableCount() == 1);
  greedy_frame(350);
  clock_ms += GREEDY_SELECT_INTERVAL_MS; fresh(); Auto_Update();
  nearer = current_target_id;
  assert(nearer && nearer != first && planned_route.count == 0 && planner_calls == 0);
  Motor_ProcessCommand('M'); Motor_ProcessCommand('P');
  assert(strstr(test_log, "DIST RANK=1") && strstr(test_log, "MODE=GREEDY"));
  robot_pose.x_mm = Greedy_Find(nearer, clock_ms)->x_mm - 200;
  clock_ms += 20; Auto_Update(); assert(auto_state == AUTO_FINAL_ALIGN);
  Greedy_Observe(2, robot_pose.x_mm+60, robot_pose.y_mm, 100, clock_ms);
  greedy_frame(100); clock_ms += 20; Auto_Update();
  assert(auto_state == AUTO_COLLECT && current_target_id == nearer); /* near lock */
  clock_ms += AUTO_COLLECT_DURATION_MS; fresh(); Auto_Update();
  assert(Greedy_CountCollected() == 1 && auto_state == AUTO_PLAN);
  clock_ms += AUTO_VISION_TIMEOUT_MS + 1; Auto_Update();
  assert(!motor_running && vision_motion_hold && Greedy_CountCollected() == 1);
  assert(!Auto_AvailableCount());
  fresh(); clock_ms += 20; Auto_Update(); assert(auto_state == AUTO_SCAN);
  Motor_ProcessCommand('J');
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update();
  clock_ms += COLLECTOR_REVERSE_DURATION_MS; CollectorRecovery_Update();
  clock_ms += COLLECTOR_DIRECTION_PAUSE_MS; CollectorRecovery_Update();
  assert(greedy_active && auto_state == AUTO_SCAN && Greedy_CountCollected() == 1);
  Motor_ProcessCommand('S'); fresh(); Auto_Update();
  assert(!greedy_active && auto_state == AUTO_IDLE && !motor_running);
  Motor_ProcessCommand('Q'); Motor_ProcessCommand('A'); assert(!greedy_active);
  Motor_ProcessCommand('Q'); Motor_ProcessCommand('C'); assert(!greedy_active && CombatStrategy_IsActive());
  Motor_ProcessCommand('Q'); Motor_ProcessCommand('D'); assert(!greedy_active && auto_state == AUTO_DEBUG);
  puts("PASS: Q nearest retarget, near lock, M/P list, vision hold, unjam preserves count and mode/stop isolation");
}

static void test_greedy_ten_then_unload(void)
{
  unsigned i;
  reset_test(); Motor_ProcessCommand('Q'); fresh();
  /* A full empty rotation repeats scanning and never unloads early. */
  scan_accumulated_angle = AUTO_SCAN_MIN_ROTATION_RAD;
  clock_ms += 20; Auto_Update(); clock_ms += 20; Auto_Update();
  assert(auto_state == AUTO_SCAN && !MissionExtension_HasPayload());
  for (i = 0; i < GREEDY_BATCH_SIZE; i++)
  {
    robot_pose = (RobotPose_t){500 + i*200.0f, 1000, 0};
    greedy_frame(100); /* one frame/quality zero is enough */
    if (auto_state == AUTO_SCAN) { clock_ms += 20; Auto_Update(); }
    clock_ms += 20; Auto_Update(); assert(auto_state == AUTO_NAVIGATE);
    clock_ms += 20; Auto_Update(); assert(auto_state == AUTO_FINAL_ALIGN);
    clock_ms += 20; Auto_Update(); assert(auto_state == AUTO_COLLECT);
    clock_ms += AUTO_COLLECT_DURATION_MS; fresh(); Auto_Update();
    assert(Greedy_CountCollected() == i+1);
    if (i+1 < GREEDY_BATCH_SIZE)
    {
      Mission_UpdateIntegration(); assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_IDLE);
      Auto_BuildPlan(); assert(auto_state == AUTO_SCAN); /* empty but <10 keeps searching */
    }
  }
  assert(auto_state == AUTO_COMPLETE && planner_calls == 0);
  assert(strstr(test_log, "GREEDY COLLECTED=10/10 ASSUMED=1"));
  Mission_UpdateIntegration(); assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_GO_STAGE);
  robot_pose = (RobotPose_t){MISSION_UNLOAD_STAGE_X_MM, MISSION_UNLOAD_STAGE_Y_MM, 0};
  Mission_UpdateIntegration(); Mission_UpdateIntegration();
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_REVERSE);
  robot_pose.x_mm = MISSION_UNLOAD_STOP_X_MM;
  Mission_UpdateIntegration(); Mission_UpdateIntegration();
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_EJECT && servo_updates > 0);
  clock_ms += 5000; Mission_UpdateIntegration();
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_DONE && !MissionExtension_HasPayload());
  greedy_frame(100); clock_ms += 20; Auto_Update(); Mission_UpdateIntegration();
  assert(auto_state == AUTO_COMPLETE && Greedy_CountCollected() == 10);
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_DONE && !motor_running);
  puts("PASS: empty scans keep searching, ten actual state-machine pickup actions trigger one return/reverse/servo unload, then stop");
}

int main(void)
{
  reset_test(); test_pwm(); test_loss_and_recovery(); test_missing_target();
  test_deposit_next_batch(); test_scope_and_override(); test_manual_and_fault();
  test_debug_direct_follow(); test_mode_switch_and_stop();
  test_bluetooth_queue(); test_vision_telemetry();
  test_recovery_distance_and_time(); test_recovery_stop_and_restore();
  test_recovery_resume_and_servo_exclusion();
  test_feedback_commands_and_auto(); test_feedback_fault_and_manual_counting();
  test_ultrasound_wrap();
  test_greedy_memory(); test_greedy_transitions(); test_greedy_ten_then_unload();
  puts("control regression tests passed");
  return 0;
}
'''
program = (prefix + section('PTD') + section('PD') + section('PV') + '\n' +
           '\n'.join(f[:f.index('{')].strip() + ';' for f in functions) + '\n' +
           '\n'.join(functions) + checks)
with tempfile.TemporaryDirectory(prefix='car-control-test-') as tmp:
    test_c = Path(tmp) / 'control_test.c'
    test_bin = Path(tmp) / 'control_test'
    test_c.write_text(program)
    sources = ['path_planner', 'target_map', 'vision_protocol', 'brush_feedback', 'greedy_collection',
               'mission_extension', 'combat_strategy']
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) +
                   ['-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-function', '-Wno-unused-variable',
                    '-I' + str(ROOT/'Core/Inc'), str(test_c)] +
                   [str(ROOT/'Core/Src'/f'{name}.c') for name in sources] +
                   ['-o', str(test_bin)], check=True)
    subprocess.run([str(test_bin)], check=True)
