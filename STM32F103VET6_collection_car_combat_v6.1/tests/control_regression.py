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
         'Motor_ResetPid', 'Motor_Stop', 'Robot_AbsFloat',
         'Auto_StartScan', 'Auto_Start', 'Auto_Stop', 'Auto_CheckVision',
         'Auto_Update', 'Auto_BuildPlan', 'Auto_FindCurrentVisionTarget',
         'Auto_DriveTowardLocal', 'Mission_UpdateIntegration',
         'motor3_forward', 'Vision_ApplyFrame', 'Mission_ServoUpdate',
         'Motor_ProcessCommand', 'Motor_ApplyDriveCommand', 'Debug_Start',
         'Debug_Update', 'Vision_ReportTelemetry', 'Robot_ModeName',
         'Bluetooth_QueueFromISR', 'Bluetooth_GetCommand', 'Bluetooth_ProcessPending',
         'Vision_ProcessIncoming']
functions = [function(name) for name in names]
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
static unsigned test_irq_mask;
static unsigned __get_PRIMASK(void) { return test_irq_mask; }
static void __disable_irq(void) { test_irq_mask = 1; }
static void __set_PRIMASK(unsigned mask) { test_irq_mask = mask; }
static uint32_t HAL_GetTick(void) { return clock_ms; }
static void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, int value)
{
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
static void Auto_SendMap(void) {}
static void Auto_SendPlan(void) {}
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
  CombatStrategy_Reset();
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
int main(void)
{
  reset_test(); test_pwm(); test_loss_and_recovery(); test_missing_target();
  test_deposit_next_batch(); test_scope_and_override(); test_manual_and_fault();
  test_debug_direct_follow(); test_mode_switch_and_stop();
  test_bluetooth_queue(); test_vision_telemetry();
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
    sources = ['path_planner', 'target_map', 'vision_protocol',
               'mission_extension', 'combat_strategy']
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) +
                   ['-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-function', '-Wno-unused-variable',
                    '-I' + str(ROOT/'Core/Inc'), str(test_c)] +
                   [str(ROOT/'Core/Src'/f'{name}.c') for name in sources] +
                   ['-o', str(test_bin)], check=True)
    subprocess.run([str(test_bin)], check=True)
