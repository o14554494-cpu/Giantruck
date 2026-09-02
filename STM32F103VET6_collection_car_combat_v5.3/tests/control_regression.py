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
         'Motor_ProcessCommand', 'Motor_ApplyDriveCommand']
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
static uint32_t HAL_GetTick(void) { return clock_ms; }
static void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, int value)
{
  if (port == GPIOB && pin == GPIO_PIN_0) brush_in1 = value;
  if (port == GPIOB && pin == GPIO_PIN_1) brush_in2 = value;
}
static void Vision_SendMode(char mode, uint8_t color)
{ (void)mode; (void)color; mode_requests++; }
static void Motor_SendText(const char *text) { (void)text; }
static void Buzzer_NotifyTargetFound(void) {}
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
int main(void)
{
  reset_test(); test_pwm(); test_loss_and_recovery(); test_missing_target();
  test_deposit_next_batch(); test_scope_and_override(); test_manual_and_fault();
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
