#ifndef COVERAGE_PATH_H
#define COVERAGE_PATH_H

#include <stdint.h>
#include "path_planner.h"

/* Q/Greedy field convention (millimetres): upper-right origin, arena in QIII.
 * Initial nose direction is -Y and is called 0 degrees by the operator. */
#define COVERAGE_FIELD_X_MIN_MM            (-3000.0f)
#define COVERAGE_FIELD_Y_MIN_MM            (-2000.0f)
#define COVERAGE_INITIAL_HEADING_RAD        (-1.57079632679f)
#define COVERAGE_OPPOSITE_HEADING_RAD       ( 1.57079632679f)
#define COVERAGE_POSITIVE_X_HEADING_RAD       0.0f
#define COVERAGE_DIAGONAL_HEADING_RAD         0.58800260355f
#define COVERAGE_UNLOAD_X_HEADING_RAD       (-3.14159265359f)
#define COVERAGE_UNLOAD_FINAL_HEADING_RAD   (-2.61799387799f)

#define COVERAGE_RETURN_REAR_Y_MM             150U
#define COVERAGE_RETURN_REAR_X_MM             220U
#define COVERAGE_UNLOAD_REAR_Y_MM             100U
#define COVERAGE_UNLOAD_REAR_X_MM             220U
#define COVERAGE_REAR_CONFIRM_SAMPLES           3U
#define COVERAGE_ADVANCE_MM                  1000.0f
#define COVERAGE_SCAN_PASSES                    2U
#define COVERAGE_SCAN_STEPS                     6U
#define COVERAGE_SCAN_STEP_RAD                  1.0471975512f
#define COVERAGE_SCAN_OBSERVE_MS              300U
#define COVERAGE_SCAN_SETTLE_MS               180U
#define COVERAGE_SCAN_TURN_TIMEOUT_MS        3000U

typedef enum
{
  COVERAGE_IDLE = 0,
  COVERAGE_RETURN_ALIGN_180,
  COVERAGE_RETURN_REVERSE_Y,
  COVERAGE_RETURN_TURN_CW_90,
  COVERAGE_RETURN_REVERSE_X,
  COVERAGE_RETURN_TURN_DIAGONAL,
  COVERAGE_ADVANCE,
  COVERAGE_SCAN_OBSERVE,
  COVERAGE_SCAN_TURN,
  COVERAGE_UNLOAD_ALIGN_INITIAL,
  COVERAGE_UNLOAD_REVERSE_Y,
  COVERAGE_UNLOAD_TURN_CW_90,
  COVERAGE_UNLOAD_REVERSE_X,
  COVERAGE_UNLOAD_TURN_CCW_30,
  COVERAGE_TURN_TEST,
  COVERAGE_FINISHED,
  COVERAGE_FAULT
} CoverageState_t;

typedef struct
{
  int16_t left_percent;
  int16_t right_percent;
  uint8_t step;
  uint8_t total;
  uint8_t finished;
  uint8_t fault;
  uint8_t target_window;
} CoverageOutput_t;

void Coverage_Reset(void);
void Coverage_Start(void);
void Coverage_StartReturn(uint32_t now_ms);
void Coverage_StartTurnTest(uint32_t now_ms, float current_heading_rad,
                            float delta_heading_rad);
void Coverage_SetInputs(uint32_t rear_distance_mm,
                        uint32_t rear_sample_tick_ms,
                        uint32_t now_ms,
                        uint8_t imu_fresh);
CoverageOutput_t Coverage_Update(const RobotPose_t *pose);
uint8_t Coverage_IsActive(void);
uint8_t Coverage_Step(void);
uint8_t Coverage_Total(void);
uint8_t Coverage_TargetWindow(void);
CoverageState_t Coverage_State(void);
const char *Coverage_StateName(void);
float Coverage_TurnTestTarget(void);

#endif
