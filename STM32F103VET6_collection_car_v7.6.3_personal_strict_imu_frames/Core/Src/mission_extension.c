#include "mission_extension.h"

#include <stddef.h>

typedef enum
{
  SAFETY_CLEAR = 0,
  SAFETY_STOP,
  SAFETY_BACK_OFF,
  SAFETY_TURN
} SafetyState_t;

static MissionUnloadState_t unload_state;
static uint32_t unload_state_tick;
/* Payload is deliberately a boolean. Q no longer estimates how many blocks
 * were collected; this flag only prevents an ordinary empty unload request. */
static uint8_t payload_present;

static SafetyState_t safety_state;
static uint32_t safety_state_tick;
static int8_t safety_turn_sign = 1;
static uint8_t front_close_count;
static uint8_t rear_close_count;
static uint8_t upper_right_unload;
static float unload_stage_x, unload_stage_y, unload_stop, unload_heading;

static float Mission_Abs(float value)
{
  return value < 0.0f ? -value : value;
}

static MissionDriveOutput_t Mission_Output(int16_t left, int16_t right,
                                           int16_t collector,
                                           uint8_t active, uint8_t finished)
{
  MissionDriveOutput_t output;
  output.left_percent = left;
  output.right_percent = right;
  output.collector_pwm = collector;
  output.active = active;
  output.finished = finished;
  return output;
}

static void Mission_SetUnloadState(MissionUnloadState_t state, uint32_t now_ms)
{
  unload_state = state;
  unload_state_tick = now_ms;
}

static MissionDriveOutput_t Mission_DriveToward(const RobotPose_t *pose,
                                                float target_x,
                                                float target_y)
{
  float heading_cos = Planner_Cos(pose->heading_rad);
  float heading_sin = Planner_Sin(pose->heading_rad);
  float dx = target_x - pose->x_mm;
  float dy = target_y - pose->y_mm;
  float forward = heading_cos * dx + heading_sin * dy;
  float left = -heading_sin * dx + heading_cos * dy;
  float distance = Planner_Sqrt(dx * dx + dy * dy);
  int16_t turn;
  int16_t speed;

  if (distance <= 120.0f)
  {
    return Mission_Output(0, 0, 0, 1U, 0U);
  }

  if (forward < 40.0f)
  {
    return left >= 0.0f ? Mission_Output(-24, 24, 0, 1U, 0U) :
                          Mission_Output(24, -24, 0, 1U, 0U);
  }

  turn = (int16_t)((left / (Mission_Abs(forward) + Mission_Abs(left))) * 55.0f);
  if (turn > 25) turn = 25;
  if (turn < -25) turn = -25;
  speed = distance < 350.0f ? 22 : 30;
  return Mission_Output((int16_t)(speed - turn),
                        (int16_t)(speed + turn), 0, 1U, 0U);
}

void MissionExtension_Reset(void)
{
  unload_state = MISSION_UNLOAD_IDLE;
  unload_state_tick = 0U;
  payload_present = 0U;
  safety_state = SAFETY_CLEAR;
  safety_state_tick = 0U;
  safety_turn_sign = 1;
  front_close_count = 0U;
  rear_close_count = 0U;
  upper_right_unload = 0U;
  unload_stage_x = MISSION_UNLOAD_STAGE_X_MM;
  unload_stage_y = MISSION_UNLOAD_STAGE_Y_MM;
  unload_stop = MISSION_UNLOAD_STOP_X_MM;
  unload_heading = MISSION_UNLOAD_HEADING_RAD;
}

void MissionExtension_SetInitialPose(RobotPose_t *pose)
{
  if (pose == NULL) return;
  pose->x_mm = MISSION_START_X_MM;
  pose->y_mm = MISSION_START_Y_MM;
  pose->heading_rad = MISSION_START_HEADING_RAD;
}

void MissionExtension_CancelMotion(void)
{
  unload_state = MISSION_UNLOAD_IDLE;
  unload_state_tick = 0U;
  safety_state = SAFETY_CLEAR;
  safety_state_tick = 0U;
  safety_turn_sign = 1;
  front_close_count = 0U;
  rear_close_count = 0U;
}

uint8_t MissionExtension_TargetInsideArena(float x_mm, float y_mm)
{
  return (x_mm >= MISSION_TARGET_MARGIN_MM) &&
         (x_mm <= (MISSION_ARENA_WIDTH_MM - MISSION_TARGET_MARGIN_MM)) &&
         (y_mm >= MISSION_TARGET_MARGIN_MM) &&
         (y_mm <= (MISSION_ARENA_HEIGHT_MM - MISSION_TARGET_MARGIN_MM));
}

void MissionExtension_RecordCollected(void)
{
  payload_present = 1U;
}

uint8_t MissionExtension_HasPayload(void)
{
  return payload_present;
}

void MissionExtension_StartUnload(uint32_t now_ms)
{
  if ((payload_present != 0U) &&
      ((unload_state == MISSION_UNLOAD_IDLE) ||
       (unload_state == MISSION_UNLOAD_DONE)))
  {
    Mission_SetUnloadState(MISSION_UNLOAD_GO_STAGE, now_ms);
  }
}

void MissionExtension_ForceStartUnload(uint32_t now_ms)
{
  if ((unload_state == MISSION_UNLOAD_IDLE) || (unload_state == MISSION_UNLOAD_DONE))
    Mission_SetUnloadState(MISSION_UNLOAD_GO_STAGE, now_ms);
}

void MissionExtension_ForceStartDockedEject(uint32_t now_ms)
{
  rear_close_count = 0U;
  Mission_SetUnloadState(MISSION_UNLOAD_EJECT, now_ms);
}

void MissionExtension_UseUpperRightUnload(void)
{
  upper_right_unload = 1U;
  unload_stage_x = MISSION_Q_UNLOAD_STAGE_X_MM;
  unload_stage_y = MISSION_Q_UNLOAD_STAGE_Y_MM;
  unload_stop = MISSION_Q_UNLOAD_STOP_Y_MM;
  unload_heading = MISSION_Q_UNLOAD_HEADING_RAD;
}

MissionDriveOutput_t MissionExtension_UpdateUnload(const RobotPose_t *pose,
                                                   uint32_t rear_distance_mm,
                                                   uint32_t now_ms)
{
  float dx;
  float dy;
  float heading_error;

  if (pose == NULL)
  {
    Mission_SetUnloadState(MISSION_UNLOAD_FAULT, now_ms);
  }

  switch (unload_state)
  {
    case MISSION_UNLOAD_IDLE:
      return Mission_Output(0, 0, 0, 0U, 0U);

    case MISSION_UNLOAD_GO_STAGE:
      dx = unload_stage_x - pose->x_mm;
      dy = unload_stage_y - pose->y_mm;
      if (Planner_Sqrt(dx * dx + dy * dy) <= 120.0f)
      {
        Mission_SetUnloadState(MISSION_UNLOAD_ALIGN, now_ms);
        return Mission_Output(0, 0, 0, 1U, 0U);
      }
      if ((now_ms - unload_state_tick) > 20000U)
      {
        Mission_SetUnloadState(MISSION_UNLOAD_FAULT, now_ms);
        return Mission_Output(0, 0, 0, 1U, 0U);
      }
      return Mission_DriveToward(pose, unload_stage_x, unload_stage_y);

    case MISSION_UNLOAD_ALIGN:
      heading_error = Planner_NormalizeAngle(unload_heading - pose->heading_rad);
      if (Mission_Abs(heading_error) <= 0.10f)
      {
        Mission_SetUnloadState(MISSION_UNLOAD_REVERSE, now_ms);
        return Mission_Output(0, 0, 0, 1U, 0U);
      }
      if ((now_ms - unload_state_tick) > 8000U)
      {
        Mission_SetUnloadState(MISSION_UNLOAD_FAULT, now_ms);
        return Mission_Output(0, 0, 0, 1U, 0U);
      }
      return heading_error > 0.0f ? Mission_Output(-22, 22, 0, 1U, 0U) :
                                    Mission_Output(22, -22, 0, 1U, 0U);

    case MISSION_UNLOAD_REVERSE:
      if ((rear_distance_mm != 0U) && (rear_distance_mm <= MISSION_REAR_STOP_MM))
      {
        if (rear_close_count < 3U) rear_close_count++;
      }
      else rear_close_count = 0U;
      if ((upper_right_unload ? pose->y_mm >= unload_stop : pose->x_mm <= unload_stop) ||
          (rear_close_count >= 2U))
      {
        Mission_SetUnloadState(MISSION_UNLOAD_EJECT, now_ms);
        return Mission_Output(0, 0, 0, 1U, 0U);
      }
      if ((now_ms - unload_state_tick) > 7000U)
      {
        Mission_SetUnloadState(MISSION_UNLOAD_FAULT, now_ms);
        return Mission_Output(0, 0, 0, 1U, 0U);
      }
      return Mission_Output(-20, -20, 0, 1U, 0U);

    case MISSION_UNLOAD_EJECT:
      if ((now_ms - unload_state_tick) >= 5000U)
      {
        payload_present = 0U;
        Mission_SetUnloadState(MISSION_UNLOAD_DONE, now_ms);
        return Mission_Output(0, 0, 0, 1U, 1U);
      }
      /* 0..1500 ms: hatch moves 90 degrees inward, then 180 degrees outward.
       * Afterwards repeated short forward/back shakes dislodge remaining blocks.
       * Timed legs are deliberately short because wheel startup boost is high. */
      if ((now_ms - unload_state_tick) < 1500U)
        return Mission_Output(0, 0, 0, 1U, 0U);
      {
        uint32_t shake = (now_ms - unload_state_tick - 1500U) % 700U;
        if (shake < 220U)
          return Mission_Output(15, 15, 0, 1U, 0U);
        if (shake < 350U)
          return Mission_Output(0, 0, 0, 1U, 0U);
        if (shake < 570U)
          return Mission_Output(-15, -15, 0, 1U, 0U);
        return Mission_Output(0, 0, 0, 1U, 0U);
      }

    case MISSION_UNLOAD_DONE:
      return Mission_Output(0, 0, 0, 0U, 1U);

    case MISSION_UNLOAD_FAULT:
    default:
      return Mission_Output(0, 0, 0, 1U, 0U);
  }
}

MissionUnloadState_t MissionExtension_GetUnloadState(void)
{
  return unload_state;
}

const char *MissionExtension_UnloadStateName(void)
{
  switch (unload_state)
  {
    case MISSION_UNLOAD_IDLE:     return "IDLE";
    case MISSION_UNLOAD_GO_STAGE: return "GO_STAGE";
    case MISSION_UNLOAD_ALIGN:    return "ALIGN";
    case MISSION_UNLOAD_REVERSE:  return "REVERSE";
    case MISSION_UNLOAD_EJECT:    return "EJECT";
    case MISSION_UNLOAD_DONE:     return "DONE";
    case MISSION_UNLOAD_FAULT:    return "FAULT";
    default:                      return "UNKNOWN";
  }
}

uint8_t MissionExtension_ApplyForwardSafety(const RobotPose_t *pose,
                                            uint32_t front_distance_mm,
                                            uint32_t now_ms,
                                            int16_t requested_left,
                                            int16_t requested_right,
                                            int16_t *safe_left,
                                            int16_t *safe_right,
                                            uint8_t *request_replan)
{
  uint8_t forward_requested;
  uint8_t near_wall = 0U;
  uint8_t obstacle = 0U;

  if ((pose == NULL) || (safe_left == NULL) || (safe_right == NULL) ||
      (request_replan == NULL))
  {
    return 0U;
  }

  *safe_left = requested_left;
  *safe_right = requested_right;
  *request_replan = 0U;
  forward_requested = (requested_left > 0) && (requested_right > 0);

  if (forward_requested != 0U)
  {
    float predicted_x = pose->x_mm + Planner_Cos(pose->heading_rad) * 220.0f;
    float predicted_y = pose->y_mm + Planner_Sin(pose->heading_rad) * 220.0f;
    near_wall = (predicted_x < MISSION_ROBOT_WALL_MARGIN_MM) ||
                (predicted_x > (MISSION_ARENA_WIDTH_MM - MISSION_ROBOT_WALL_MARGIN_MM)) ||
                (predicted_y < MISSION_ROBOT_WALL_MARGIN_MM) ||
                (predicted_y > (MISSION_ARENA_HEIGHT_MM - MISSION_ROBOT_WALL_MARGIN_MM));

    if ((front_distance_mm != 0U) &&
        (front_distance_mm <= MISSION_FRONT_STOP_MM))
    {
      if (front_close_count < 3U) front_close_count++;
    }
    else
    {
      front_close_count = 0U;
    }
    obstacle = (front_close_count >= 2U) || near_wall;
  }
  else if (safety_state == SAFETY_CLEAR)
  {
    front_close_count = 0U;
  }

  if ((safety_state == SAFETY_CLEAR) && (obstacle != 0U))
  {
    float to_centre_x = MISSION_ARENA_WIDTH_MM * 0.5f - pose->x_mm;
    float to_centre_y = MISSION_ARENA_HEIGHT_MM * 0.5f - pose->y_mm;
    float cross = Planner_Cos(pose->heading_rad) * to_centre_y -
                  Planner_Sin(pose->heading_rad) * to_centre_x;
    safety_turn_sign = cross >= 0.0f ? 1 : -1;
    safety_state = SAFETY_STOP;
    safety_state_tick = now_ms;
  }

  switch (safety_state)
  {
    case SAFETY_CLEAR:
      return 0U;

    case SAFETY_STOP:
      *safe_left = 0;
      *safe_right = 0;
      if ((now_ms - safety_state_tick) >= 150U)
      {
        safety_state = SAFETY_BACK_OFF;
        safety_state_tick = now_ms;
      }
      return 1U;

    case SAFETY_BACK_OFF:
      *safe_left = -18;
      *safe_right = -18;
      if ((now_ms - safety_state_tick) >= 350U)
      {
        safety_state = SAFETY_TURN;
        safety_state_tick = now_ms;
      }
      return 1U;

    case SAFETY_TURN:
      *safe_left = (int16_t)(-24 * safety_turn_sign);
      *safe_right = (int16_t)(24 * safety_turn_sign);
      if ((now_ms - safety_state_tick) >= 650U)
      {
        safety_state = SAFETY_CLEAR;
        safety_state_tick = now_ms;
        front_close_count = 0U;
        *request_replan = 1U;
      }
      return 1U;

    default:
      safety_state = SAFETY_CLEAR;
      return 0U;
  }
}
