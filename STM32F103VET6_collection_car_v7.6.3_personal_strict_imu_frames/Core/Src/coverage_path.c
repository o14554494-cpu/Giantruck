#include "coverage_path.h"

#define COVERAGE_ALIGN_TOLERANCE_RAD        0.035f
#define COVERAGE_TURN_PID_KP                 24.0f
#define COVERAGE_TURN_PID_KI                  0.8f
#define COVERAGE_TURN_PID_KD                  1.5f
#define COVERAGE_TURN_PID_MIN_PERCENT            8
#define COVERAGE_TURN_PID_MAX_PERCENT           28
#define COVERAGE_REVERSE_SPEED_PERCENT         12
#define COVERAGE_REVERSE_PID_KP               8.0f
#define COVERAGE_REVERSE_PID_KI               0.05f
#define COVERAGE_REVERSE_PID_KD                0.0f
#define COVERAGE_REVERSE_PID_MAX_PERCENT         3
#define COVERAGE_REVERSE_DEADBAND_RAD         0.052f
#define COVERAGE_REVERSE_FILTER_ALPHA          0.15f
#define COVERAGE_REVERSE_SLEW_PER_STEP          0.25f
#define COVERAGE_REVERSE_PIVOT_ENTER_RAD       0.22f
#define COVERAGE_REVERSE_PIVOT_EXIT_RAD        0.052f
#define COVERAGE_HEADING_PID_I_LIMIT          0.5f
#define COVERAGE_HEADING_PID_DT_MIN          0.005f
#define COVERAGE_HEADING_PID_DT_MAX          0.100f
#define COVERAGE_ADVANCE_SPEED_PERCENT         28
#define COVERAGE_ALIGN_TIMEOUT_MS            8000U
#define COVERAGE_REVERSE_TIMEOUT_MS          12000U
#define COVERAGE_ADVANCE_TIMEOUT_MS          12000U
#define COVERAGE_IMU_WAIT_TIMEOUT_MS            1500U

typedef struct
{
  float integral;
  float previous_error;
  uint32_t previous_tick;
  uint8_t valid;
} CoverageHeadingPid_t;

static CoverageState_t state;
static uint32_t state_tick;
static uint32_t now_tick;
static uint32_t rear_tick;
static uint32_t consumed_rear_tick;
static uint32_t rear_mm;
static uint8_t rear_close_count;
static uint8_t imu_is_fresh;
static uint8_t scan_pass;
static uint8_t scan_step;
static uint32_t scan_settle_tick;
static float segment_start_x;
static float segment_start_y;
static float scan_pass_start_heading;
static float turn_test_target_heading;
static CoverageHeadingPid_t turn_heading_pid;
static CoverageHeadingPid_t reverse_heading_pid;
static float reverse_filtered_error;
static float reverse_correction;
static uint8_t reverse_filter_valid;
static uint8_t reverse_pivot_active;

static float Coverage_Abs(float value)
{
  return value < 0.0f ? -value : value;
}

static float Coverage_Clamp(float value, float minimum, float maximum)
{
  if (value > maximum) return maximum;
  if (value < minimum) return minimum;
  return value;
}

static void Coverage_HeadingPidReset(CoverageHeadingPid_t *pid)
{
  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
  pid->previous_tick = now_tick;
  pid->valid = 0U;
}

static float Coverage_HeadingPidStep(CoverageHeadingPid_t *pid,
                                     float error,
                                     float kp, float ki, float kd,
                                     float output_limit)
{
  float dt = COVERAGE_HEADING_PID_DT_MIN;
  float derivative = 0.0f;
  float output;
  if (pid->valid != 0U)
  {
    uint32_t elapsed_ms = now_tick - pid->previous_tick;
    dt = (float)elapsed_ms / 1000.0f;
    dt = Coverage_Clamp(dt, COVERAGE_HEADING_PID_DT_MIN,
                        COVERAGE_HEADING_PID_DT_MAX);
    derivative = (error - pid->previous_error) / dt;
  }
  pid->integral += error * dt;
  pid->integral = Coverage_Clamp(pid->integral,
                                 -COVERAGE_HEADING_PID_I_LIMIT,
                                  COVERAGE_HEADING_PID_I_LIMIT);
  output = kp * error + ki * pid->integral + kd * derivative;
  pid->previous_error = error;
  pid->previous_tick = now_tick;
  pid->valid = 1U;
  return Coverage_Clamp(output, -output_limit, output_limit);
}

static void Coverage_Enter(CoverageState_t next)
{
  state = next;
  state_tick = now_tick;
  rear_close_count = 0U;
  consumed_rear_tick = rear_tick;
  Coverage_HeadingPidReset(&turn_heading_pid);
  Coverage_HeadingPidReset(&reverse_heading_pid);
  reverse_filtered_error = 0.0f;
  reverse_correction = 0.0f;
  reverse_filter_valid = 0U;
  reverse_pivot_active = 0U;
  scan_settle_tick = 0U;
}

static uint8_t Coverage_RearReached(uint32_t threshold_mm)
{
  if (rear_tick == consumed_rear_tick) return 0U;
  consumed_rear_tick = rear_tick;
  if (rear_mm != 0U && rear_mm <= threshold_mm)
  {
    if (rear_close_count < COVERAGE_REAR_CONFIRM_SAMPLES) rear_close_count++;
  }
  else rear_close_count = 0U;
  return rear_close_count >= COVERAGE_REAR_CONFIRM_SAMPLES;
}

static uint8_t Coverage_Align(float current, float target,
                              CoverageOutput_t *out)
{
  float error = Planner_NormalizeAngle(target - current);
  float command;
  float magnitude;
  if (Coverage_Abs(error) <= COVERAGE_ALIGN_TOLERANCE_RAD)
  {
    out->left_percent = 0;
    out->right_percent = 0;
    return 1U;
  }
  command = Coverage_HeadingPidStep(&turn_heading_pid, error,
                                    COVERAGE_TURN_PID_KP,
                                    COVERAGE_TURN_PID_KI,
                                    COVERAGE_TURN_PID_KD,
                                    COVERAGE_TURN_PID_MAX_PERCENT);
  /* Never let derivative braking reverse the requested turn direction. */
  if ((command > 0.0f) != (error > 0.0f)) command = 0.0f;
  magnitude = Coverage_Abs(command);
  if (magnitude < COVERAGE_TURN_PID_MIN_PERCENT)
    magnitude = COVERAGE_TURN_PID_MIN_PERCENT;
  if (error > 0.0f)
  {
    out->left_percent = -(int16_t)magnitude;
    out->right_percent = (int16_t)magnitude;
  }
  else
  {
    out->left_percent = (int16_t)magnitude;
    out->right_percent = -(int16_t)magnitude;
  }
  return 0U;
}

/* Reverse heading hold. A small IMU error produces differential reverse
 * wheel commands. A larger error stops translation and pivots first, so wheel
 * slip cannot make the vehicle keep backing diagonally into a wall. */
static void Coverage_ReverseHold(float current, float target,
                                 CoverageOutput_t *out)
{
  float error = Planner_NormalizeAngle(target - current);
  float requested_correction;
  float slew;

  /* Latch the pivot recovery. Without hysteresis, IMU noise around the old
   * threshold repeatedly switched between pivoting and reversing. */
  if (Coverage_Abs(error) >= COVERAGE_REVERSE_PIVOT_ENTER_RAD)
    reverse_pivot_active = 1U;
  if (reverse_pivot_active != 0U)
  {
    if (Coverage_Abs(error) <= COVERAGE_REVERSE_PIVOT_EXIT_RAD)
    {
      reverse_pivot_active = 0U;
      Coverage_HeadingPidReset(&turn_heading_pid);
      Coverage_HeadingPidReset(&reverse_heading_pid);
      reverse_filtered_error = 0.0f;
      reverse_correction = 0.0f;
      reverse_filter_valid = 0U;
    }
    else
    {
      reverse_correction = 0.0f;
      reverse_filter_valid = 0U;
      Coverage_HeadingPidReset(&reverse_heading_pid);
      (void)Coverage_Align(current, target, out);
      return;
    }
  }

  /* Heading noise below about 3 degrees must not swap the faster wheel.
   * Reset I memory inside the deadband and ramp the previous correction back
   * to zero instead of changing it abruptly. */
  if (Coverage_Abs(error) <= COVERAGE_REVERSE_DEADBAND_RAD)
  {
    Coverage_HeadingPidReset(&reverse_heading_pid);
    reverse_filtered_error = 0.0f;
    reverse_filter_valid = 0U;
    requested_correction = 0.0f;
  }
  else
  {
    if (reverse_filter_valid == 0U)
    {
      reverse_filtered_error = error;
      reverse_filter_valid = 1U;
    }
    else
    {
      reverse_filtered_error += COVERAGE_REVERSE_FILTER_ALPHA *
                                (error - reverse_filtered_error);
    }
    requested_correction = Coverage_HeadingPidStep(
        &reverse_heading_pid, reverse_filtered_error,
        COVERAGE_REVERSE_PID_KP, COVERAGE_REVERSE_PID_KI,
        COVERAGE_REVERSE_PID_KD, COVERAGE_REVERSE_PID_MAX_PERCENT);
    if ((requested_correction > 0.0f) != (error > 0.0f))
      requested_correction = 0.0f;
  }

  slew = requested_correction - reverse_correction;
  slew = Coverage_Clamp(slew, -COVERAGE_REVERSE_SLEW_PER_STEP,
                               COVERAGE_REVERSE_SLEW_PER_STEP);
  reverse_correction += slew;
  if (Coverage_Abs(reverse_correction) < 0.25f &&
      requested_correction == 0.0f)
    reverse_correction = 0.0f;

  /* Both commands always remain reverse. Correction only changes their
   * relative speeds, so a noisy yaw sample cannot reverse one wheel. */
  out->left_percent =
      (int16_t)(-COVERAGE_REVERSE_SPEED_PERCENT - reverse_correction);
  out->right_percent =
      (int16_t)(-COVERAGE_REVERSE_SPEED_PERCENT + reverse_correction);
}

static uint8_t Coverage_TimedOut(uint32_t limit_ms)
{
  return (uint32_t)(now_tick - state_tick) >= limit_ms;
}

void Coverage_Reset(void)
{
  state = COVERAGE_IDLE;
  state_tick = now_tick = rear_tick = consumed_rear_tick = 0U;
  rear_mm = 0U;
  rear_close_count = imu_is_fresh = scan_pass = scan_step = 0U;
  segment_start_x = segment_start_y = scan_pass_start_heading = 0.0f;
  scan_settle_tick = 0U;
  turn_test_target_heading = 0.0f;
  Coverage_HeadingPidReset(&turn_heading_pid);
  Coverage_HeadingPidReset(&reverse_heading_pid);
  reverse_filtered_error = 0.0f;
  reverse_correction = 0.0f;
  reverse_filter_valid = 0U;
  reverse_pivot_active = 0U;
}

void Coverage_Start(void)
{
  Coverage_StartReturn(now_tick);
}

void Coverage_StartReturn(uint32_t now_ms)
{
  now_tick = now_ms;
  scan_pass = scan_step = 0U;
  Coverage_Enter(COVERAGE_RETURN_ALIGN_180);
}

void Coverage_StartTurnTest(uint32_t now_ms, float current_heading_rad,
                            float delta_heading_rad)
{
  now_tick = now_ms;
  turn_test_target_heading = Planner_NormalizeAngle(current_heading_rad +
                                                     delta_heading_rad);
  Coverage_Enter(COVERAGE_TURN_TEST);
}

void Coverage_SetInputs(uint32_t rear_distance_mm,
                        uint32_t rear_sample_tick_ms,
                        uint32_t now_ms,
                        uint8_t imu_fresh)
{
  rear_mm = rear_distance_mm;
  rear_tick = rear_sample_tick_ms;
  now_tick = now_ms;
  imu_is_fresh = imu_fresh;
}

uint8_t Coverage_IsActive(void)
{
  return state != COVERAGE_IDLE && state != COVERAGE_FINISHED &&
         state != COVERAGE_FAULT;
}

uint8_t Coverage_Step(void)
{
  return (uint8_t)(scan_pass * COVERAGE_SCAN_STEPS + scan_step);
}

uint8_t Coverage_Total(void)
{
  return (uint8_t)(COVERAGE_SCAN_PASSES * COVERAGE_SCAN_STEPS);
}

uint8_t Coverage_TargetWindow(void)
{
  return state == COVERAGE_SCAN_OBSERVE || state == COVERAGE_SCAN_TURN;
}

CoverageState_t Coverage_State(void)
{
  return state;
}

const char *Coverage_StateName(void)
{
  switch (state)
  {
    case COVERAGE_IDLE:                    return "IDLE";
    case COVERAGE_RETURN_ALIGN_180:        return "RETURN_ALIGN_180";
    case COVERAGE_RETURN_REVERSE_Y:        return "RETURN_REVERSE_Y_150";
    case COVERAGE_RETURN_TURN_CW_90:       return "RETURN_CW90";
    case COVERAGE_RETURN_REVERSE_X:        return "RETURN_REVERSE_X_220";
    case COVERAGE_RETURN_TURN_DIAGONAL:    return "RETURN_CCW_DIAGONAL";
    case COVERAGE_ADVANCE:                 return "ADVANCE_1000";
    case COVERAGE_SCAN_OBSERVE:            return "SCAN_OBSERVE";
    case COVERAGE_SCAN_TURN:               return "SCAN_TURN_60";
    case COVERAGE_UNLOAD_ALIGN_INITIAL:    return "UNLOAD_ALIGN_0";
    case COVERAGE_UNLOAD_REVERSE_Y:        return "UNLOAD_REVERSE_Y_100";
    case COVERAGE_UNLOAD_TURN_CW_90:       return "UNLOAD_CW90";
    case COVERAGE_UNLOAD_REVERSE_X:        return "UNLOAD_REVERSE_X_220";
    case COVERAGE_UNLOAD_TURN_CCW_30:      return "UNLOAD_CCW30";
    case COVERAGE_TURN_TEST:               return "TURN_PID_TEST";
    case COVERAGE_FINISHED:                return "DOCKED";
    case COVERAGE_FAULT:                   return "FAULT";
    default:                               return "UNKNOWN";
  }
}

float Coverage_TurnTestTarget(void)
{
  return turn_test_target_heading;
}

CoverageOutput_t Coverage_Update(const RobotPose_t *pose)
{
  CoverageOutput_t out = {0, 0, Coverage_Step(), Coverage_Total(), 0U, 0U, 0U};
  float dx;
  float dy;
  float distance;
  float progress;

  if (pose == 0 || state == COVERAGE_IDLE) return out;
  if (state == COVERAGE_FINISHED)
  {
    out.finished = 1U;
    return out;
  }
  if (state == COVERAGE_FAULT)
  {
    out.fault = 1U;
    return out;
  }

  /* Absolute turns and every reverse approach require fresh IMU yaw. Stop and
   * allow a short first-frame/recovery window; never drive blind. */
  if ((state == COVERAGE_RETURN_ALIGN_180 ||
       state == COVERAGE_RETURN_REVERSE_Y ||
       state == COVERAGE_RETURN_TURN_CW_90 ||
       state == COVERAGE_RETURN_REVERSE_X ||
       state == COVERAGE_RETURN_TURN_DIAGONAL ||
       state == COVERAGE_UNLOAD_ALIGN_INITIAL ||
       state == COVERAGE_UNLOAD_REVERSE_Y ||
       state == COVERAGE_UNLOAD_TURN_CW_90 ||
       state == COVERAGE_UNLOAD_REVERSE_X ||
       state == COVERAGE_UNLOAD_TURN_CCW_30 ||
       state == COVERAGE_TURN_TEST ||
       state == COVERAGE_SCAN_TURN) && imu_is_fresh == 0U)
  {
    Coverage_HeadingPidReset(&turn_heading_pid);
    Coverage_HeadingPidReset(&reverse_heading_pid);
    if (Coverage_TimedOut(COVERAGE_IMU_WAIT_TIMEOUT_MS))
    {
      state = COVERAGE_FAULT;
      out.fault = 1U;
    }
    return out;
  }

  switch (state)
  {
    case COVERAGE_RETURN_ALIGN_180:
      if (Coverage_Align(pose->heading_rad, COVERAGE_OPPOSITE_HEADING_RAD, &out))
        Coverage_Enter(COVERAGE_RETURN_REVERSE_Y);
      else if (Coverage_TimedOut(COVERAGE_ALIGN_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      break;

    case COVERAGE_RETURN_REVERSE_Y:
      if (Coverage_RearReached(COVERAGE_RETURN_REAR_Y_MM))
        Coverage_Enter(COVERAGE_RETURN_TURN_CW_90);
      else if (Coverage_TimedOut(COVERAGE_REVERSE_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      else Coverage_ReverseHold(pose->heading_rad,
                                COVERAGE_OPPOSITE_HEADING_RAD, &out);
      break;

    case COVERAGE_RETURN_TURN_CW_90:
      if (Coverage_Align(pose->heading_rad, COVERAGE_POSITIVE_X_HEADING_RAD, &out))
        Coverage_Enter(COVERAGE_RETURN_REVERSE_X);
      else if (Coverage_TimedOut(COVERAGE_ALIGN_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      break;

    case COVERAGE_RETURN_REVERSE_X:
      if (Coverage_RearReached(COVERAGE_RETURN_REAR_X_MM))
        Coverage_Enter(COVERAGE_RETURN_TURN_DIAGONAL);
      else if (Coverage_TimedOut(COVERAGE_REVERSE_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      else Coverage_ReverseHold(pose->heading_rad,
                                COVERAGE_POSITIVE_X_HEADING_RAD, &out);
      break;

    case COVERAGE_RETURN_TURN_DIAGONAL:
      if (Coverage_Align(pose->heading_rad, COVERAGE_DIAGONAL_HEADING_RAD, &out))
      {
        segment_start_x = pose->x_mm;
        segment_start_y = pose->y_mm;
        Coverage_Enter(COVERAGE_ADVANCE);
      }
      else if (Coverage_TimedOut(COVERAGE_ALIGN_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      break;

    case COVERAGE_ADVANCE:
      dx = pose->x_mm - segment_start_x;
      dy = pose->y_mm - segment_start_y;
      distance = Planner_Sqrt(dx * dx + dy * dy);
      progress = dx * Planner_Cos(COVERAGE_DIAGONAL_HEADING_RAD) +
                 dy * Planner_Sin(COVERAGE_DIAGONAL_HEADING_RAD);
      if (progress >= COVERAGE_ADVANCE_MM && distance >= COVERAGE_ADVANCE_MM)
      {
        scan_step = 0U;
        /* Anchor all six headings to one IMU-derived pass origin. The final
         * 360-degree target is therefore the original 0-degree heading and
         * cannot accumulate six separate wheel-slip errors. */
        scan_pass_start_heading = pose->heading_rad;
        Coverage_Enter(COVERAGE_SCAN_OBSERVE);
      }
      else if (Coverage_TimedOut(COVERAGE_ADVANCE_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      else out.left_percent = out.right_percent = COVERAGE_ADVANCE_SPEED_PERCENT;
      break;

    case COVERAGE_SCAN_OBSERVE:
      out.target_window = 1U;
      if (Coverage_TimedOut(COVERAGE_SCAN_OBSERVE_MS))
      {
        if (scan_step >= COVERAGE_SCAN_STEPS)
        {
          scan_pass++;
          if (scan_pass >= COVERAGE_SCAN_PASSES)
            Coverage_Enter(COVERAGE_UNLOAD_ALIGN_INITIAL);
          else
          {
            segment_start_x = pose->x_mm;
            segment_start_y = pose->y_mm;
            Coverage_Enter(COVERAGE_ADVANCE);
          }
        }
        else
        {
          Coverage_Enter(COVERAGE_SCAN_TURN);
        }
      }
      break;

    case COVERAGE_SCAN_TURN:
      out.target_window = 1U;
      if (Coverage_Align(pose->heading_rad,
                         Planner_NormalizeAngle(scan_pass_start_heading +
                           (float)(scan_step + 1U) * COVERAGE_SCAN_STEP_RAD),
                         &out))
      {
        if (scan_settle_tick == 0U) scan_settle_tick = now_tick;
        if ((uint32_t)(now_tick - scan_settle_tick) >=
            COVERAGE_SCAN_SETTLE_MS)
        {
          scan_step++;
          Coverage_Enter(COVERAGE_SCAN_OBSERVE);
        }
      }
      else
      {
        scan_settle_tick = 0U;
        if (Coverage_TimedOut(COVERAGE_SCAN_TURN_TIMEOUT_MS))
          Coverage_Enter(COVERAGE_FAULT);
      }
      break;

    case COVERAGE_UNLOAD_ALIGN_INITIAL:
      if (Coverage_Align(pose->heading_rad, COVERAGE_INITIAL_HEADING_RAD, &out))
        Coverage_Enter(COVERAGE_UNLOAD_REVERSE_Y);
      else if (Coverage_TimedOut(COVERAGE_ALIGN_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      break;

    case COVERAGE_UNLOAD_REVERSE_Y:
      if (Coverage_RearReached(COVERAGE_UNLOAD_REAR_Y_MM))
        Coverage_Enter(COVERAGE_UNLOAD_TURN_CW_90);
      else if (Coverage_TimedOut(COVERAGE_REVERSE_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      else Coverage_ReverseHold(pose->heading_rad,
                                COVERAGE_INITIAL_HEADING_RAD, &out);
      break;

    case COVERAGE_UNLOAD_TURN_CW_90:
      if (Coverage_Align(pose->heading_rad, COVERAGE_UNLOAD_X_HEADING_RAD, &out))
        Coverage_Enter(COVERAGE_UNLOAD_REVERSE_X);
      else if (Coverage_TimedOut(COVERAGE_ALIGN_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      break;

    case COVERAGE_UNLOAD_REVERSE_X:
      if (Coverage_RearReached(COVERAGE_UNLOAD_REAR_X_MM))
        Coverage_Enter(COVERAGE_UNLOAD_TURN_CCW_30);
      else if (Coverage_TimedOut(COVERAGE_REVERSE_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      else Coverage_ReverseHold(pose->heading_rad,
                                COVERAGE_UNLOAD_X_HEADING_RAD, &out);
      break;

    case COVERAGE_UNLOAD_TURN_CCW_30:
      if (Coverage_Align(pose->heading_rad, COVERAGE_UNLOAD_FINAL_HEADING_RAD, &out))
        Coverage_Enter(COVERAGE_FINISHED);
      else if (Coverage_TimedOut(COVERAGE_ALIGN_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      break;

    case COVERAGE_TURN_TEST:
      if (Coverage_Align(pose->heading_rad, turn_test_target_heading, &out))
        Coverage_Enter(COVERAGE_FINISHED);
      else if (Coverage_TimedOut(COVERAGE_ALIGN_TIMEOUT_MS))
        Coverage_Enter(COVERAGE_FAULT);
      break;

    default:
      Coverage_Enter(COVERAGE_FAULT);
      break;
  }

  out.step = Coverage_Step();
  out.total = Coverage_Total();
  out.target_window = Coverage_TargetWindow();
  out.finished = state == COVERAGE_FINISHED;
  out.fault = state == COVERAGE_FAULT;
  return out;
}
