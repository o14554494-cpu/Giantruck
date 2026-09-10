#include "imu_navigation.h"

#include "path_planner.h"

#define IMU_RAW_TO_RAD (3.14159265358979323846f / 32768.0f)

static int32_t ImuNavigation_Abs32(int32_t value)
{
  return value < 0 ? -value : value;
}

static float ImuNavigation_AbsFloat(float value)
{
  return value < 0.0f ? -value : value;
}

static int32_t ImuNavigation_MaxStepRaw(uint32_t elapsed_ms)
{
  uint32_t limit = IMU_NAV_STEP_MARGIN_RAW;
  if (elapsed_ms > IMU_NAV_STALE_MS) elapsed_ms = IMU_NAV_STALE_MS;
  limit += elapsed_ms * IMU_NAV_RATE_RAW_PER_MS;
  if (limit > IMU_NAV_MAX_STEP_RAW) limit = IMU_NAV_MAX_STEP_RAW;
  return (int32_t)limit;
}

static float ImuNavigation_CandidateHeading(const ImuNavigation_t *navigation,
                                            int16_t yaw_raw)
{
  float relative_heading = IMU_NAV_YAW_SIGN *
      (float)JY901_RelativeYawRaw(yaw_raw,
                                 navigation->reference_yaw_raw) *
      IMU_RAW_TO_RAD;
  return Planner_NormalizeAngle(navigation->reference_heading_rad +
                                relative_heading);
}

static void ImuNavigation_Accept(ImuNavigation_t *navigation,
                                 const JY901Snapshot *snapshot,
                                 float candidate_heading)
{
  navigation->last_yaw_raw = snapshot->angle[2];
  navigation->angle_frames = snapshot->angle_frames;
  navigation->last_tick = snapshot->angle_tick;
  navigation->heading_rad = candidate_heading;
  navigation->recovery_active = 0U;
  navigation->recovery_samples = 0U;
}

void ImuNavigation_Reset(ImuNavigation_t *navigation)
{
  if (navigation == 0) return;
  navigation->valid = 0U;
  navigation->reference_valid = 0U;
  navigation->recovery_active = 0U;
  navigation->recovery_samples = 0U;
  navigation->reference_yaw_raw = 0;
  navigation->last_yaw_raw = 0;
  navigation->recovery_last_yaw_raw = 0;
  navigation->angle_frames = 0U;
  navigation->observed_angle_frames = 0U;
  navigation->last_tick = 0U;
  navigation->observed_tick = 0U;
  navigation->rejected_steps = 0U;
  navigation->stale_recoveries = 0U;
  navigation->recovery_accepts = 0U;
  navigation->recovery_resets = 0U;
  navigation->reference_heading_rad = 0.0f;
  navigation->heading_rad = 0.0f;
}

uint8_t ImuNavigation_SetReference(ImuNavigation_t *navigation,
                                   const JY901Snapshot *snapshot,
                                   uint32_t now_ms,
                                   float heading_rad)
{
  if ((navigation == 0) || (snapshot == 0) ||
      (snapshot->angle_frames == 0U) ||
      ((uint32_t)(now_ms - snapshot->angle_tick) > IMU_NAV_STALE_MS))
    return 0U;

  navigation->valid = 1U;
  navigation->reference_valid = 1U;
  navigation->reference_yaw_raw = snapshot->angle[2];
  navigation->last_yaw_raw = snapshot->angle[2];
  navigation->recovery_last_yaw_raw = snapshot->angle[2];
  navigation->angle_frames = snapshot->angle_frames;
  navigation->observed_angle_frames = snapshot->angle_frames;
  navigation->last_tick = snapshot->angle_tick;
  navigation->observed_tick = snapshot->angle_tick;
  navigation->recovery_active = 0U;
  navigation->recovery_samples = 0U;
  navigation->rejected_steps = 0U;
  navigation->stale_recoveries = 0U;
  navigation->recovery_accepts = 0U;
  navigation->recovery_resets = 0U;
  navigation->reference_heading_rad = Planner_NormalizeAngle(heading_rad);
  navigation->heading_rad = navigation->reference_heading_rad;
  return 1U;
}

uint8_t ImuNavigation_Update(ImuNavigation_t *navigation,
                             const JY901Snapshot *snapshot,
                             uint32_t now_ms,
                             float encoder_heading_rad)
{
  int32_t delta_raw;
  int32_t recovery_delta_raw;
  int32_t max_step_raw;
  uint32_t elapsed_ms;
  uint8_t stale_gap;
  float candidate_heading;
  float encoder_error;
  if ((navigation == 0) || (snapshot == 0) ||
      (snapshot->angle_frames == 0U) ||
      ((uint32_t)(now_ms - snapshot->angle_tick) > IMU_NAV_STALE_MS) ||
      (snapshot->angle_frames == navigation->observed_angle_frames))
    return 0U;

  if (!navigation->valid || navigation->reference_valid == 0U)
  {
    return ImuNavigation_SetReference(navigation, snapshot, now_ms,
                                      encoder_heading_rad);
  }

  /* Mark the source frame as consumed before filtering it, but do not touch
   * last_yaw_raw, angle_frames, last_tick or heading_rad until it is accepted.
   * This prevents a repeated bad value from legitimising itself on frame 2. */
  navigation->observed_angle_frames = snapshot->angle_frames;
  navigation->observed_tick = snapshot->angle_tick;
  elapsed_ms = (uint32_t)(snapshot->angle_tick - navigation->last_tick);
  stale_gap = elapsed_ms > IMU_NAV_STALE_MS;
  delta_raw = JY901_RelativeYawRaw(snapshot->angle[2],
                                   navigation->last_yaw_raw);
  max_step_raw = ImuNavigation_MaxStepRaw(elapsed_ms);
  candidate_heading = ImuNavigation_CandidateHeading(navigation,
                                                      snapshot->angle[2]);

  if (!stale_gap && ImuNavigation_Abs32(delta_raw) <= max_step_raw)
  {
    ImuNavigation_Accept(navigation, snapshot, candidate_heading);
    return 1U;
  }

  navigation->rejected_steps++;
  encoder_error = ImuNavigation_AbsFloat(Planner_NormalizeAngle(
      candidate_heading - encoder_heading_rad));

  if (navigation->recovery_active == 0U)
  {
    navigation->recovery_active = 1U;
    navigation->recovery_samples = 1U;
    navigation->recovery_last_yaw_raw = snapshot->angle[2];
    if (stale_gap) navigation->stale_recoveries++;
    return 1U;
  }

  recovery_delta_raw = JY901_RelativeYawRaw(
      snapshot->angle[2], navigation->recovery_last_yaw_raw);
  navigation->recovery_last_yaw_raw = snapshot->angle[2];
  if (ImuNavigation_Abs32(recovery_delta_raw) >
          IMU_NAV_RECOVERY_MAX_FRAME_STEP_RAW ||
      encoder_error > IMU_NAV_RECOVERY_ENCODER_TOLERANCE_RAD)
  {
    navigation->recovery_samples = 1U;
    navigation->recovery_resets++;
    return 1U;
  }

  if (navigation->recovery_samples < IMU_NAV_RECOVERY_REQUIRED_FRAMES)
    navigation->recovery_samples++;
  if (navigation->recovery_samples < IMU_NAV_RECOVERY_REQUIRED_FRAMES)
    return 1U;

  navigation->recovery_accepts++;
  ImuNavigation_Accept(navigation, snapshot, candidate_heading);
  return 1U;
}

uint8_t ImuNavigation_Fresh(const ImuNavigation_t *navigation,
                            uint32_t now_ms)
{
  return navigation != 0 && navigation->valid &&
      navigation->recovery_active == 0U &&
      (uint32_t)(now_ms - navigation->last_tick) <= IMU_NAV_STALE_MS;
}
