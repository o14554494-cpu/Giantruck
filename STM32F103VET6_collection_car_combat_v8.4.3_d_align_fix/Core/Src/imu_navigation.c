#include "imu_navigation.h"

#include "path_planner.h"

#define IMU_RAW_TO_RAD (3.14159265358979323846f / 32768.0f)

static int32_t ImuNavigation_Abs32(int32_t value)
{
  return value < 0 ? -value : value;
}

void ImuNavigation_Reset(ImuNavigation_t *navigation)
{
  if (navigation == 0) return;
  navigation->valid = 0U;
  navigation->reference_valid = 0U;
  navigation->reference_yaw_raw = 0;
  navigation->last_yaw_raw = 0;
  navigation->angle_frames = 0U;
  navigation->last_tick = 0U;
  navigation->rejected_steps = 0U;
  navigation->stale_recoveries = 0U;
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
  navigation->angle_frames = snapshot->angle_frames;
  navigation->last_tick = snapshot->angle_tick;
  navigation->rejected_steps = 0U;
  navigation->stale_recoveries = 0U;
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
  uint8_t stale_gap;
  float relative_heading;
  if ((navigation == 0) || (snapshot == 0) ||
      (snapshot->angle_frames == 0U) ||
      ((uint32_t)(now_ms - snapshot->angle_tick) > IMU_NAV_STALE_MS) ||
      (snapshot->angle_frames == navigation->angle_frames))
    return 0U;

  stale_gap = navigation->valid &&
      ((uint32_t)(now_ms - navigation->last_tick) > IMU_NAV_STALE_MS);
  if (!navigation->valid || navigation->reference_valid == 0U)
  {
    return ImuNavigation_SetReference(navigation, snapshot, now_ms,
                                      encoder_heading_rad);
  }

  delta_raw = JY901_RelativeYawRaw(snapshot->angle[2],
                                   navigation->last_yaw_raw);
  navigation->last_yaw_raw = snapshot->angle[2];
  navigation->angle_frames = snapshot->angle_frames;
  navigation->last_tick = snapshot->angle_tick;
  if (!stale_gap && ImuNavigation_Abs32(delta_raw) > IMU_NAV_MAX_STEP_RAW)
  {
    navigation->rejected_steps++;
    /* Keep the last trustworthy heading and, crucially, the startup yaw
     * reference.  A later normal frame is evaluated against that reference. */
    return 1U;
  }

  if (stale_gap) navigation->stale_recoveries++;
  relative_heading = IMU_NAV_YAW_SIGN *
      (float)JY901_RelativeYawRaw(snapshot->angle[2],
                                 navigation->reference_yaw_raw) *
      IMU_RAW_TO_RAD;
  navigation->heading_rad = Planner_NormalizeAngle(
      navigation->reference_heading_rad + relative_heading);
  return 1U;
}

uint8_t ImuNavigation_Fresh(const ImuNavigation_t *navigation,
                            uint32_t now_ms)
{
  return navigation != 0 && navigation->valid &&
      (uint32_t)(now_ms - navigation->last_tick) <= IMU_NAV_STALE_MS;
}
