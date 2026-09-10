#ifndef IMU_NAVIGATION_H
#define IMU_NAVIGATION_H

#include <stdint.h>

#include "jy901.h"

/* JY901 yaw must increase for a counter-clockwise vehicle rotation. Change
 * this to -1 only if the stationary I test proves that the sign is opposite. */
#define IMU_NAV_YAW_SIGN             1.0f
#define IMU_NAV_STALE_MS             500U
#define IMU_NAV_BLEND                0.65f
/* A checksum-valid packet is still not automatically trustworthy.  Normal
 * updates must respect both a physical yaw-rate limit and this absolute cap.
 * JY901 angle raw uses 32768 counts for 180 degrees. */
#define IMU_NAV_MAX_STEP_RAW         4551 /* About 25 degrees absolute cap. */
#define IMU_NAV_STEP_MARGIN_RAW       546 /* About 3 degrees of timing margin. */
#define IMU_NAV_RATE_RAW_PER_MS        33 /* About 181 degrees/second. */

/* After a stale link or an implausible jump, hold the last accepted heading.
 * Recovery requires several mutually consistent frames and agreement with
 * short-term encoder odometry.  A persistent wrong 90-degree value therefore
 * cannot become valid merely because it was repeated twice. */
#define IMU_NAV_RECOVERY_REQUIRED_FRAMES          5U
#define IMU_NAV_RECOVERY_MAX_FRAME_STEP_RAW     2731 /* About 15 degrees. */
#define IMU_NAV_RECOVERY_ENCODER_TOLERANCE_RAD 0.523599f /* 30 degrees. */

typedef struct
{
  uint8_t valid;
  uint8_t reference_valid;
  uint8_t recovery_active;
  uint8_t recovery_samples;
  int16_t reference_yaw_raw;
  int16_t last_yaw_raw;
  int16_t recovery_last_yaw_raw;
  uint32_t angle_frames;
  uint32_t observed_angle_frames;
  uint32_t last_tick;
  uint32_t observed_tick;
  uint32_t rejected_steps;
  uint32_t stale_recoveries;
  uint32_t recovery_accepts;
  uint32_t recovery_resets;
  float reference_heading_rad;
  float heading_rad;
} ImuNavigation_t;

void ImuNavigation_Reset(ImuNavigation_t *navigation);
/* Lock a fresh raw-yaw frame to a known software heading.  Greedy mode calls
 * this only after the stationary startup samples have been confirmed. */
uint8_t ImuNavigation_SetReference(ImuNavigation_t *navigation,
                                   const JY901Snapshot *snapshot,
                                   uint32_t now_ms,
                                   float heading_rad);
/* Returns 1 only when a new angle frame was consumed.  Heading is always
 * calculated from the preserved raw-yaw reference.  A stale recovery or an
 * implausible one-frame step never overwrites that reference with odometry. */
uint8_t ImuNavigation_Update(ImuNavigation_t *navigation,
                             const JY901Snapshot *snapshot,
                             uint32_t now_ms,
                             float encoder_heading_rad);
uint8_t ImuNavigation_Fresh(const ImuNavigation_t *navigation,
                            uint32_t now_ms);

#endif /* IMU_NAVIGATION_H */
