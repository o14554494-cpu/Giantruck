#ifndef IMU_NAVIGATION_H
#define IMU_NAVIGATION_H

#include <stdint.h>

#include "jy901.h"

/* JY901 yaw must increase for a counter-clockwise vehicle rotation. Change
 * this to -1 only if the stationary I test proves that the sign is opposite. */
#define IMU_NAV_YAW_SIGN             1.0f
#define IMU_NAV_STALE_MS             500U
#define IMU_NAV_BLEND                0.65f
#define IMU_NAV_MAX_STEP_RAW         8192 /* 45 degrees between accepted frames */

typedef struct
{
  uint8_t valid;
  uint8_t reference_valid;
  int16_t reference_yaw_raw;
  int16_t last_yaw_raw;
  uint32_t angle_frames;
  uint32_t last_tick;
  uint32_t rejected_steps;
  uint32_t stale_recoveries;
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
