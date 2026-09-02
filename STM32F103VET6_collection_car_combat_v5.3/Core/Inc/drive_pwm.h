#ifndef DRIVE_PWM_H
#define DRIVE_PWM_H

#include <stdint.h>

/* Wheel-motor dead-zone compensation, NOT the collector PWM.
 * Logical commands 1..100 map to physical duty 60..100%; zero stays off.
 * Retune on the real vehicle; an open-loop command is not a measured speed. */
#define MOTOR_MIN_DRIVE_PWM_PERCENT 60

#if (MOTOR_MIN_DRIVE_PWM_PERCENT < 1) || (MOTOR_MIN_DRIVE_PWM_PERCENT > 100)
#error "MOTOR_MIN_DRIVE_PWM_PERCENT must be in 1..100"
#endif

static inline int16_t Motor_MapDrivePercent(int16_t command)
{
  int32_t magnitude = command;
  int16_t duty;
  if (magnitude == 0) return 0;
  if (magnitude < 0) magnitude = -magnitude;
  if (magnitude > 100) magnitude = 100;
  duty = (int16_t)(MOTOR_MIN_DRIVE_PWM_PERCENT +
      ((magnitude - 1) * (100 - MOTOR_MIN_DRIVE_PWM_PERCENT) + 49) / 99);
  return command < 0 ? (int16_t)-duty : duty;
}

#endif
