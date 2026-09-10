#ifndef MISSION_EXTENSION_H
#define MISSION_EXTENSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "path_planner.h"

/*
 * Arena coordinate convention (millimetres):
 *   origin: lower-left inside corner in the supplied field drawing
 *   +X: toward the right-hand wall (3000 mm side)
 *   +Y: toward the upper wall (2000 mm side)
 *
 * The start pose must match the real placement of the car.  These defaults
 * follow the schematic and are calibration values, not measurements.
 */
#define MISSION_ARENA_WIDTH_MM             3000.0f
#define MISSION_ARENA_HEIGHT_MM            2000.0f
#define MISSION_TARGET_MARGIN_MM             80.0f
#define MISSION_ROBOT_WALL_MARGIN_MM        100.0f

/* Upper-right unloading approach from the supplied yellow route. The black
 * zone was not dimensioned: stage/stop positions must be measured on site. */
#define MISSION_UNLOAD_STAGE_X_MM           650.0f
#define MISSION_UNLOAD_STAGE_Y_MM           200.0f
#define MISSION_UNLOAD_STOP_X_MM            180.0f
#define MISSION_UNLOAD_HEADING_RAD             0.0f
#define MISSION_Q_UNLOAD_STAGE_X_MM        2500.0f
#define MISSION_Q_UNLOAD_STAGE_Y_MM        1450.0f
#define MISSION_Q_UNLOAD_STOP_Y_MM         1750.0f
#define MISSION_Q_UNLOAD_HEADING_RAD       (-1.57079632679f)

/* The vehicle is physically placed in the upper-right unloading area before
 * a new run. Q resets its software pose to this measured/calibrated waiting
 * point; heading -90 degrees means the nose points down into the arena and
 * the rear points toward the unloading wall. */
#define MISSION_START_X_MM                  MISSION_Q_UNLOAD_STAGE_X_MM
#define MISSION_START_Y_MM                  MISSION_Q_UNLOAD_STAGE_Y_MM
#define MISSION_START_HEADING_RAD           MISSION_Q_UNLOAD_HEADING_RAD

#define MISSION_FRONT_STOP_MM                100U
#define MISSION_REAR_STOP_MM                 130U

typedef enum
{
  MISSION_UNLOAD_IDLE = 0,
  MISSION_UNLOAD_GO_STAGE,
  MISSION_UNLOAD_ALIGN,
  MISSION_UNLOAD_REVERSE,
  MISSION_UNLOAD_EJECT,
  MISSION_UNLOAD_DONE,
  MISSION_UNLOAD_FAULT
} MissionUnloadState_t;

typedef struct
{
  int16_t left_percent;
  int16_t right_percent;
  int16_t collector_pwm; /* Reserved, always 0. Rear servo handles unloading;
                         * mission outputs do not control the front brush. */
  uint8_t active;
  uint8_t finished;
} MissionDriveOutput_t;

void MissionExtension_Reset(void);
/* Cancel unloading and reactive avoidance without losing the payload count. */
void MissionExtension_CancelMotion(void);
void MissionExtension_SetInitialPose(RobotPose_t *pose);
uint8_t MissionExtension_TargetInsideArena(float x_mm, float y_mm);

void MissionExtension_RecordCollected(void);
uint8_t MissionExtension_HasPayload(void);

void MissionExtension_StartUnload(uint32_t now_ms);
void MissionExtension_ForceStartUnload(uint32_t now_ms);
/* Q coverage has already docked with rear ultrasonic and IMU. Skip the old
 * coordinate GO_STAGE/REVERSE states and start the hatch+shake sequence. */
void MissionExtension_ForceStartDockedEject(uint32_t now_ms);
void MissionExtension_UseUpperRightUnload(void);
MissionDriveOutput_t MissionExtension_UpdateUnload(const RobotPose_t *pose,
                                                   uint32_t rear_distance_mm,
                                                   uint32_t now_ms);
MissionUnloadState_t MissionExtension_GetUnloadState(void);
const char *MissionExtension_UnloadStateName(void);

/*
 * Reactive wall/obstacle guard. Call this only during ordinary forward
 * navigation, not during final target alignment/collection (the target itself
 * would otherwise look like an obstacle to a single ultrasonic sensor).
 */
uint8_t MissionExtension_ApplyForwardSafety(const RobotPose_t *pose,
                                            uint32_t front_distance_mm,
                                            uint32_t now_ms,
                                            int16_t requested_left,
                                            int16_t requested_right,
                                            int16_t *safe_left,
                                            int16_t *safe_right,
                                            uint8_t *request_replan);

#ifdef __cplusplus
}
#endif

#endif /* MISSION_EXTENSION_H */
