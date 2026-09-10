#ifndef COMBAT_STRATEGY_H
#define COMBAT_STRATEGY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "path_planner.h"
#include "target_map.h"

/* Five-minute adversarial challenge. All coordinates are millimetres in the
 * arena frame defined by mission_extension.h. */
#define COMBAT_MATCH_DURATION_MS          300000U
/* Reserve the final minute exclusively for returning and unloading. */
#define COMBAT_ENDGAME_RETURN_MS          240000U
#define COMBAT_MAX_OPPONENT_DWELL_MS       25000U
/* Kept for telemetry/backward compatibility.  The Bluetooth combat profile
 * deliberately does not return early by count or deposit interval. */
#define COMBAT_DEPOSIT_INTERVAL_MS         60000U
#define COMBAT_BATCH_SIZE_NORMAL               4U
#define COMBAT_BATCH_SIZE_LATE                 2U

/* Own warehouse follows the lower-left zone in the supplied field drawing.
 * The opposing warehouse is assumed symmetric in the upper-right corner.
 * Change these four values when the official field drawing is available. */
#define COMBAT_OWN_ZONE_MAX_X_MM           350.0f
#define COMBAT_OWN_ZONE_MAX_Y_MM           400.0f
#define COMBAT_OPP_ZONE_MIN_X_MM           2650.0f
#define COMBAT_OPP_ZONE_MIN_Y_MM           1600.0f

typedef struct
{
  uint16_t target_id;
  float score;
  uint8_t valid;
  uint8_t opponent_warehouse_target;
} CombatSelection_t;

void CombatStrategy_Reset(void);
void CombatStrategy_Start(uint32_t now_ms);
void CombatStrategy_Stop(void);
uint8_t CombatStrategy_IsActive(void);

void CombatStrategy_UpdateZone(const RobotPose_t *pose, uint32_t now_ms);
CombatSelection_t CombatStrategy_SelectTarget(const RobotPose_t *pose,
                                               const MapTarget_t *candidates,
                                               uint8_t candidate_count,
                                               uint32_t now_ms);

void CombatStrategy_RecordCollected(uint32_t now_ms);
void CombatStrategy_RecordDeposit(uint32_t now_ms);
uint8_t CombatStrategy_GetPayloadCount(void);
uint8_t CombatStrategy_ShouldReturn(uint32_t now_ms);
uint8_t CombatStrategy_MatchFinished(uint32_t now_ms);
uint32_t CombatStrategy_ElapsedMs(uint32_t now_ms);
uint32_t CombatStrategy_OpponentDwellMs(uint32_t now_ms);

/* Search waypoints implement a rolling regional sweep when the camera map is
 * temporarily empty. The route deliberately visits the opposing warehouse
 * approach but forces an exit before the 30-second rule. */
void CombatStrategy_GetPatrolWaypoint(float *x_mm, float *y_mm,
                                      uint32_t now_ms);
void CombatStrategy_AdvancePatrol(void);

const char *CombatStrategy_PhaseName(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* COMBAT_STRATEGY_H */
