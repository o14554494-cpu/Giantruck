#include "combat_strategy.h"

#include <stddef.h>

#include "mission_extension.h"

typedef struct
{
  float x_mm;
  float y_mm;
} CombatWaypoint_t;

static const CombatWaypoint_t patrol_waypoints[] =
{
  { 850.0f,  550.0f},
  {1500.0f,  500.0f},
  {2300.0f,  550.0f},
  {2350.0f, 1100.0f},
  {2350.0f, 1500.0f}, /* opposing warehouse approach */
  {2720.0f, 1750.0f}, /* short legal steal scan */
  {1800.0f, 1500.0f},
  {1000.0f, 1500.0f},
  { 550.0f, 1000.0f}
};

#define COMBAT_PATROL_COUNT \
  ((uint8_t)(sizeof(patrol_waypoints) / sizeof(patrol_waypoints[0])))

static uint8_t combat_active;
static uint32_t combat_start_tick;
static uint32_t last_deposit_tick;
static uint8_t payload_count;
static uint8_t patrol_index;

static uint8_t inside_opponent_zone;
static uint8_t opponent_exit_required;
static uint32_t opponent_entry_tick;
static uint32_t opponent_dwell_ms;

static uint8_t Combat_InOpponentZone(float x_mm, float y_mm)
{
  return (x_mm >= COMBAT_OPP_ZONE_MIN_X_MM) &&
         (y_mm >= COMBAT_OPP_ZONE_MIN_Y_MM);
}

static uint8_t Combat_InOwnZone(float x_mm, float y_mm)
{
  return (x_mm <= COMBAT_OWN_ZONE_MAX_X_MM) &&
         (y_mm <= COMBAT_OWN_ZONE_MAX_Y_MM);
}

static float Combat_Distance(float x1, float y1, float x2, float y2)
{
  float dx = x2 - x1;
  float dy = y2 - y1;
  return Planner_Sqrt(dx * dx + dy * dy);
}

void CombatStrategy_Reset(void)
{
  combat_active = 0U;
  combat_start_tick = 0U;
  last_deposit_tick = 0U;
  payload_count = 0U;
  patrol_index = 0U;
  inside_opponent_zone = 0U;
  opponent_exit_required = 0U;
  opponent_entry_tick = 0U;
  opponent_dwell_ms = 0U;
}

void CombatStrategy_Start(uint32_t now_ms)
{
  CombatStrategy_Reset();
  combat_active = 1U;
  combat_start_tick = now_ms;
  last_deposit_tick = now_ms;
}

void CombatStrategy_Stop(void)
{
  CombatStrategy_Reset();
}

uint8_t CombatStrategy_IsActive(void)
{
  return combat_active;
}

uint32_t CombatStrategy_ElapsedMs(uint32_t now_ms)
{
  return combat_active != 0U ? now_ms - combat_start_tick : 0U;
}

uint8_t CombatStrategy_MatchFinished(uint32_t now_ms)
{
  return (combat_active != 0U) &&
         (CombatStrategy_ElapsedMs(now_ms) >= COMBAT_MATCH_DURATION_MS);
}

void CombatStrategy_UpdateZone(const RobotPose_t *pose, uint32_t now_ms)
{
  uint8_t inside;
  if ((combat_active == 0U) || (pose == NULL)) return;

  inside = Combat_InOpponentZone(pose->x_mm, pose->y_mm);
  if ((inside != 0U) && (inside_opponent_zone == 0U))
  {
    inside_opponent_zone = 1U;
    opponent_entry_tick = now_ms;
    opponent_dwell_ms = 0U;
  }
  else if (inside != 0U)
  {
    opponent_dwell_ms = now_ms - opponent_entry_tick;
    if (opponent_dwell_ms >= COMBAT_MAX_OPPONENT_DWELL_MS)
    {
      opponent_exit_required = 1U;
    }
  }
  else if (inside_opponent_zone != 0U)
  {
    inside_opponent_zone = 0U;
    opponent_dwell_ms = 0U;
    opponent_exit_required = 0U;
  }
}

uint32_t CombatStrategy_OpponentDwellMs(uint32_t now_ms)
{
  if ((inside_opponent_zone != 0U) && (combat_active != 0U))
  {
    return now_ms - opponent_entry_tick;
  }
  return opponent_dwell_ms;
}

CombatSelection_t CombatStrategy_SelectTarget(const RobotPose_t *pose,
                                               const MapTarget_t *candidates,
                                               uint8_t candidate_count,
                                               uint32_t now_ms)
{
  CombatSelection_t best = {0U, 0.0f, 0U, 0U};
  uint8_t index;

  if ((combat_active == 0U) || (pose == NULL) || (candidates == NULL))
  {
    return best;
  }

  for (index = 0U; index < candidate_count; index++)
  {
    const MapTarget_t *target = &candidates[index];
    float travel;
    float return_distance;
    float dx;
    float dy;
    float forward_projection;
    float heading_penalty;
    float cluster_bonus = 0.0f;
    float score;
    uint8_t other;
    uint8_t opponent_target;
    uint32_t elapsed = CombatStrategy_ElapsedMs(now_ms);
    float steal_bonus;

    opponent_target = Combat_InOpponentZone(target->x_mm, target->y_mm);
    /* Never plan to collect our own already-scored warehouse contents. */
    if (Combat_InOwnZone(target->x_mm, target->y_mm) != 0U)
    {
      continue;
    }
    if ((opponent_target != 0U) && (opponent_exit_required != 0U))
    {
      continue;
    }

    travel = Combat_Distance(pose->x_mm, pose->y_mm,
                             target->x_mm, target->y_mm);
    return_distance = Combat_Distance(target->x_mm, target->y_mm,
                                      MISSION_UNLOAD_STAGE_X_MM,
                                      MISSION_UNLOAD_STAGE_Y_MM);
    /* In the last 30 seconds, stay close enough to bank the target and avoid
     * abandoning our warehouse to chase a distant steal. */
    if ((elapsed >= COMBAT_ENDGAME_RETURN_MS) && (return_distance > 900.0f))
    {
      continue;
    }
    dx = target->x_mm - pose->x_mm;
    dy = target->y_mm - pose->y_mm;
    forward_projection = Planner_Cos(pose->heading_rad) * dx +
                         Planner_Sin(pose->heading_rad) * dy;
    heading_penalty = forward_projection >= 0.0f ? 0.0f : 260.0f;

    for (other = 0U; other < candidate_count; other++)
    {
      if ((other != index) &&
          (Combat_InOwnZone(candidates[other].x_mm,
                            candidates[other].y_mm) == 0U) &&
          (Combat_Distance(target->x_mm, target->y_mm,
                           candidates[other].x_mm,
                           candidates[other].y_mm) <= 420.0f))
      {
        cluster_bonus += 110.0f;
      }
    }
    if (cluster_bonus > 330.0f) cluster_bonus = 330.0f;

    /* Lower is better. Returning distance matters more as the hopper fills.
     * A warehouse steal is approximately double-value: it adds one to us and
     * can remove one from the opponent, hence the bounded tactical bonus. */
    steal_bonus = elapsed < 60000U ? 180.0f : 520.0f;
    score = travel + heading_penalty +
            return_distance * (payload_count >= 2U ? 0.55f : 0.20f) -
            (float)target->quality * 2.0f - cluster_bonus -
            (opponent_target != 0U ? steal_bonus : 0.0f);

    if ((best.valid == 0U) || (score < best.score))
    {
      best.target_id = target->id;
      best.score = score;
      best.valid = 1U;
      best.opponent_warehouse_target = opponent_target;
    }
  }
  return best;
}

void CombatStrategy_RecordCollected(uint32_t now_ms)
{
  (void)now_ms;
  if ((combat_active != 0U) && (payload_count < 255U)) payload_count++;
}

void CombatStrategy_RecordDeposit(uint32_t now_ms)
{
  payload_count = 0U;
  last_deposit_tick = now_ms;
}

uint8_t CombatStrategy_GetPayloadCount(void)
{
  return payload_count;
}

uint8_t CombatStrategy_ShouldReturn(uint32_t now_ms)
{
  if (combat_active == 0U) return 0U;
  /* This profile banks once: collection/patrol owns the first four minutes,
   * then the black-zone return and hatch sequence owns the final minute.
   * Payload count is diagnostic only because several blocks can enter during
   * one blind feed action. */
  return CombatStrategy_ElapsedMs(now_ms) >= COMBAT_ENDGAME_RETURN_MS;
}

void CombatStrategy_GetPatrolWaypoint(float *x_mm, float *y_mm,
                                      uint32_t now_ms)
{
  if ((x_mm == NULL) || (y_mm == NULL)) return;
  if (CombatStrategy_ElapsedMs(now_ms) >= COMBAT_ENDGAME_RETURN_MS)
  {
    /* Guard just outside our warehouse. A block pulled outside the zone by
     * the opponent becomes a normal candidate and can be recaptured. */
    *x_mm = 550.0f;
    *y_mm = 450.0f;
    return;
  }
  if (opponent_exit_required != 0U)
  {
    *x_mm = 1500.0f;
    *y_mm = 1000.0f;
    return;
  }
  *x_mm = patrol_waypoints[patrol_index].x_mm;
  *y_mm = patrol_waypoints[patrol_index].y_mm;
}

void CombatStrategy_AdvancePatrol(void)
{
  patrol_index++;
  if (patrol_index >= COMBAT_PATROL_COUNT) patrol_index = 0U;
}

const char *CombatStrategy_PhaseName(uint32_t now_ms)
{
  uint32_t elapsed;
  if (combat_active == 0U) return "OFF";
  elapsed = CombatStrategy_ElapsedMs(now_ms);
  if (elapsed >= COMBAT_MATCH_DURATION_MS) return "FINISHED";
  if (elapsed >= COMBAT_ENDGAME_RETURN_MS) return "ENDGAME";
  if (opponent_exit_required != 0U) return "EXIT_OPP";
  if (inside_opponent_zone != 0U) return "STEAL";
  return "HARVEST";
}
