#include "path_planner.h"

#include <string.h>

#define PLANNER_PI                    3.14159265358979323846f
#define PLANNER_HALF_PI               1.57079632679489661923f
#define PLANNER_TWO_PI                6.28318530717958647692f
#define PLANNER_TURN_PENALTY_MM       260.0f
#define PLANNER_QUALITY_PENALTY_MM    2.0f
#define PLANNER_TWO_OPT_PASSES        4U

static float Planner_Abs(float value)
{
  return value < 0.0f ? -value : value;
}

float Planner_NormalizeAngle(float angle_rad)
{
  while (angle_rad > PLANNER_PI)
  {
    angle_rad -= PLANNER_TWO_PI;
  }
  while (angle_rad < -PLANNER_PI)
  {
    angle_rad += PLANNER_TWO_PI;
  }
  return angle_rad;
}

float Planner_Sin(float angle_rad)
{
  float x;
  float x2;

  x = Planner_NormalizeAngle(angle_rad);
  if (x > PLANNER_HALF_PI)
  {
    x = PLANNER_PI - x;
  }
  else if (x < -PLANNER_HALF_PI)
  {
    x = -PLANNER_PI - x;
  }

  x2 = x * x;
  return x * (1.0f - x2 / 6.0f +
              (x2 * x2) / 120.0f -
              (x2 * x2 * x2) / 5040.0f);
}

float Planner_Cos(float angle_rad)
{
  return Planner_Sin(angle_rad + PLANNER_HALF_PI);
}

float Planner_Sqrt(float value)
{
  float estimate;
  uint8_t iteration;

  if (value <= 0.0f)
  {
    return 0.0f;
  }

  estimate = value > 1.0f ? value : 1.0f;
  for (iteration = 0U; iteration < 16U; iteration++)
  {
    estimate = 0.5f * (estimate + value / estimate);
  }
  return estimate;
}

static const MapTarget_t *Planner_FindCandidate(const MapTarget_t *candidates,
                                                 uint8_t candidate_count,
                                                 uint16_t target_id)
{
  uint8_t index;

  for (index = 0U; index < candidate_count; index++)
  {
    if (candidates[index].id == target_id)
    {
      return &candidates[index];
    }
  }
  return NULL;
}

static float Planner_StepCost(float from_x, float from_y,
                              float heading_x, float heading_y,
                              const MapTarget_t *target)
{
  float dx = target->x_mm - from_x;
  float dy = target->y_mm - from_y;
  float distance = Planner_Sqrt(dx * dx + dy * dy);
  float turn_sine = 0.0f;

  if (distance > 1.0f)
  {
    turn_sine = Planner_Abs(heading_x * dy - heading_y * dx) / distance;
  }

  return distance + PLANNER_TURN_PENALTY_MM * turn_sine +
         PLANNER_QUALITY_PENALTY_MM * (float)(100U - target->quality);
}

static float Planner_RouteCost(const RobotPose_t *pose,
                               const MapTarget_t *candidates,
                               uint8_t candidate_count,
                               const PlannerRoute_t *route)
{
  float x = pose->x_mm;
  float y = pose->y_mm;
  float heading_x = Planner_Cos(pose->heading_rad);
  float heading_y = Planner_Sin(pose->heading_rad);
  float total_cost = 0.0f;
  uint8_t route_index;

  for (route_index = 0U; route_index < route->count; route_index++)
  {
    const MapTarget_t *target = Planner_FindCandidate(
        candidates, candidate_count, route->target_ids[route_index]);
    float dx;
    float dy;
    float distance;

    if (target == NULL)
    {
      continue;
    }

    total_cost += Planner_StepCost(x, y, heading_x, heading_y, target);
    dx = target->x_mm - x;
    dy = target->y_mm - y;
    distance = Planner_Sqrt(dx * dx + dy * dy);
    if (distance > 1.0f)
    {
      heading_x = dx / distance;
      heading_y = dy / distance;
    }
    x = target->x_mm;
    y = target->y_mm;
  }
  return total_cost;
}

static void Planner_ReverseSegment(PlannerRoute_t *route,
                                   uint8_t first, uint8_t last)
{
  while (first < last)
  {
    uint16_t temporary = route->target_ids[first];
    route->target_ids[first] = route->target_ids[last];
    route->target_ids[last] = temporary;
    first++;
    last--;
  }
}

void Planner_BuildRoute(const RobotPose_t *pose,
                        const MapTarget_t *candidates,
                        uint8_t candidate_count,
                        PlannerRoute_t *route)
{
  uint8_t used[TARGET_MAP_MAX_TARGETS];
  float current_x;
  float current_y;
  float heading_x;
  float heading_y;
  uint8_t route_index;
  uint8_t pass;

  if (route == NULL)
  {
    return;
  }

  memset(route, 0, sizeof(*route));
  if ((pose == NULL) || (candidates == NULL) || (candidate_count == 0U))
  {
    return;
  }

  if (candidate_count > TARGET_MAP_MAX_TARGETS)
  {
    candidate_count = TARGET_MAP_MAX_TARGETS;
  }
  memset(used, 0, sizeof(used));

  current_x = pose->x_mm;
  current_y = pose->y_mm;
  heading_x = Planner_Cos(pose->heading_rad);
  heading_y = Planner_Sin(pose->heading_rad);

  /* Nearest-neighbour seed with turn and observation-quality penalties. */
  for (route_index = 0U;
       (route_index < candidate_count) &&
       (route_index < PLANNER_MAX_ROUTE_TARGETS);
       route_index++)
  {
    int16_t best_index = -1;
    float best_cost = 3.4e38f;
    uint8_t candidate_index;

    for (candidate_index = 0U;
         candidate_index < candidate_count;
         candidate_index++)
    {
      float cost;
      if (used[candidate_index] != 0U)
      {
        continue;
      }

      cost = Planner_StepCost(current_x, current_y, heading_x, heading_y,
                              &candidates[candidate_index]);
      if (cost < best_cost)
      {
        best_cost = cost;
        best_index = (int16_t)candidate_index;
      }
    }

    if (best_index < 0)
    {
      break;
    }

    used[best_index] = 1U;
    route->target_ids[route->count++] = candidates[best_index].id;

    {
      float dx = candidates[best_index].x_mm - current_x;
      float dy = candidates[best_index].y_mm - current_y;
      float distance = Planner_Sqrt(dx * dx + dy * dy);
      if (distance > 1.0f)
      {
        heading_x = dx / distance;
        heading_y = dy / distance;
      }
      current_x = candidates[best_index].x_mm;
      current_y = candidates[best_index].y_mm;
    }
  }

  /* Short 2-opt improvement: reverse route segments when total cost drops. */
  route->estimated_cost_mm = Planner_RouteCost(
      pose, candidates, candidate_count, route);
  for (pass = 0U; pass < PLANNER_TWO_OPT_PASSES; pass++)
  {
    uint8_t improved = 0U;
    uint8_t first;

    for (first = 0U; first + 1U < route->count; first++)
    {
      uint8_t last;
      for (last = (uint8_t)(first + 1U); last < route->count; last++)
      {
        float candidate_cost;
        Planner_ReverseSegment(route, first, last);
        candidate_cost = Planner_RouteCost(
            pose, candidates, candidate_count, route);
        if (candidate_cost + 0.5f < route->estimated_cost_mm)
        {
          route->estimated_cost_mm = candidate_cost;
          improved = 1U;
        }
        else
        {
          Planner_ReverseSegment(route, first, last);
        }
      }
    }

    if (improved == 0U)
    {
      break;
    }
  }
}
