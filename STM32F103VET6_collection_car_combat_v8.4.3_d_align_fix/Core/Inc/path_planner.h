#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "target_map.h"

#define PLANNER_MAX_ROUTE_TARGETS 8U

typedef struct
{
  float x_mm;
  float y_mm;
  float heading_rad;
} RobotPose_t;

typedef struct
{
  uint16_t target_ids[PLANNER_MAX_ROUTE_TARGETS];
  uint8_t count;
  float estimated_cost_mm;
} PlannerRoute_t;

void Planner_BuildRoute(const RobotPose_t *pose,
                        const MapTarget_t *candidates,
                        uint8_t candidate_count,
                        PlannerRoute_t *route);

float Planner_Sqrt(float value);
float Planner_Sin(float angle_rad);
float Planner_Cos(float angle_rad);
float Planner_NormalizeAngle(float angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* PATH_PLANNER_H */
