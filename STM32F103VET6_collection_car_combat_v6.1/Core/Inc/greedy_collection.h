#ifndef GREEDY_COLLECTION_H
#define GREEDY_COLLECTION_H

#include "path_planner.h"

/* Distance-only memory. Coordinates are in the calibrated arena frame. */
#define GREEDY_CAPACITY 32U
#define GREEDY_BATCH_SIZE 10U
#define GREEDY_TARGET_TTL_MS 20000U
#define GREEDY_SELECT_INTERVAL_MS 300U
#define GREEDY_MERGE_RADIUS_MM 120.0f

void Greedy_Reset(void);
void Greedy_ClearPending(void);
void Greedy_Observe(uint8_t color, float x, float y, uint8_t quality, uint32_t now);
MapTarget_t *Greedy_Find(uint16_t id, uint32_t now);
MapTarget_t *Greedy_Nearest(const RobotPose_t *pose, uint32_t now);
/* Sorted by current Euclidean distance; no route or weighted cost. */
uint8_t Greedy_List(const RobotPose_t *pose, uint32_t now,
                    uint16_t *ids, uint8_t capacity);
uint8_t Greedy_MarkCollected(uint16_t id);
void Greedy_Forget(uint16_t id);
uint8_t Greedy_CountCollected(void);

#endif
