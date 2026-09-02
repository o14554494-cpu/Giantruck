#ifndef TARGET_MAP_H
#define TARGET_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TARGET_MAP_MAX_TARGETS 16U

typedef struct
{
  uint16_t id;
  uint8_t color;
  float x_mm;              /* Global forward-axis coordinate. */
  float y_mm;              /* Global left-axis coordinate. */
  uint8_t quality;
  uint8_t seen_count;
  uint8_t valid;
  uint8_t collected;
  uint32_t last_seen_ms;
} MapTarget_t;

typedef struct
{
  MapTarget_t targets[TARGET_MAP_MAX_TARGETS];
  uint16_t next_id;
} TargetMap_t;

void TargetMap_Reset(TargetMap_t *map);
uint16_t TargetMap_Upsert(TargetMap_t *map, uint8_t color,
                          float x_mm, float y_mm, uint8_t quality,
                          uint32_t now_ms);
/* Returns uncollected targets confirmed by at least two observations, or by
 * one strong observation. This prevents transient colour blobs from entering
 * the route planner. */
uint8_t TargetMap_GetCandidates(const TargetMap_t *map,
                                MapTarget_t *output, uint8_t output_capacity);
MapTarget_t *TargetMap_FindById(TargetMap_t *map, uint16_t id);
void TargetMap_MarkCollected(TargetMap_t *map, uint16_t id, uint32_t now_ms);
uint8_t TargetMap_CountAvailable(const TargetMap_t *map);

#ifdef __cplusplus
}
#endif

#endif /* TARGET_MAP_H */
