#include "target_map.h"

#include <string.h>

#define TARGET_MERGE_RADIUS_MM 120.0f
#define TARGET_MIN_QUALITY     15U
#define TARGET_COLLECT_HOLDOFF_MS 2000U
#define TARGET_MIN_CONFIRMATIONS  2U
#define TARGET_STRONG_QUALITY     75U

static float TargetMap_DistanceSquared(float x1, float y1, float x2, float y2)
{
  float dx = x2 - x1;
  float dy = y2 - y1;
  return dx * dx + dy * dy;
}

static uint8_t TargetMap_IsPlanningCandidate(const MapTarget_t *target)
{
  if ((target == NULL) || (target->valid == 0U) ||
      (target->collected != 0U))
  {
    return 0U;
  }

  return (target->seen_count >= TARGET_MIN_CONFIRMATIONS) ||
         (target->quality >= TARGET_STRONG_QUALITY);
}

void TargetMap_Reset(TargetMap_t *map)
{
  if (map == NULL)
  {
    return;
  }

  memset(map, 0, sizeof(*map));
  map->next_id = 1U;
}

uint16_t TargetMap_Upsert(TargetMap_t *map, uint8_t color,
                          float x_mm, float y_mm, uint8_t quality,
                          uint32_t now_ms)
{
  uint8_t index;
  int16_t free_index = -1;
  int16_t replacement_index = -1;
  uint32_t oldest_tick = 0xFFFFFFFFUL;
  const float merge_distance_squared =
      TARGET_MERGE_RADIUS_MM * TARGET_MERGE_RADIUS_MM;

  if ((map == NULL) || (color == 0U) || (quality < TARGET_MIN_QUALITY))
  {
    return 0U;
  }

  for (index = 0U; index < TARGET_MAP_MAX_TARGETS; index++)
  {
    MapTarget_t *target = &map->targets[index];

    if (target->valid == 0U)
    {
      if (free_index < 0)
      {
        free_index = (int16_t)index;
      }
      continue;
    }

    if ((target->color == color) &&
        (TargetMap_DistanceSquared(target->x_mm, target->y_mm,
                                   x_mm, y_mm) <= merge_distance_squared))
    {
      /* Do not immediately recreate a target while it is passing beneath the
       * camera during the timed collection motion. If it remains visible after
       * the holdoff, it is treated as a failed pickup and becomes available. */
      if ((target->collected != 0U) &&
          ((now_ms - target->last_seen_ms) < TARGET_COLLECT_HOLDOFF_MS))
      {
        return target->id;
      }

      target->collected = 0U;
      /* Low-pass the global position to reduce frame-to-frame camera noise. */
      target->x_mm = target->x_mm * 0.65f + x_mm * 0.35f;
      target->y_mm = target->y_mm * 0.65f + y_mm * 0.35f;
      target->quality = (uint8_t)(((uint16_t)target->quality * 2U + quality) / 3U);
      if (target->seen_count < 255U)
      {
        target->seen_count++;
      }
      target->last_seen_ms = now_ms;
      return target->id;
    }

    if ((target->collected != 0U) && (target->last_seen_ms < oldest_tick))
    {
      oldest_tick = target->last_seen_ms;
      replacement_index = (int16_t)index;
    }
  }

  if (free_index >= 0)
  {
    replacement_index = free_index;
  }
  else if (replacement_index < 0)
  {
    /* If the map is full, replace the stalest low-quality available target. */
    uint16_t worst_score = 0xFFFFU;
    for (index = 0U; index < TARGET_MAP_MAX_TARGETS; index++)
    {
      MapTarget_t *target = &map->targets[index];
      uint16_t score = (uint16_t)target->quality +
                       (uint16_t)((now_ms - target->last_seen_ms) < 2000U ? 100U : 0U);
      if (score < worst_score)
      {
        worst_score = score;
        replacement_index = (int16_t)index;
      }
    }
  }

  if (replacement_index >= 0)
  {
    MapTarget_t *target = &map->targets[replacement_index];
    memset(target, 0, sizeof(*target));
    target->id = map->next_id++;
    if (map->next_id == 0U)
    {
      map->next_id = 1U;
    }
    target->color = color;
    target->x_mm = x_mm;
    target->y_mm = y_mm;
    target->quality = quality;
    target->seen_count = 1U;
    target->valid = 1U;
    target->last_seen_ms = now_ms;
    return target->id;
  }

  return 0U;
}

uint8_t TargetMap_GetCandidates(const TargetMap_t *map,
                                MapTarget_t *output, uint8_t output_capacity)
{
  uint8_t index;
  uint8_t count = 0U;

  if ((map == NULL) || (output == NULL) || (output_capacity == 0U))
  {
    return 0U;
  }

  for (index = 0U;
       (index < TARGET_MAP_MAX_TARGETS) && (count < output_capacity);
       index++)
  {
    const MapTarget_t *target = &map->targets[index];
    if (TargetMap_IsPlanningCandidate(target) != 0U)
    {
      output[count++] = *target;
    }
  }
  return count;
}

MapTarget_t *TargetMap_FindById(TargetMap_t *map, uint16_t id)
{
  uint8_t index;

  if ((map == NULL) || (id == 0U))
  {
    return NULL;
  }

  for (index = 0U; index < TARGET_MAP_MAX_TARGETS; index++)
  {
    if ((map->targets[index].valid != 0U) && (map->targets[index].id == id))
    {
      return &map->targets[index];
    }
  }
  return NULL;
}

void TargetMap_MarkCollected(TargetMap_t *map, uint16_t id, uint32_t now_ms)
{
  MapTarget_t *target = TargetMap_FindById(map, id);
  if (target != NULL)
  {
    target->collected = 1U;
    target->last_seen_ms = now_ms;
  }
}

uint8_t TargetMap_CountAvailable(const TargetMap_t *map)
{
  uint8_t index;
  uint8_t count = 0U;

  if (map == NULL)
  {
    return 0U;
  }

  for (index = 0U; index < TARGET_MAP_MAX_TARGETS; index++)
  {
    if (TargetMap_IsPlanningCandidate(&map->targets[index]) != 0U)
    {
      count++;
    }
  }
  return count;
}
