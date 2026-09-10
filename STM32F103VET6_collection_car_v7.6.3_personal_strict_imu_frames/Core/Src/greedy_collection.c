#include "greedy_collection.h"
#include <string.h>

static MapTarget_t entries[GREEDY_CAPACITY];
static uint16_t next_id;
static uint8_t collected_count;

static float DistanceSquared(float x, float y, const MapTarget_t *target)
{
  float dx = target->x_mm - x, dy = target->y_mm - y;
  return dx * dx + dy * dy;
}

static void Expire(uint32_t now)
{
  uint8_t i;
  for (i = 0; i < GREEDY_CAPACITY; i++)
    if (entries[i].valid && !entries[i].collected &&
        (uint32_t)(now - entries[i].last_seen_ms) > GREEDY_TARGET_TTL_MS)
      entries[i].valid = 0;
}

void Greedy_Reset(void)
{
  memset(entries, 0, sizeof(entries));
  next_id = 1;
  collected_count = 0;
}

void Greedy_ClearPending(void)
{
  uint8_t i;
  for (i = 0; i < GREEDY_CAPACITY; i++)
    if (!entries[i].collected) entries[i].valid = 0;
}

void Greedy_Observe(uint8_t color, float x, float y, uint8_t quality, uint32_t now)
{
  uint8_t i;
  int slot = -1, match = -1, oldest = -1;
  float best = GREEDY_MERGE_RADIUS_MM * GREEDY_MERGE_RADIUS_MM;
  uint32_t oldest_age = 0;
  if (collected_count >= GREEDY_BATCH_SIZE) return;
  Expire(now);
  /* Keep collected positions for this batch, even across loss/recovery.
   * They are software pickup records, NOT physical presence measurements. */
  for (i = 0; i < GREEDY_CAPACITY; i++)
    if (entries[i].valid && entries[i].collected && entries[i].color == color &&
        DistanceSquared(x, y, &entries[i]) <= best) return;
  for (i = 0; i < GREEDY_CAPACITY; i++)
  {
    MapTarget_t *t = &entries[i];
    if (!t->valid) { if (slot < 0) slot = i; continue; }
    if (t->collected) continue;
    if (oldest < 0 || (uint32_t)(now - t->last_seen_ms) > oldest_age)
    { oldest = i; oldest_age = now - t->last_seen_ms; }
    if (t->color == color)
    {
      float d = DistanceSquared(x, y, t);
      if (d <= best) { best = d; match = i; }
    }
  }
  if (match >= 0)
  {
    MapTarget_t *t = &entries[match];
    t->x_mm = x; t->y_mm = y; t->quality = quality; t->last_seen_ms = now;
    if (t->seen_count < 255U) t->seen_count++;
    return;
  }
  if (slot < 0) slot = oldest;
  if (slot >= 0)
  {
    MapTarget_t *t = &entries[slot];
    memset(t, 0, sizeof(*t));
    t->id = next_id++;
    if (next_id == 0) next_id = 1;
    t->valid = 1; t->seen_count = 1; t->color = color; t->quality = quality;
    t->x_mm = x; t->y_mm = y; t->last_seen_ms = now;
  }
}

MapTarget_t *Greedy_Find(uint16_t id, uint32_t now)
{
  uint8_t i;
  Expire(now);
  for (i = 0; i < GREEDY_CAPACITY; i++)
    if (entries[i].valid && !entries[i].collected && entries[i].id == id)
      return &entries[i];
  return NULL;
}

uint8_t Greedy_List(const RobotPose_t *pose, uint32_t now,
                    uint16_t *ids, uint8_t capacity)
{
  uint8_t sorted[GREEDY_CAPACITY], count = 0, i;
  if (!pose || !ids || !capacity) return 0;
  Expire(now);
  for (i = 0; i < GREEDY_CAPACITY; i++)
  {
    uint8_t j;
    float distance;
    if (!entries[i].valid || entries[i].collected) continue;
    distance = DistanceSquared(pose->x_mm, pose->y_mm, &entries[i]);
    j = count;
    while (j > 0)
    {
      float previous = DistanceSquared(pose->x_mm, pose->y_mm, &entries[sorted[j-1]]);
      if (previous < distance ||
          (previous == distance && entries[sorted[j-1]].id < entries[i].id)) break;
      sorted[j] = sorted[j-1]; j--;
    }
    sorted[j] = i; count++;
  }
  if (count > capacity) count = capacity;
  for (i = 0; i < count; i++) ids[i] = entries[sorted[i]].id;
  return count;
}

MapTarget_t *Greedy_Nearest(const RobotPose_t *pose, uint32_t now)
{
  uint16_t id;
  return Greedy_List(pose, now, &id, 1) ? Greedy_Find(id, now) : NULL;
}

uint8_t Greedy_MarkCollected(uint16_t id)
{
  uint8_t i;
  for (i = 0; i < GREEDY_CAPACITY; i++)
    if (entries[i].valid && !entries[i].collected && entries[i].id == id &&
        collected_count < GREEDY_BATCH_SIZE)
    { entries[i].collected = 1; collected_count++; return 1; }
  return 0;
}

void Greedy_Forget(uint16_t id)
{
  uint8_t i;
  for (i = 0; i < GREEDY_CAPACITY; i++)
    if (entries[i].id == id && !entries[i].collected) entries[i].valid = 0;
}

uint8_t Greedy_CountCollected(void) { return collected_count; }

uint8_t Greedy_RecordPickup(void)
{
  if (collected_count >= GREEDY_BATCH_SIZE) return 0U;
  collected_count++;
  return 1U;
}
