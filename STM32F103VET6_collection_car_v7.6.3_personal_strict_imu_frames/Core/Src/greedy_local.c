#include "greedy_local.h"
#include <string.h>

static GreedyLocalTarget_t lock;
static uint32_t reset_tick, acquire_delay;
static float Abs(float x) { return x < 0 ? -x : x; }

void GreedyLocal_Reset(uint32_t now, uint32_t acquire_delay_ms)
{
  memset(&lock, 0, sizeof(lock));
  reset_tick = now;
  acquire_delay = acquire_delay_ms;
}

void GreedyLocal_Observe(const VisionFrame_t *frame, uint32_t now)
{
  const VisionTarget_t *best = NULL;
  float best_score = 3.4e38f;
  uint8_t i;
  lock.visible = 0U;
  if (!frame || (uint32_t)(now - reset_tick) < acquire_delay) return;
  for (i = 0; i < frame->target_count && i < VISION_MAX_TARGETS; i++)
  {
    const VisionTarget_t *t = &frame->targets[i];
    float score;
    if ((t->color != VISION_COLOR_RED && t->color != VISION_COLOR_YELLOW) ||
        t->forward_mm < 60U || t->forward_mm > 2500U) continue;
    if (!lock.locked)
      score = (float)t->forward_mm * t->forward_mm + (float)t->lateral_mm * t->lateral_mm;
    else
    {
      float bearing_error, range_error;
      if (t->color != lock.target.color) continue;
      /* Associate by bearing and depth continuity, not drifting world pose.
       * This is a local track, not a guaranteed physical object identity. */
      bearing_error = Abs((float)t->lateral_mm / t->forward_mm -
                          (float)lock.target.lateral_mm / lock.target.forward_mm);
      range_error = Abs((float)t->forward_mm - lock.target.forward_mm);
      if (bearing_error > 0.45f || range_error > 150.0f + 0.5f * lock.target.forward_mm) continue;
      score = bearing_error * 500.0f + range_error;
    }
    if (score < best_score) { best_score = score; best = t; }
  }
  if (!best) return;
  if (!lock.locked) lock.locked_since_ms = now;
  lock.target = *best;
  lock.last_seen_ms = now;
  lock.locked = lock.visible = 1U;
}

const GreedyLocalTarget_t *GreedyLocal_Get(void) { return &lock; }
uint8_t GreedyLocal_Visible(uint32_t now)
{
  return lock.locked && lock.visible && (uint32_t)(now - lock.last_seen_ms) <= GREEDY_LOCAL_LOST_MS;
}
