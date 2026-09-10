#ifndef GREEDY_LOCAL_H
#define GREEDY_LOCAL_H
#include "vision_protocol.h"

#define GREEDY_LOCAL_LOST_MS 600U
#define GREEDY_LOCAL_FOLLOW_MAX_MS 12000U
#define GREEDY_LOCAL_AFTER_PICK_MS 400U
#define GREEDY_LOCAL_REPORT_MS 1000U

typedef struct
{
  VisionTarget_t target;
  uint32_t last_seen_ms;
  uint32_t locked_since_ms;
  uint8_t locked;
  uint8_t visible;
} GreedyLocalTarget_t;

/* A single camera-relative target. No arena coordinates or historical list. */
void GreedyLocal_Reset(uint32_t now, uint32_t acquire_delay_ms);
void GreedyLocal_Observe(const VisionFrame_t *frame, uint32_t now);
const GreedyLocalTarget_t *GreedyLocal_Get(void);
uint8_t GreedyLocal_Visible(uint32_t now);
#endif
