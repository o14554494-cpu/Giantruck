#ifndef BRUSH_FEEDBACK_H
#define BRUSH_FEEDBACK_H
#include <stdint.h>
#include "brush_feedback_config.h"

typedef enum
{
  BRUSH_EVENT_NONE = 0,
  BRUSH_EVENT_LOW_RPM,
  BRUSH_EVENT_HIGH_CURRENT,
  BRUSH_EVENT_NO_FEEDBACK,
  BRUSH_EVENT_ENCODER_ERRORS,
  BRUSH_EVENT_WRONG_DIRECTION,
  BRUSH_EVENT_ADC_LOST
} BrushEvent_t;

typedef struct
{
  int32_t count; /* Relative to N, diagnostic only; wrap-safe unsigned internally. */
  uint32_t errors;
  uint32_t counts_per_rev;
  int32_t rpm_x10;
  uint16_t adc_raw;
  uint16_t adc_mv;
  int32_t current_ma; /* -1 = current conversion/protection not configured */
  uint8_t adc_fresh;
  uint8_t enabled;
  uint8_t forward_verified;
  int8_t drive;
  BrushEvent_t last_event;
} BrushFeedbackSnapshot_t;

void BrushFeedback_Reset(uint32_t now, uint8_t ab);
void BrushFeedback_EncoderEdge(uint8_t ab); /* ISR: no blocking calls */
void BrushFeedback_RecordAdc(uint16_t raw, uint32_t now);
void BrushFeedback_SetDrive(int8_t direction, uint32_t now);
void BrushFeedback_SetEnabled(uint8_t enabled, uint32_t now);
void BrushFeedback_Zero(void); /* Baseline only; does not modify ISR counters. */
uint8_t BrushFeedback_TeachOneTurn(void);
uint32_t BrushFeedback_Count(void);
uint32_t BrushFeedback_Errors(void);
uint32_t BrushFeedback_Cpr(void);
BrushFeedbackSnapshot_t BrushFeedback_Snapshot(uint32_t now);
BrushEvent_t BrushFeedback_Update(uint32_t now);
const char *BrushFeedback_EventName(BrushEvent_t event);

/* STM32-only implementation; not used in host logic tests. */
void BrushFeedback_HardwareInit(void);
void BrushFeedback_HardwarePoll(void);
#endif
