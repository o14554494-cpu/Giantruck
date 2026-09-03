#include "brush_feedback.h"
#include <string.h>

static volatile uint32_t encoder_count, encoder_errors;
static uint8_t encoder_ab;
static uint32_t zero_count, zero_errors, previous_count, previous_errors;
static uint32_t sample_tick, drive_tick, low_tick, current_tick, adc_tick;
static uint8_t low_pending, current_pending, adc_seen;
static BrushFeedbackSnapshot_t feedback;

static int32_t magnitude(int32_t value)
{
  /* Clamp the impossible half-range difference without signed overflow. */
  if (value == INT32_MIN) return INT32_MAX;
  return value < 0 ? -value : value;
}

void BrushFeedback_Reset(uint32_t now, uint8_t ab)
{
  memset(&feedback, 0, sizeof(feedback));
  encoder_count = encoder_errors = zero_count = zero_errors = 0U;
  previous_count = previous_errors = 0U;
  encoder_ab = ab & 3U;
  sample_tick = drive_tick = now;
  low_pending = current_pending = adc_seen = 0U;
  adc_tick = low_tick = current_tick = now;
  feedback.counts_per_rev = BRUSH_COUNTS_PER_REV;
  feedback.enabled = BRUSH_AUTO_ENABLE_ON_BOOT;
  feedback.current_ma = -1;
}

void BrushFeedback_EncoderEdge(uint8_t ab)
{
  /* + sequence: 00,01,11,10,00. Decode BOTH edges of BOTH phases (x4).
   * Simultaneous bit changes are invalid, never counted as shaft motion. */
  static const int8_t steps[16] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};
  uint8_t next = ab & 3U;
  if ((encoder_ab ^ next) == 3U) encoder_errors++;
  else encoder_count += (uint32_t)(int32_t)(steps[encoder_ab * 4U + next] * BRUSH_ENCODER_SIGN);
  encoder_ab = next;
}

uint32_t BrushFeedback_Count(void) { return encoder_count; }
uint32_t BrushFeedback_Errors(void) { return encoder_errors; }
uint32_t BrushFeedback_Cpr(void) { return feedback.counts_per_rev; }

void BrushFeedback_Zero(void)
{
  zero_count = encoder_count;
  zero_errors = encoder_errors;
}

uint8_t BrushFeedback_TeachOneTurn(void)
{
  uint32_t count = (uint32_t)magnitude((int32_t)(encoder_count - zero_count));
  if (feedback.drive != 0 || count < 4U || count > 1000000U ||
      encoder_errors != zero_errors) return 0U;
  feedback.counts_per_rev = count;
  feedback.forward_verified = 0U;
  return 1U;
}

void BrushFeedback_RecordAdc(uint16_t raw, uint32_t now)
{
  if (raw > 4095U) return;
  feedback.adc_raw = raw;
  feedback.adc_mv = (uint16_t)(((uint32_t)raw * BRUSH_ADC_VREF_MV + 2047U) / 4095U);
#if BRUSH_CURRENT_ENABLE
  feedback.current_ma = (int32_t)((uint32_t)magnitude(
      (int32_t)feedback.adc_mv - (int32_t)BRUSH_CURRENT_ZERO_MV) * 1000U /
      BRUSH_CURRENT_MV_PER_AMP);
#endif
  adc_seen = 1U;
  adc_tick = now;
}

void BrushFeedback_SetDrive(int8_t direction, uint32_t now)
{
  if (feedback.drive == direction) return;
  feedback.drive = direction;
  drive_tick = sample_tick = now;
  previous_count = encoder_count;
  previous_errors = encoder_errors;
  feedback.rpm_x10 = 0;
  low_pending = current_pending = 0U;
}

void BrushFeedback_SetEnabled(uint8_t enabled, uint32_t now)
{
  feedback.enabled = enabled != 0U && feedback.counts_per_rev >= 4U;
  if (enabled) feedback.last_event = BRUSH_EVENT_NONE;
  drive_tick = now;
  low_pending = current_pending = 0U;
  previous_errors = encoder_errors;
}

BrushFeedbackSnapshot_t BrushFeedback_Snapshot(uint32_t now)
{
  BrushFeedbackSnapshot_t value = feedback;
  value.count = (int32_t)(encoder_count - zero_count);
  value.errors = encoder_errors - zero_errors;
  value.adc_fresh = adc_seen && (uint32_t)(now - adc_tick) <= BRUSH_ADC_STALE_MS;
  return value;
}

BrushEvent_t BrushFeedback_Update(uint32_t now)
{
  uint32_t elapsed = now - sample_tick;
  uint32_t count, errors;
  int32_t delta;
  BrushEvent_t event = BRUSH_EVENT_NONE;
  if (elapsed < BRUSH_SAMPLE_MS) return BRUSH_EVENT_NONE;
  count = encoder_count;
  errors = encoder_errors;
  delta = (int32_t)(count - previous_count);
  feedback.rpm_x10 = feedback.counts_per_rev == 0U ? 0 :
      (int32_t)(((int64_t)delta * 600000) / ((int64_t)feedback.counts_per_rev * elapsed));
  sample_tick = now;
  previous_count = count;
  if (feedback.drive > 0 && feedback.rpm_x10 >= BRUSH_STALL_RPM_X10 && errors == previous_errors)
    feedback.forward_verified = 1U;
  if (!feedback.enabled || feedback.drive == 0)
  {
    previous_errors = errors;
    return BRUSH_EVENT_NONE;
  }
  if (errors != previous_errors) event = BRUSH_EVENT_ENCODER_ERRORS;
  previous_errors = errors;
  if ((uint32_t)(now - drive_tick) < BRUSH_STARTUP_GRACE_MS)
  {
    low_pending = current_pending = 0U;
    if (event == BRUSH_EVENT_NONE) return event;
  }
  else
  {
#if BRUSH_CURRENT_ENABLE
    if (!adc_seen || (uint32_t)(now - adc_tick) > BRUSH_ADC_STALE_MS)
      event = BRUSH_EVENT_ADC_LOST;
    else if (feedback.current_ma >= (int32_t)BRUSH_CURRENT_STALL_MA)
    {
      if (!current_pending) { current_pending = 1U; current_tick = now; }
      if ((uint32_t)(now - current_tick) >= BRUSH_CURRENT_CONFIRM_MS && event == BRUSH_EVENT_NONE)
        event = BRUSH_EVENT_HIGH_CURRENT;
    }
    else current_pending = 0U;
#endif
    if (feedback.drive > 0)
    {
      if (feedback.rpm_x10 < -BRUSH_STALL_RPM_X10 && event == BRUSH_EVENT_NONE)
        event = BRUSH_EVENT_WRONG_DIRECTION;
      if (feedback.rpm_x10 < BRUSH_STALL_RPM_X10)
      {
        if (!low_pending) { low_pending = 1U; low_tick = now; }
        if ((uint32_t)(now - low_tick) >= BRUSH_STALL_CONFIRM_MS && event == BRUSH_EVENT_NONE)
          event = feedback.forward_verified ? BRUSH_EVENT_LOW_RPM : BRUSH_EVENT_NO_FEEDBACK;
      }
      else low_pending = 0U;
    }
  }
  if (event != BRUSH_EVENT_NONE)
  {
    feedback.last_event = event;
    low_pending = current_pending = 0U;
  }
  return event;
}

const char *BrushFeedback_EventName(BrushEvent_t event)
{
  switch (event)
  {
    case BRUSH_EVENT_LOW_RPM: return "LOW_RPM";
    case BRUSH_EVENT_HIGH_CURRENT: return "HIGH_CURRENT";
    case BRUSH_EVENT_NO_FEEDBACK: return "NO_FEEDBACK";
    case BRUSH_EVENT_ENCODER_ERRORS: return "ENCODER_ERRORS";
    case BRUSH_EVENT_WRONG_DIRECTION: return "ENCODER_SIGN";
    case BRUSH_EVENT_ADC_LOST: return "ADC_LOST";
    default: return "NONE";
  }
}
