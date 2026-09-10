#include <assert.h>
#include <stdio.h>
#include "brush_feedback.h"

static uint8_t ab;
static void raw_steps(int count)
{
  const uint8_t next[4] = {1,3,0,2}, prev[4] = {2,0,3,1};
  int i, n = count < 0 ? -count : count;
  for (i = 0; i < n; i++)
  {
    ab = count < 0 ? prev[ab] : next[ab];
    BrushFeedback_EncoderEdge(ab);
  }
}
static void steps(int count)
{
  /* Tests below specify normalized mechanical direction; raw tests explicitly
   * exercise the A/B edge order independently of the configured polarity. */
  raw_steps(count * BRUSH_ENCODER_SIGN);
}
static void reset(uint32_t now)
{
  ab = 0;
  BrushFeedback_Reset(now, ab);
}
static void teach(void)
{
  BrushFeedback_Zero(); steps(40);
  assert(BrushFeedback_TeachOneTurn());
  assert(BrushFeedback_Cpr() == 40);
}
static BrushEvent_t sample(uint32_t now, int edge_count)
{
  steps(edge_count);
  BrushFeedback_RecordAdc(2048, now); /* about zero current for test config */
  return BrushFeedback_Update(now);
}
int main(void)
{
  uint32_t now, count;
  BrushEvent_t event;
  reset(0); BrushFeedback_SetDrive(1, 0); BrushFeedback_ArmForCollection(0);
  for (now = 100; now <= 2000; now += 100)
  {
    raw_steps(-4); BrushFeedback_RecordAdc(2048, now);
    event = BrushFeedback_Update(now);
#if BRUSH_ENCODER_SIGN == -1
    assert(event == BRUSH_EVENT_NONE);
    assert(BrushFeedback_Snapshot(now).delta_counts == 4);
#else
    if (now == 1000U + BRUSH_WRONG_DIRECTION_CONFIRM_MS)
      assert(event == BRUSH_EVENT_WRONG_DIRECTION); /* Reproduce reported polarity fault. */
#endif
  }
  puts("PASS: actual negative A/B sequence reproduces +1 mismatch and runs normally with -1 correction");
  /* Q needs no calibration command, but must see fresh forward pulses first. */
  reset(0); BrushFeedback_SetDrive(1, 0);
  BrushFeedback_SetEnabled(1, 0);
  assert(!BrushFeedback_Snapshot(0).enabled); /* E policy unchanged */
  BrushFeedback_ArmForCollection(0);
  assert(BrushFeedback_Snapshot(0).enabled && BrushFeedback_Cpr() == 0);
  for (now = 100; now <= 1100; now += 100) assert(sample(now, 4) == BRUSH_EVENT_NONE);
  assert(BrushFeedback_Snapshot(1100).forward_verified);
  assert(BrushFeedback_Snapshot(1100).delta_counts == 4);
  for (now = 1200; now <= 1700; now += 100) assert(sample(now, 0) == BRUSH_EVENT_NONE);
  assert(sample(1800, 0) == BRUSH_EVENT_NO_PULSES);
  BrushFeedback_SetEnabled(0, 1800);
  assert(sample(2000, 0) == BRUSH_EVENT_NONE);
  BrushFeedback_ArmForCollection(2000);
  assert(!BrushFeedback_Snapshot(2000).forward_verified);
  for (now = 2100; now <= 3500; now += 100) assert(sample(now, 0) == BRUSH_EVENT_NONE);
  assert(sample(3600, 0) == BRUSH_EVENT_NO_FEEDBACK);

  reset(0); BrushFeedback_SetDrive(1, 0); BrushFeedback_ArmForCollection(0);
  assert(sample(1100, -4) == BRUSH_EVENT_NONE);
  assert(sample(1200, 4) == BRUSH_EVENT_NONE); /* one transient reverse sample is not fatal */
  assert(sample(1300, -4) == BRUSH_EVENT_NONE);
  assert(sample(1400, -4) == BRUSH_EVENT_NONE);
  assert(sample(1500, -4) == BRUSH_EVENT_NONE);
  assert(sample(1600, -4) == BRUSH_EVENT_WRONG_DIRECTION);
  reset(0); BrushFeedback_SetDrive(1, 0); BrushFeedback_ArmForCollection(0);
  BrushFeedback_EncoderEdge(3); ab = 3;
  assert(sample(100, 0) == BRUSH_EVENT_NONE);
  assert(BrushFeedback_Snapshot(100).error_window == 1);
  reset(UINT32_MAX - 50U); BrushFeedback_SetDrive(1, UINT32_MAX - 50U);
  BrushFeedback_ArmForCollection(UINT32_MAX - 50U);
  assert(sample(49U, 4) == BRUSH_EVENT_NONE);
  assert(BrushFeedback_Snapshot(49U).forward_motion);
  puts("PASS: Q pulse arming without CPR, fresh forward verification, pulse-loss confirmation, disarm, polarity, invalid edges and tick wrap");
  reset(0);
  assert(!BrushFeedback_TeachOneTurn());
  steps(-4); assert((int32_t)BrushFeedback_Count() == -4);
  steps(4); assert(BrushFeedback_Count() == 0); /* unsigned wrap, no UB */
  BrushFeedback_EncoderEdge(3); ab = 3;
  assert(BrushFeedback_Errors() == 1);
  steps(40); assert(!BrushFeedback_TeachOneTurn());
  BrushFeedback_Zero(); steps(40);
  assert(BrushFeedback_TeachOneTurn());
  assert(BrushFeedback_Snapshot(0).count == 40 && BrushFeedback_Snapshot(0).errors == 0);

  reset(0); teach();
  BrushFeedback_SetDrive(1, 0);
  assert(sample(100, 4) == BRUSH_EVENT_NONE);
  assert(BrushFeedback_Snapshot(100).rpm_x10 == 600);
  assert(BrushFeedback_Snapshot(100).forward_verified);
  BrushFeedback_SetEnabled(1, 100);
  for (now = 200; now <= 1000; now += 100) assert(sample(now, 0) == BRUSH_EVENT_NONE);
  for (now = 1100; now <= 1600; now += 100) assert(sample(now, 0) == BRUSH_EVENT_NONE);
  assert(sample(1700, 0) == BRUSH_EVENT_LOW_RPM);
  BrushFeedback_SetEnabled(0, 1700);
  assert(sample(3000, 0) == BRUSH_EVENT_NONE);

  reset(0); teach(); BrushFeedback_SetDrive(1, 0); BrushFeedback_SetEnabled(1, 0);
  for (now = 100; now <= 1500; now += 100) assert(sample(now, 0) == BRUSH_EVENT_NONE);
  assert(sample(1600, 0) == BRUSH_EVENT_NO_FEEDBACK);

  reset(0); teach(); BrushFeedback_SetDrive(1, 0); BrushFeedback_SetEnabled(1, 0);
  for (now = 1100; now < 1400; now += 100) assert(sample(now, -4) == BRUSH_EVENT_NONE);
  assert(sample(1400, -4) == BRUSH_EVENT_WRONG_DIRECTION);
  reset(0); teach(); BrushFeedback_SetDrive(1, 0); BrushFeedback_SetEnabled(1, 0);
  BrushFeedback_EncoderEdge(ab ^ 3);
  assert(sample(100, 0) == BRUSH_EVENT_NONE);

  reset(0); teach(); BrushFeedback_SetDrive(1, 0); BrushFeedback_SetEnabled(1, 0);
  for (now = 100; now <= 1100; now += 100) assert(sample(now, 4) == BRUSH_EVENT_NONE);
  for (now = 1200; now <= 1600; now += 100) assert(sample(now, 0) == BRUSH_EVENT_NONE);
  assert(sample(1700, 4) == BRUSH_EVENT_NONE); /* transient dip resets confirmation */
  assert(sample(1800, 0) == BRUSH_EVENT_NONE);

  reset(UINT32_MAX - 50U); teach();
  BrushFeedback_SetDrive(1, UINT32_MAX - 50U);
  assert(sample(49U, 4) == BRUSH_EVENT_NONE);
  assert(BrushFeedback_Snapshot(49U).rpm_x10 == 600);
  count = BrushFeedback_Count(); steps(-80);
  assert((int32_t)(count - BrushFeedback_Count()) == 2 * 40);

  reset(0); teach(); BrushFeedback_SetDrive(1, 0); BrushFeedback_SetEnabled(1, 0);
  for (now = 100; now <= 1100; now += 100) assert(sample(now, 4) == BRUSH_EVENT_NONE);
  steps(4); BrushFeedback_RecordAdc(3000, 1200);
  assert(BrushFeedback_Update(1200) == BRUSH_EVENT_NONE);
  steps(4); BrushFeedback_RecordAdc(3000, 1300);
  assert(BrushFeedback_Update(1300) == BRUSH_EVENT_NONE);
  steps(4); BrushFeedback_RecordAdc(3000, 1400);
  event = BrushFeedback_Update(1400);
#if BRUSH_CURRENT_ENABLE
  assert(BrushFeedback_Snapshot(1400).current_ma > 1000);
  assert(event == BRUSH_EVENT_HIGH_CURRENT);
  steps(4);
  assert(BrushFeedback_Update(1800) == BRUSH_EVENT_ADC_LOST);
  puts("PASS: calibrated current threshold confirmation and ADC freshness fault");
#else
  assert(BrushFeedback_Snapshot(1400).current_ma == -1);
  assert(event == BRUSH_EVENT_NONE); /* floating/unconfigured PA4 cannot trip */
#endif
  puts("PASS: real x4 decoder, invalid transitions, CPR teach, signed speed, startup grace, persistent stall, unplug/polarity and tick wrap");
  /* Isolated glitches remain uncounted, window expires by time, not loop count. */
  reset(0); BrushFeedback_SetDrive(1,0); BrushFeedback_ArmForCollection(0);
  for (now=100; now<=2000; now+=100)
  {
    if (now==100 || now==300) { ab ^= 3; BrushFeedback_EncoderEdge(ab); }
    assert(sample(now,4)==BRUSH_EVENT_NONE);
  }
  assert(BrushFeedback_Count()==80 && BrushFeedback_Errors()==2);
  assert(BrushFeedback_Snapshot(now).error_window==0);
  /* Dense burst faults even during startup. Invalid transitions add no travel. */
  reset(0); BrushFeedback_SetDrive(1,0); BrushFeedback_ArmForCollection(0);
  for (count=0; count<BRUSH_ERROR_WINDOW_LIMIT; count++) { ab^=3; BrushFeedback_EncoderEdge(ab); }
  assert(sample(100,0)==BRUSH_EVENT_ENCODER_ERRORS);
  assert(BrushFeedback_Count()==0 && BrushFeedback_Snapshot(100).encoder_fault);
  /* Sparse but consecutive bad samples also fault. */
  reset(0); BrushFeedback_SetDrive(1,0); BrushFeedback_ArmForCollection(0);
  for(count=1; count<=BRUSH_ERROR_CONSECUTIVE_LIMIT; count++)
  {
    ab^=3; BrushFeedback_EncoderEdge(ab);
    event=sample(count*100,4);
    assert(event==(count==BRUSH_ERROR_CONSECUTIVE_LIMIT ? BRUSH_EVENT_ENCODER_ERRORS : BRUSH_EVENT_NONE));
  }
  /* Disarm does not erase diagnostics; rearm starts a fresh window. */
  BrushFeedback_SetEnabled(0,300);
  assert(BrushFeedback_Snapshot(300).encoder_fault);
  BrushFeedback_ArmForCollection(300);
  assert(BrushFeedback_Snapshot(300).error_window==0);
  assert(BrushFeedback_Snapshot(300).last_event==BRUSH_EVENT_NONE);
  /* Tolerating a glitch must not invent valid pulses or suppress missing-feedback protection. */
  reset(0); BrushFeedback_SetDrive(1,0); BrushFeedback_ArmForCollection(0);
  ab^=3; BrushFeedback_EncoderEdge(ab);
  for(now=100; now<1600; now+=100) assert(sample(now,0)==BRUSH_EVENT_NONE);
  assert(sample(1600,0)==BRUSH_EVENT_NO_FEEDBACK);
  /* Tick wrap and long scheduling gap expire windows/reset consecutive streak. */
  reset(UINT32_MAX-50); BrushFeedback_SetDrive(1,UINT32_MAX-50); BrushFeedback_ArmForCollection(UINT32_MAX-50);
  ab^=3; BrushFeedback_EncoderEdge(ab); assert(sample(49,4)==BRUSH_EVENT_NONE);
  ab^=3; BrushFeedback_EncoderEdge(ab); assert(sample(1249,4)==BRUSH_EVENT_NONE);
  assert(BrushFeedback_Snapshot(1249).error_window==1 && BrushFeedback_Snapshot(1249).error_streak==1);
  puts("PASS: isolated errors tolerated without fabricated pulses, dense/streak faults, window expiry, rearm, missing feedback and tick wrap");
  return 0;
}
