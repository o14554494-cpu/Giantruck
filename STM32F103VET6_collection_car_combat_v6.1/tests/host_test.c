#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "path_planner.h"
#include "mission_extension.h"
#include "combat_strategy.h"
#include "target_map.h"
#include "vision_protocol.h"

static void FeedPayload(const char *payload)
{
  char packet[160];
  unsigned int checksum = 0U;
  size_t index;

  for (index = 0U; payload[index] != '\0'; index++)
  {
    checksum ^= (unsigned int)(unsigned char)payload[index];
  }

  (void)snprintf(packet, sizeof(packet), "$%s*%02X\r\n", payload, checksum);
  for (index = 0U; packet[index] != '\0'; index++)
  {
    VisionProtocol_RxByteFromISR((uint8_t)packet[index]);
  }
}

static void TestVisionProtocol(void)
{
  VisionFrame_t frame;

  VisionProtocol_Init();
  FeedPayload("F,42,2");
  FeedPayload("T,42,0,1,-35,620,81,1300");
  FeedPayload("T,42,1,2,44,750,70,900");
  FeedPayload("E,42");
  VisionProtocol_Process();

  assert(VisionProtocol_GetFrame(&frame) == 1U);
  assert(frame.sequence == 42U);
  assert(frame.target_count == 2U);
  assert(frame.targets[0].color == VISION_COLOR_RED);
  assert(frame.targets[0].lateral_mm == -35);
  assert(frame.targets[1].forward_mm == 750U);
  assert(VisionProtocol_GetFrame(&frame) == 0U);
  assert(VisionProtocol_GetErrorCount() == 0U);
}

static void TestTargetMap(void)
{
  TargetMap_t map;
  MapTarget_t candidates[TARGET_MAP_MAX_TARGETS];
  uint16_t first_id;
  uint16_t merged_id;

  TargetMap_Reset(&map);
  first_id = TargetMap_Upsert(&map, VISION_COLOR_RED,
                              500.0f, 100.0f, 80U, 100U);
  merged_id = TargetMap_Upsert(&map, VISION_COLOR_RED,
                               530.0f, 90.0f, 70U, 200U);
  assert(first_id != 0U);
  assert(merged_id == first_id);
  assert(TargetMap_GetCandidates(&map, candidates,
                                 TARGET_MAP_MAX_TARGETS) == 1U);

  TargetMap_MarkCollected(&map, first_id, 1000U);
  assert(TargetMap_CountAvailable(&map) == 0U);

  /* A near observation is ignored while the block passes under the camera. */
  assert(TargetMap_Upsert(&map, VISION_COLOR_RED,
                          510.0f, 95.0f, 75U, 1500U) == first_id);
  assert(TargetMap_CountAvailable(&map) == 0U);

  /* Continued visibility after the holdoff reopens a failed collection. */
  assert(TargetMap_Upsert(&map, VISION_COLOR_RED,
                          510.0f, 95.0f, 75U, 3101U) == first_id);
  assert(TargetMap_CountAvailable(&map) == 1U);

  /* A weak one-frame observation is not planned until it is confirmed. */
  TargetMap_Reset(&map);
  first_id = TargetMap_Upsert(&map, VISION_COLOR_YELLOW,
                              900.0f, -200.0f, 45U, 100U);
  assert(first_id != 0U);
  assert(TargetMap_CountAvailable(&map) == 0U);
  merged_id = TargetMap_Upsert(&map, VISION_COLOR_YELLOW,
                               920.0f, -190.0f, 50U, 300U);
  assert(merged_id == first_id);
  assert(TargetMap_CountAvailable(&map) == 1U);
}

static void TestEmptyVisionFrame(void)
{
  VisionFrame_t frame;

  VisionProtocol_Init();
  FeedPayload("F,99,0");
  FeedPayload("E,99");
  VisionProtocol_Process();

  assert(VisionProtocol_GetFrame(&frame) == 1U);
  assert(frame.sequence == 99U);
  assert(frame.target_count == 0U);
}

static void TestPlanner(void)
{
  RobotPose_t pose = {0.0f, 0.0f, 0.0f};
  MapTarget_t candidates[4];
  PlannerRoute_t route;
  uint8_t first;
  uint8_t second;

  memset(candidates, 0, sizeof(candidates));
  candidates[0] = (MapTarget_t){1U, VISION_COLOR_RED, 800.0f, 0.0f,
                                90U, 1U, 1U, 0U, 0U};
  candidates[1] = (MapTarget_t){2U, VISION_COLOR_YELLOW, 200.0f, 400.0f,
                                90U, 1U, 1U, 0U, 0U};
  candidates[2] = (MapTarget_t){3U, VISION_COLOR_RED, 1200.0f, 50.0f,
                                85U, 1U, 1U, 0U, 0U};
  candidates[3] = (MapTarget_t){4U, VISION_COLOR_YELLOW, 600.0f, 450.0f,
                                80U, 1U, 1U, 0U, 0U};

  Planner_BuildRoute(&pose, candidates, 4U, &route);
  assert(route.count == 4U);
  assert(route.estimated_cost_mm > 0.0f);
  assert(Planner_Sqrt(4.0f) > 1.99f && Planner_Sqrt(4.0f) < 2.01f);
  assert(Planner_Cos(0.0f) > 0.99f);

  for (first = 0U; first < route.count; first++)
  {
    for (second = (uint8_t)(first + 1U); second < route.count; second++)
    {
      assert(route.target_ids[first] != route.target_ids[second]);
    }
  }
}

static void TestMissionExtension(void)
{
  RobotPose_t pose;
  MissionDriveOutput_t output;
  int16_t safe_left;
  int16_t safe_right;
  uint8_t replan;

  MissionExtension_Reset();
  MissionExtension_SetInitialPose(&pose);
  assert(pose.x_mm == MISSION_START_X_MM);
  assert(MissionExtension_TargetInsideArena(500.0f, 500.0f) == 1U);
  assert(MissionExtension_TargetInsideArena(-10.0f, 500.0f) == 0U);
  assert(MissionExtension_TargetInsideArena(3100.0f, 500.0f) == 0U);

  MissionExtension_StartUnload(0U);
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_IDLE);
  MissionExtension_RecordCollected();
  MissionExtension_StartUnload(100U);
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_GO_STAGE);

  pose.x_mm = MISSION_UNLOAD_STAGE_X_MM;
  pose.y_mm = MISSION_UNLOAD_STAGE_Y_MM;
  pose.heading_rad = 0.0f;
  output = MissionExtension_UpdateUnload(&pose, 0U, 200U);
  assert(output.active == 1U);
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_ALIGN);
  (void)MissionExtension_UpdateUnload(&pose, 0U, 300U);
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_REVERSE);

  pose.x_mm = MISSION_UNLOAD_STOP_X_MM - 1.0f;
  (void)MissionExtension_UpdateUnload(&pose, 0U, 400U);
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_EJECT);
  /* EJECT 持续 5 秒：400ms 进入，5600ms 已超过 5 秒 */
  output = MissionExtension_UpdateUnload(&pose, 0U, 5600U);
  assert(output.finished == 1U);
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_DONE);

  MissionExtension_Reset();
  pose.x_mm = 1500.0f;
  pose.y_mm = 1000.0f;
  pose.heading_rad = 0.0f;
  assert(MissionExtension_ApplyForwardSafety(&pose, 150U, 0U, 30, 30,
                                             &safe_left, &safe_right,
                                             &replan) == 0U);
  assert(MissionExtension_ApplyForwardSafety(&pose, 150U, 100U, 30, 30,
                                             &safe_left, &safe_right,
                                             &replan) == 1U);
  assert(safe_left == 0 && safe_right == 0);
}

static void TestCombatStrategy(void)
{
  RobotPose_t pose = {1500.0f, 1000.0f, 0.0f};
  MapTarget_t candidates[2];
  CombatSelection_t selection;
  float patrol_x;
  float patrol_y;

  memset(candidates, 0, sizeof(candidates));
  candidates[0] = (MapTarget_t){11U, VISION_COLOR_RED, 1800.0f, 1000.0f,
                                80U, 2U, 1U, 0U, 0U};
  candidates[1] = (MapTarget_t){12U, VISION_COLOR_YELLOW, 2700.0f, 1700.0f,
                                90U, 2U, 1U, 0U, 0U};

  CombatStrategy_Start(1000U);
  assert(CombatStrategy_IsActive() == 1U);
  selection = CombatStrategy_SelectTarget(&pose, candidates, 2U, 1100U);
  assert(selection.valid == 1U);
  assert(selection.target_id == 11U); /* enemy steal bonus is bounded */

  CombatStrategy_RecordCollected(2000U);
  CombatStrategy_RecordCollected(3000U);
  assert(CombatStrategy_ShouldReturn(4000U) == 0U);
  CombatStrategy_RecordCollected(4000U);
  assert(CombatStrategy_ShouldReturn(5000U) == 0U);
  CombatStrategy_RecordCollected(5000U);
  assert(CombatStrategy_ShouldReturn(6000U) == 1U);
  CombatStrategy_RecordDeposit(6000U);
  assert(CombatStrategy_GetPayloadCount() == 0U);

  pose.x_mm = 2700.0f;
  pose.y_mm = 1700.0f;
  CombatStrategy_UpdateZone(&pose, 7000U);
  CombatStrategy_UpdateZone(&pose, 32001U);
  assert(CombatStrategy_OpponentDwellMs(32001U) >=
         COMBAT_MAX_OPPONENT_DWELL_MS);
  selection = CombatStrategy_SelectTarget(&pose, candidates, 2U, 32001U);
  assert(selection.valid == 1U);
  assert(selection.target_id == 11U); /* no new enemy target while exiting */
  CombatStrategy_GetPatrolWaypoint(&patrol_x, &patrol_y, 32001U);
  assert(patrol_x == 1500.0f && patrol_y == 1000.0f);

  pose.x_mm = 1500.0f;
  pose.y_mm = 1000.0f;
  CombatStrategy_UpdateZone(&pose, 33000U);
  assert(CombatStrategy_OpponentDwellMs(33000U) == 0U);

  candidates[0].x_mm = 200.0f;
  candidates[0].y_mm = 200.0f; /* own warehouse: never collect */
  selection = CombatStrategy_SelectTarget(&pose, candidates, 2U, 272000U);
  assert(selection.valid == 0U); /* other target is too far in endgame */
  CombatStrategy_GetPatrolWaypoint(&patrol_x, &patrol_y, 272000U);
  assert(patrol_x == 550.0f && patrol_y == 450.0f);

  assert(CombatStrategy_UpdateStuck(&pose, 1U, 34000U) == 0U);
  assert(CombatStrategy_UpdateStuck(&pose, 1U, 38001U) == 1U);
  assert(CombatStrategy_MatchFinished(301001U) == 1U);
  CombatStrategy_Stop();
  assert(CombatStrategy_IsActive() == 0U);
}

int main(void)
{
  TestVisionProtocol();
  TestEmptyVisionFrame();
  TestTargetMap();
  TestPlanner();
  TestMissionExtension();
  TestCombatStrategy();
  puts("host tests passed");
  return 0;
}
