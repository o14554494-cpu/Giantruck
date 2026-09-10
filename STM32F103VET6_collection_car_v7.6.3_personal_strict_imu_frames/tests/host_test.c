#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "path_planner.h"
#include "mission_extension.h"
#include "coverage_path.h"
#include "imu_navigation.h"
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
  VisionBlackZone_t black;
  uint8_t wall_hit;

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

  FeedPayload("W,1");
  VisionProtocol_Process();
  assert(VisionProtocol_GetWallState(&wall_hit) == 1U);
  assert(wall_hit == 1U);
  assert(VisionProtocol_GetWallState(&wall_hit) == 0U);
  FeedPayload("W,0");
  VisionProtocol_Process();
  assert(VisionProtocol_GetWallState(&wall_hit) == 1U && wall_hit == 0U);
  assert(VisionProtocol_GetErrorCount() == 0U);

  FeedPayload("B,1,-0.250,0.125,120,80,6400,30.5");
  VisionProtocol_Process();
  assert(VisionProtocol_GetBlackZone(&black) == 1U);
  assert(black.seen == 1U && black.center_x_milli == -250);
  assert(black.center_y_milli == 125 && black.width_px == 120U);
  assert(black.height_px == 80U && black.area_px == 6400U);
  assert(black.coverage_x10 == 305U);
  assert(VisionProtocol_GetBlackZone(&black) == 0U);

  FeedPayload("B,0,0,0,0,0,0,0.0");
  VisionProtocol_Process();
  assert(VisionProtocol_GetBlackZone(&black) == 1U);
  assert(black.seen == 0U && black.coverage_x10 == 0U);

  FeedPayload("B,1,1.001,0.000,10,10,100,3.0");
  VisionProtocol_Process();
  assert(VisionProtocol_GetBlackZone(&black) == 0U);
  assert(VisionProtocol_GetErrorCount() == 1U);
}

static void TestImuNavigation(void)
{
  ImuNavigation_t nav;
  JY901Snapshot snapshot = {0};
  uint32_t tick;
  ImuNavigation_Reset(&nav);
  snapshot.angle_frames = 1U; snapshot.angle_tick = 100U; snapshot.angle[2] = 0;
  assert(ImuNavigation_Update(&nav, &snapshot, 100U, 0.25f));
  assert(ImuNavigation_Fresh(&nav, 100U));
  assert(nav.heading_rad > 0.24f && nav.heading_rad < 0.26f);
  snapshot.angle_frames++; snapshot.angle_tick = 200U; snapshot.angle[2] = 2731; /* +15 deg */
  assert(ImuNavigation_Update(&nav, &snapshot, 200U, 0.25f));
  snapshot.angle_frames++; snapshot.angle_tick = 300U; snapshot.angle[2] = 5461; /* +30 deg total */
  assert(ImuNavigation_Update(&nav, &snapshot, 300U, 0.77f));
  assert(nav.heading_rad > 0.77f && nav.heading_rad < 0.78f);
  assert(!ImuNavigation_Fresh(&nav, 300U + IMU_NAV_STALE_MS + 1U));

  /* A stale link must not accept its first recovery frame.  Five consistent
   * frames agreeing with encoder odometry are required. */
  snapshot.angle_frames++; snapshot.angle_tick = 1000U; snapshot.angle[2] = -10923;
  assert(ImuNavigation_Update(&nav, &snapshot, 1000U, -0.80f));
  assert(nav.heading_rad > 0.77f && nav.heading_rad < 0.78f);
  assert(!ImuNavigation_Fresh(&nav, 1000U));
  assert(nav.stale_recoveries == 1U);
  assert(nav.reference_yaw_raw == 0); /* Startup yaw survives stale recovery. */
  for (tick = 1010U; tick <= 1040U; tick += 10U)
  {
    snapshot.angle_frames++;
    snapshot.angle_tick = tick;
    snapshot.angle[2] = -10923;
    assert(ImuNavigation_Update(&nav, &snapshot, tick, -0.80f));
  }
  assert(nav.heading_rad < -0.79f && nav.heading_rad > -0.80f);
  assert(ImuNavigation_Fresh(&nav, 1040U));
  assert(nav.recovery_accepts == 1U);

  /* A persistent checksum-valid 90-degree jump must never legitimise itself
   * merely by being repeated.  The previous implementation accepted frame 2
   * because frame 1 had already overwritten last_yaw_raw. */
  for (tick = 1050U; tick <= 1110U; tick += 10U)
  {
    snapshot.angle_frames++;
    snapshot.angle_tick = tick;
    snapshot.angle[2] = 5461; /* 90 degrees from last accepted -60 degrees. */
    assert(ImuNavigation_Update(&nav, &snapshot, tick, -0.80f));
  }
  assert(nav.heading_rad < -0.79f && nav.heading_rad > -0.80f);
  assert(nav.last_yaw_raw == -10923);
  assert(nav.recovery_active == 1U);
  assert(nav.recovery_samples == 1U);
  assert(nav.recovery_resets >= 5U);
  assert(nav.reference_yaw_raw == 0);

  /* Returning to the last trustworthy yaw is accepted immediately. */
  snapshot.angle_frames++;
  snapshot.angle_tick = 1120U;
  snapshot.angle[2] = -10923;
  assert(ImuNavigation_Update(&nav, &snapshot, 1120U, -0.80f));
  assert(nav.recovery_active == 0U);
  assert(nav.last_yaw_raw == -10923);
}

static void TestDualImuReferences(void)
{
  ImuNavigation_t startup;
  ImuNavigation_t local;
  JY901Snapshot snapshot = {0};

  ImuNavigation_Reset(&startup);
  ImuNavigation_Reset(&local);
  snapshot.angle_frames = 1U;
  snapshot.angle_tick = 100U;
  snapshot.angle[2] = 4000;
  assert(ImuNavigation_SetReference(&startup, &snapshot, 100U, 0.0f));
  assert(ImuNavigation_SetReference(&local, &snapshot, 100U, 0.0f));

  snapshot.angle_frames = 2U;
  snapshot.angle_tick = 200U;
  snapshot.angle[2] = 6000;
  assert(ImuNavigation_Update(&startup, &snapshot, 200U, 0.0f));
  assert(ImuNavigation_Update(&local, &snapshot, 200U, 0.0f));
  assert(startup.heading_rad > 0.19f && startup.heading_rad < 0.20f);

  /* Refreshing the local zero must never alter the startup zero. */
  assert(ImuNavigation_SetReference(&local, &snapshot, 200U, 0.0f));
  assert(startup.reference_yaw_raw == 4000);
  assert(local.reference_yaw_raw == 6000);
  assert(local.heading_rad == 0.0f);

  snapshot.angle_frames = 3U;
  snapshot.angle_tick = 300U;
  snapshot.angle[2] = 6500;
  assert(ImuNavigation_Update(&startup, &snapshot, 300U, 0.0f));
  assert(ImuNavigation_Update(&local, &snapshot, 300U, 0.0f));
  assert(startup.heading_rad > local.heading_rad);
  assert(startup.reference_yaw_raw == 4000);
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
  CoverageOutput_t coverage_output;
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
  pose.heading_rad = MISSION_UNLOAD_HEADING_RAD;
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

  MissionExtension_Reset(); MissionExtension_UseUpperRightUnload();
  MissionExtension_ForceStartUnload(0U);
  pose = (RobotPose_t){MISSION_Q_UNLOAD_STAGE_X_MM, MISSION_Q_UNLOAD_STAGE_Y_MM,
                       MISSION_Q_UNLOAD_HEADING_RAD};
  (void)MissionExtension_UpdateUnload(&pose, 0U, 10U);
  (void)MissionExtension_UpdateUnload(&pose, 0U, 20U);
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_REVERSE);
  (void)MissionExtension_UpdateUnload(&pose, MISSION_REAR_STOP_MM, 30U);
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_REVERSE);
  (void)MissionExtension_UpdateUnload(&pose, MISSION_REAR_STOP_MM, 40U);
  assert(MissionExtension_GetUnloadState() == MISSION_UNLOAD_EJECT);

  /* Q return-origin sequence: all rear thresholds require three NEW samples. */
  Coverage_Reset();
  Coverage_StartReturn(0U);
  pose = (RobotPose_t){-1200.0f, -700.0f,
                       COVERAGE_OPPOSITE_HEADING_RAD - 1.5f};
  Coverage_SetInputs(0U, 0U, 20U, 1U);
  coverage_output = Coverage_Update(&pose);
  assert(coverage_output.left_percent == -28 &&
         coverage_output.right_percent == 28);
  pose.heading_rad = COVERAGE_OPPOSITE_HEADING_RAD - 0.10f;
  Coverage_SetInputs(0U, 0U, 40U, 1U);
  coverage_output = Coverage_Update(&pose);
  assert(coverage_output.left_percent == -8 &&
         coverage_output.right_percent == 8);

  Coverage_Reset();
  Coverage_StartReturn(0U);
  pose = (RobotPose_t){-1200.0f, -700.0f, COVERAGE_OPPOSITE_HEADING_RAD};
  Coverage_SetInputs(0U, 0U, 0U, 1U);
  (void)Coverage_Update(&pose);
  assert(Coverage_State() == COVERAGE_RETURN_REVERSE_Y);
  /* Sub-deadband IMU noise must keep equal reverse wheel commands. */
  pose.heading_rad = COVERAGE_OPPOSITE_HEADING_RAD - 0.01f;
  Coverage_SetInputs(500U, 20U, 20U, 1U);
  coverage_output = Coverage_Update(&pose);
  assert(coverage_output.left_percent == -12 &&
         coverage_output.right_percent == -12);
  pose.heading_rad = COVERAGE_OPPOSITE_HEADING_RAD + 0.01f;
  Coverage_SetInputs(500U, 40U, 40U, 1U);
  coverage_output = Coverage_Update(&pose);
  assert(coverage_output.left_percent == -12 &&
         coverage_output.right_percent == -12);
  pose.heading_rad = COVERAGE_OPPOSITE_HEADING_RAD - 0.10f;
  Coverage_SetInputs(500U, 60U, 60U, 1U);
  coverage_output = Coverage_Update(&pose);
  assert(coverage_output.left_percent < coverage_output.right_percent);
  assert(coverage_output.left_percent < 0 && coverage_output.right_percent < 0);
  /* A single opposite noisy sample ramps to neutral correction instead of
   * immediately exchanging the faster wheel. */
  pose.heading_rad = COVERAGE_OPPOSITE_HEADING_RAD + 0.10f;
  Coverage_SetInputs(500U, 80U, 80U, 1U);
  coverage_output = Coverage_Update(&pose);
  assert(coverage_output.left_percent == coverage_output.right_percent);
  pose.heading_rad = COVERAGE_OPPOSITE_HEADING_RAD;
  Coverage_SetInputs(150U, 100U, 100U, 1U); (void)Coverage_Update(&pose);
  Coverage_SetInputs(150U, 200U, 200U, 1U); (void)Coverage_Update(&pose);
  assert(Coverage_State() == COVERAGE_RETURN_REVERSE_Y);
  Coverage_SetInputs(150U, 300U, 300U, 1U); (void)Coverage_Update(&pose);
  assert(Coverage_State() == COVERAGE_RETURN_TURN_CW_90);
  pose.heading_rad = COVERAGE_POSITIVE_X_HEADING_RAD;
  Coverage_SetInputs(500U, 400U, 400U, 1U); (void)Coverage_Update(&pose);
  assert(Coverage_State() == COVERAGE_RETURN_REVERSE_X);
  Coverage_SetInputs(220U, 500U, 500U, 1U); (void)Coverage_Update(&pose);
  Coverage_SetInputs(220U, 600U, 600U, 1U); (void)Coverage_Update(&pose);
  Coverage_SetInputs(220U, 700U, 700U, 1U); (void)Coverage_Update(&pose);
  pose.heading_rad = COVERAGE_DIAGONAL_HEADING_RAD;
  Coverage_SetInputs(500U, 800U, 800U, 1U); (void)Coverage_Update(&pose);
  assert(Coverage_State() == COVERAGE_ADVANCE);

  /* Two 1000 mm advances, each followed by six measured 60-degree steps. */
  {
    uint32_t tick = 800U;
    uint8_t pass;
    for (pass = 0U; pass < COVERAGE_SCAN_PASSES; pass++)
    {
      uint8_t step;
      pose.x_mm += 1005.0f * Planner_Cos(COVERAGE_DIAGONAL_HEADING_RAD);
      pose.y_mm += 1005.0f * Planner_Sin(COVERAGE_DIAGONAL_HEADING_RAD);
      tick += 100U;
      Coverage_SetInputs(500U, tick, tick, 1U); (void)Coverage_Update(&pose);
      assert(Coverage_State() == COVERAGE_SCAN_OBSERVE);
      for (step = 0U; step < COVERAGE_SCAN_STEPS; step++)
      {
        tick += COVERAGE_SCAN_OBSERVE_MS + 1U;
        Coverage_SetInputs(500U, tick, tick, 1U); (void)Coverage_Update(&pose);
        assert(Coverage_State() == COVERAGE_SCAN_TURN);
        pose.heading_rad = Planner_NormalizeAngle(
            pose.heading_rad + COVERAGE_SCAN_STEP_RAD);
        tick += 100U;
        Coverage_SetInputs(500U, tick, tick, 1U); (void)Coverage_Update(&pose);
        assert(Coverage_State() == COVERAGE_SCAN_TURN);
        tick += COVERAGE_SCAN_SETTLE_MS + 1U;
        Coverage_SetInputs(500U, tick, tick, 1U); (void)Coverage_Update(&pose);
        assert(Coverage_State() == COVERAGE_SCAN_OBSERVE);
      }
      tick += COVERAGE_SCAN_OBSERVE_MS + 1U;
      Coverage_SetInputs(500U, tick, tick, 1U); (void)Coverage_Update(&pose);
      assert(Coverage_State() == (pass == 0U ? COVERAGE_ADVANCE :
                                                   COVERAGE_UNLOAD_ALIGN_INITIAL));
    }

    pose.heading_rad = COVERAGE_INITIAL_HEADING_RAD;
    tick += 100U; Coverage_SetInputs(500U, tick, tick, 1U); (void)Coverage_Update(&pose);
    assert(Coverage_State() == COVERAGE_UNLOAD_REVERSE_Y);
    tick += 100U; Coverage_SetInputs(100U, tick, tick, 1U); (void)Coverage_Update(&pose);
    tick += 100U; Coverage_SetInputs(100U, tick, tick, 1U); (void)Coverage_Update(&pose);
    tick += 100U; Coverage_SetInputs(100U, tick, tick, 1U); (void)Coverage_Update(&pose);
    assert(Coverage_State() == COVERAGE_UNLOAD_TURN_CW_90);
    pose.heading_rad = COVERAGE_UNLOAD_X_HEADING_RAD;
    tick += 100U; Coverage_SetInputs(500U, tick, tick, 1U); (void)Coverage_Update(&pose);
    assert(Coverage_State() == COVERAGE_UNLOAD_REVERSE_X);
    tick += 100U; Coverage_SetInputs(220U, tick, tick, 1U); (void)Coverage_Update(&pose);
    tick += 100U; Coverage_SetInputs(220U, tick, tick, 1U); (void)Coverage_Update(&pose);
    tick += 100U; Coverage_SetInputs(220U, tick, tick, 1U); (void)Coverage_Update(&pose);
    pose.heading_rad = COVERAGE_UNLOAD_FINAL_HEADING_RAD;
    tick += 100U; Coverage_SetInputs(500U, tick, tick, 1U);
    assert(Coverage_Update(&pose).finished == 1U);
    assert(Coverage_State() == COVERAGE_FINISHED);
  }

  /* A large reverse-heading error latches the stationary pivot until the
   * tighter exit band is reached; it cannot alternate with reverse travel. */
  Coverage_Reset(); Coverage_StartReturn(0U);
  pose.heading_rad = COVERAGE_OPPOSITE_HEADING_RAD;
  Coverage_SetInputs(0U, 0U, 0U, 1U); (void)Coverage_Update(&pose);
  pose.heading_rad = COVERAGE_OPPOSITE_HEADING_RAD - 0.23f;
  Coverage_SetInputs(500U, 20U, 20U, 1U);
  coverage_output = Coverage_Update(&pose);
  assert(coverage_output.left_percent < 0 && coverage_output.right_percent > 0);
  pose.heading_rad = COVERAGE_OPPOSITE_HEADING_RAD - 0.10f;
  Coverage_SetInputs(500U, 40U, 40U, 1U);
  coverage_output = Coverage_Update(&pose);
  assert(coverage_output.left_percent < 0 && coverage_output.right_percent > 0);
  pose.heading_rad = COVERAGE_OPPOSITE_HEADING_RAD - 0.04f;
  Coverage_SetInputs(500U, 60U, 60U, 1U);
  coverage_output = Coverage_Update(&pose);
  assert(coverage_output.left_percent < 0 && coverage_output.right_percent < 0);

  /* Reverse states fail closed when IMU yaw is stale. */
  Coverage_Reset(); Coverage_StartReturn(0U);
  pose.heading_rad = COVERAGE_OPPOSITE_HEADING_RAD;
  Coverage_SetInputs(0U, 0U, 0U, 1U); (void)Coverage_Update(&pose);
  assert(Coverage_State() == COVERAGE_RETURN_REVERSE_Y);
  Coverage_SetInputs(500U, 1600U, 1600U, 0U);
  assert(Coverage_Update(&pose).fault == 1U);

  MissionExtension_Reset();
  pose.x_mm = 1500.0f;
  pose.y_mm = 1000.0f;
  pose.heading_rad = 0.0f;
  assert(MissionExtension_ApplyForwardSafety(&pose, 150U, 0U, 30, 30,
                                             &safe_left, &safe_right,
                                             &replan) == 0U);
  assert(MissionExtension_ApplyForwardSafety(&pose, 80U, 100U, 30, 30,
                                             &safe_left, &safe_right,
                                             &replan) == 0U);
  assert(MissionExtension_ApplyForwardSafety(&pose, 80U, 200U, 30, 30,
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
  TestImuNavigation();
  TestDualImuReferences();
  TestEmptyVisionFrame();
  TestTargetMap();
  TestPlanner();
  TestMissionExtension();
  TestCombatStrategy();
  puts("host tests passed");
  return 0;
}
