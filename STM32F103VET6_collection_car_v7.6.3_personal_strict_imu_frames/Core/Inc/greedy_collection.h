#ifndef GREEDY_COLLECTION_H
#define GREEDY_COLLECTION_H

#include "path_planner.h"

/* Legacy distance-only memory API retained for regression/history. Current Q
 * uses greedy_local.h for one camera-relative target, plus the pickup counter
 * and scan/feed parameters below; it does not populate this global memory. */
#define GREEDY_CAPACITY 32U
#define GREEDY_BATCH_SIZE 10U
#define GREEDY_TARGET_TTL_MS 20000U
#define GREEDY_SELECT_INTERVAL_MS 300U
#define GREEDY_MERGE_RADIUS_MM 120.0f

/* Stop-and-observe search. Rotation deliberately uses wheel odometry only;
 * the IMU is reserved for keeping nominally straight travel straight. */
#define GREEDY_SCAN_STEP_RAD 1.0471976f /* 60 degrees */
#define GREEDY_SCAN_STEPS 6U
#define GREEDY_SCAN_TURN_MAX_MS 3000U
#define GREEDY_SCAN_OBSERVE_MS 300U

/* Logical commands through drive_pwm.h with the current 60% floor:
 * 38 -> 75% scan PWM; 32 -> 73% straight following/feed PWM. */
#define GREEDY_FORWARD_SPEED_PERCENT 32
#define GREEDY_SCAN_SPEED_PERCENT 38

/* Keep a real forward component while steering.  The previous one-wheel
 * arc command made the car orbit a side of the block.  A larger bounded
 * differential is needed because the 60% physical PWM floor compresses the
 * difference between small logical commands. */
#define GREEDY_STEER_GAIN 70.0f
#define GREEDY_STEER_MAX_PERCENT 20
#define GREEDY_ALIGN_SPEED_PERCENT 20
#define GREEDY_ALIGN_STEER_MAX_PERCENT 18

/* Commit while a close target is still visible, then feed past its estimated
 * position using signed wheel travel. Empty frames do not abort this motion;
 * a lost camera link still does. These distances need physical calibration. */
#define GREEDY_COLLECT_TRIGGER_MM 220U
#define GREEDY_COLLECT_EXTRA_MM 150.0f
#define GREEDY_COLLECT_TIMEOUT_MS 4000U

void Greedy_Reset(void);
void Greedy_ClearPending(void);
void Greedy_Observe(uint8_t color, float x, float y, uint8_t quality, uint32_t now);
MapTarget_t *Greedy_Find(uint16_t id, uint32_t now);
MapTarget_t *Greedy_Nearest(const RobotPose_t *pose, uint32_t now);
/* Sorted by current Euclidean distance; no route or weighted cost. */
uint8_t Greedy_List(const RobotPose_t *pose, uint32_t now,
                    uint16_t *ids, uint8_t capacity);
uint8_t Greedy_MarkCollected(uint16_t id);
void Greedy_Forget(uint16_t id);
uint8_t Greedy_CountCollected(void);
/* Q local mode counts completed feed motions without a map entry. */
uint8_t Greedy_RecordPickup(void);

#endif
