#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define VISION_MAX_TARGETS 8U

typedef enum
{
  VISION_COLOR_NONE = 0,
  VISION_COLOR_RED = 1,
  VISION_COLOR_YELLOW = 2
} VisionColor_t;

typedef struct
{
  uint8_t color;
  int16_t lateral_mm;       /* Positive is to the robot's right. */
  uint16_t forward_mm;      /* Positive is in front of the robot. */
  uint8_t quality;          /* 0..100 */
  uint32_t pixels;
} VisionTarget_t;

typedef struct
{
  uint16_t sequence;
  uint8_t target_count;
  VisionTarget_t targets[VISION_MAX_TARGETS];
} VisionFrame_t;

typedef struct
{
  uint8_t seen;
  int16_t center_x_milli;    /* OpenMV float -1..1 converted to x1000. */
  int16_t center_y_milli;    /* OpenMV float -1..1 converted to x1000. */
  uint16_t width_px;
  uint16_t height_px;
  uint32_t area_px;
  uint16_t coverage_x10;     /* OpenMV float percent converted to x10. */
} VisionBlackZone_t;

void VisionProtocol_Init(void);
void VisionProtocol_RxByteFromISR(uint8_t byte);
void VisionProtocol_Process(void);
uint8_t VisionProtocol_GetFrame(VisionFrame_t *frame);
/* Consume the newest OpenMV wall state ($W,0/$W,1). Returns 1 for a new
 * state packet and 0 when no unread wall packet is available. */
uint8_t VisionProtocol_GetWallState(uint8_t *wall_hit);
/* Consume the newest OpenMV black-zone packet ($B,...). */
uint8_t VisionProtocol_GetBlackZone(VisionBlackZone_t *black_zone);
uint32_t VisionProtocol_GetErrorCount(void);
uint16_t VisionProtocol_FormatCommand(char *buffer, uint16_t buffer_size,
                                      char mode, uint8_t color);

#ifdef __cplusplus
}
#endif

#endif /* VISION_PROTOCOL_H */
