#include "vision_protocol.h"

#include <stdio.h>
#include <string.h>

#define VISION_RX_RING_SIZE 512U
#define VISION_LINE_SIZE    128U

static volatile uint8_t rx_ring[VISION_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t protocol_error_count;

static char line_buffer[VISION_LINE_SIZE];
static uint16_t line_length;

static VisionFrame_t building_frame;
static uint8_t building_active;
static uint8_t expected_target_count;
static VisionFrame_t completed_frame;
static uint8_t completed_frame_ready;
static uint8_t completed_wall_state;
static uint8_t completed_wall_state_ready;
static VisionBlackZone_t completed_black_zone;
static uint8_t completed_black_zone_ready;

/* Parse a decimal string into a fixed-point integer without pulling floating
 * point scanf support into the STM32 image.  scale must be a power of ten:
 * 1000 converts normalized camera coordinates, 10 converts percent values. */
static uint8_t VisionProtocol_ParseScaled(const char *text,
                                          uint32_t scale,
                                          int32_t minimum,
                                          int32_t maximum,
                                          int32_t *result)
{
  uint32_t whole = 0U;
  uint32_t fraction = 0U;
  uint32_t place = scale;
  int32_t value;
  int32_t sign = 1;
  uint8_t digit_seen = 0U;

  if ((text == NULL) || (result == NULL) || (scale == 0U)) return 0U;
  if (*text == '-')
  {
    sign = -1;
    text++;
  }
  else if (*text == '+') text++;

  while ((*text >= '0') && (*text <= '9'))
  {
    digit_seen = 1U;
    if (whole > 1000000U) return 0U;
    whole = whole * 10U + (uint32_t)(*text - '0');
    text++;
  }
  if (digit_seen == 0U) return 0U;

  if (*text == '.')
  {
    text++;
    while ((*text >= '0') && (*text <= '9'))
    {
      if (place > 1U)
      {
        place /= 10U;
        fraction += (uint32_t)(*text - '0') * place;
      }
      else if (*text != '0') return 0U;
      text++;
    }
  }
  if (*text != '\0') return 0U;
  if (whole > 1000000U) return 0U;
  value = (int32_t)(whole * scale + fraction) * sign;
  if ((value < minimum) || (value > maximum)) return 0U;
  *result = value;
  return 1U;
}

static uint8_t VisionProtocol_ParseUnsigned(const char *text,
                                            uint32_t maximum,
                                            uint32_t *result)
{
  uint32_t value = 0U;
  uint8_t digit_seen = 0U;
  if ((text == NULL) || (result == NULL)) return 0U;
  while ((*text >= '0') && (*text <= '9'))
  {
    uint32_t digit = (uint32_t)(*text - '0');
    digit_seen = 1U;
    if ((value > maximum / 10U) ||
        ((value == maximum / 10U) && (digit > maximum % 10U)))
      return 0U;
    value = value * 10U + digit;
    text++;
  }
  if ((digit_seen == 0U) || (*text != '\0')) return 0U;
  *result = value;
  return 1U;
}

static uint8_t VisionProtocol_ParseBlack(char *payload)
{
  char *fields[8];
  uint8_t count = 0U;
  char *cursor = payload;
  uint32_t seen, width, height, area, coverage;
  int32_t centre_x, centre_y, scaled_coverage;

  if ((payload == NULL) || (payload[0] != 'B') || (payload[1] != ','))
    return 0U;

  fields[count++] = cursor;
  while ((*cursor != '\0') && (count < 8U))
  {
    if (*cursor == ',')
    {
      *cursor = '\0';
      fields[count++] = cursor + 1;
    }
    cursor++;
  }
  if ((count != 8U) || (*fields[0] != 'B') || (fields[0][1] != '\0'))
    return 0U;

  if (!VisionProtocol_ParseUnsigned(fields[1], 1U, &seen) ||
      !VisionProtocol_ParseScaled(fields[2], 1000U, -1000, 1000,
                                  &centre_x) ||
      !VisionProtocol_ParseScaled(fields[3], 1000U, -1000, 1000,
                                  &centre_y) ||
      !VisionProtocol_ParseUnsigned(fields[4], 2000U, &width) ||
      !VisionProtocol_ParseUnsigned(fields[5], 2000U, &height) ||
      !VisionProtocol_ParseUnsigned(fields[6], 0xFFFFFFFFU, &area) ||
      !VisionProtocol_ParseScaled(fields[7], 10U, 0, 1000,
                                  &scaled_coverage))
    return 0U;

  coverage = (uint32_t)scaled_coverage;
  completed_black_zone.seen = (uint8_t)seen;
  completed_black_zone.center_x_milli = (int16_t)centre_x;
  completed_black_zone.center_y_milli = (int16_t)centre_y;
  completed_black_zone.width_px = (uint16_t)width;
  completed_black_zone.height_px = (uint16_t)height;
  completed_black_zone.area_px = area;
  completed_black_zone.coverage_x10 = (uint16_t)coverage;
  completed_black_zone_ready = 1U;
  return 1U;
}

static uint8_t VisionProtocol_Checksum(const char *payload)
{
  uint8_t checksum = 0U;

  while (*payload != '\0')
  {
    checksum ^= (uint8_t)*payload;
    payload++;
  }
  return checksum;
}

static int8_t VisionProtocol_HexValue(char character)
{
  if ((character >= '0') && (character <= '9'))
  {
    return (int8_t)(character - '0');
  }
  if ((character >= 'A') && (character <= 'F'))
  {
    return (int8_t)(character - 'A' + 10);
  }
  if ((character >= 'a') && (character <= 'f'))
  {
    return (int8_t)(character - 'a' + 10);
  }
  return -1;
}

static uint8_t VisionProtocol_ValidateLine(char *line, char **payload)
{
  char *separator;
  int8_t high_nibble;
  int8_t low_nibble;
  uint8_t received_checksum;

  if ((line == NULL) || (line[0] != '$'))
  {
    return 0U;
  }

  separator = strrchr(line, '*');
  if ((separator == NULL) || (separator[1] == '\0') ||
      (separator[2] == '\0') || (separator[3] != '\0'))
  {
    return 0U;
  }

  high_nibble = VisionProtocol_HexValue(separator[1]);
  low_nibble = VisionProtocol_HexValue(separator[2]);
  if ((high_nibble < 0) || (low_nibble < 0))
  {
    return 0U;
  }

  received_checksum = (uint8_t)(((uint8_t)high_nibble << 4U) |
                                (uint8_t)low_nibble);
  *separator = '\0';
  *payload = &line[1];

  return VisionProtocol_Checksum(*payload) == received_checksum;
}

static void VisionProtocol_ParsePayload(char *payload)
{
  unsigned int sequence;
  unsigned int count;
  unsigned int index;
  unsigned int color;
  int lateral_mm;
  unsigned int forward_mm;
  unsigned int quality;
  unsigned long pixels;
  unsigned int wall_hit;
  if ((payload[0] == 'B') && (payload[1] == ','))
  {
    if (VisionProtocol_ParseBlack(payload) == 0U) protocol_error_count++;
    return;
  }

  if ((sscanf(payload, "W,%u", &wall_hit) == 1) && (wall_hit <= 1U))
  {
    completed_wall_state = (uint8_t)wall_hit;
    completed_wall_state_ready = 1U;
    return;
  }

  if (sscanf(payload, "F,%u,%u", &sequence, &count) == 2)
  {
    if (count > VISION_MAX_TARGETS)
    {
      protocol_error_count++;
      building_active = 0U;
      return;
    }

    memset(&building_frame, 0, sizeof(building_frame));
    building_frame.sequence = (uint16_t)sequence;
    expected_target_count = (uint8_t)count;
    building_active = 1U;
    return;
  }

  if (sscanf(payload, "T,%u,%u,%u,%d,%u,%u,%lu",
             &sequence, &index, &color, &lateral_mm, &forward_mm,
             &quality, &pixels) == 7)
  {
    VisionTarget_t *target;

    if ((building_active == 0U) ||
        ((uint16_t)sequence != building_frame.sequence) ||
        (index != building_frame.target_count) ||
        (building_frame.target_count >= VISION_MAX_TARGETS) ||
        ((color != VISION_COLOR_RED) && (color != VISION_COLOR_YELLOW)) ||
        (lateral_mm < -32768) || (lateral_mm > 32767) ||
        (forward_mm > 10000U) || (quality > 100U))
    {
      protocol_error_count++;
      return;
    }

    target = &building_frame.targets[building_frame.target_count];
    target->color = (uint8_t)color;
    target->lateral_mm = (int16_t)lateral_mm;
    target->forward_mm = (uint16_t)forward_mm;
    target->quality = (uint8_t)quality;
    target->pixels = (uint32_t)pixels;
    building_frame.target_count++;
    return;
  }

  if (sscanf(payload, "E,%u", &sequence) == 1)
  {
    if ((building_active != 0U) &&
        ((uint16_t)sequence == building_frame.sequence) &&
        (building_frame.target_count == expected_target_count))
    {
      completed_frame = building_frame;
      completed_frame_ready = 1U;
    }
    else
    {
      protocol_error_count++;
    }
    building_active = 0U;
    return;
  }

  /* Control/status text and malformed lines are intentionally ignored here. */
}

void VisionProtocol_Init(void)
{
  rx_head = 0U;
  rx_tail = 0U;
  protocol_error_count = 0U;
  line_length = 0U;
  building_active = 0U;
  expected_target_count = 0U;
  completed_frame_ready = 0U;
  completed_wall_state = 0U;
  completed_wall_state_ready = 0U;
  completed_black_zone_ready = 0U;
  memset(&building_frame, 0, sizeof(building_frame));
  memset(&completed_frame, 0, sizeof(completed_frame));
  memset(&completed_black_zone, 0, sizeof(completed_black_zone));
}

void VisionProtocol_RxByteFromISR(uint8_t byte)
{
  uint16_t next_head = (uint16_t)((rx_head + 1U) % VISION_RX_RING_SIZE);

  if (next_head == rx_tail)
  {
    protocol_error_count++;
    return;
  }

  rx_ring[rx_head] = byte;
  rx_head = next_head;
}

void VisionProtocol_Process(void)
{
  while (rx_tail != rx_head)
  {
    uint8_t byte = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % VISION_RX_RING_SIZE);

    if (byte == '\n')
    {
      char *payload = NULL;
      line_buffer[line_length] = '\0';

      if (VisionProtocol_ValidateLine(line_buffer, &payload) != 0U)
      {
        VisionProtocol_ParsePayload(payload);
      }
      else if (line_length != 0U)
      {
        protocol_error_count++;
      }
      line_length = 0U;
    }
    else if (byte != '\r')
    {
      if (line_length < (VISION_LINE_SIZE - 1U))
      {
        line_buffer[line_length++] = (char)byte;
      }
      else
      {
        line_length = 0U;
        protocol_error_count++;
      }
    }
  }
}

uint8_t VisionProtocol_GetFrame(VisionFrame_t *frame)
{
  if ((frame == NULL) || (completed_frame_ready == 0U))
  {
    return 0U;
  }

  *frame = completed_frame;
  completed_frame_ready = 0U;
  return 1U;
}

uint8_t VisionProtocol_GetWallState(uint8_t *wall_hit)
{
  if ((wall_hit == NULL) || (completed_wall_state_ready == 0U))
  {
    return 0U;
  }

  *wall_hit = completed_wall_state;
  completed_wall_state_ready = 0U;
  return 1U;
}

uint8_t VisionProtocol_GetBlackZone(VisionBlackZone_t *black_zone)
{
  if ((black_zone == NULL) || (completed_black_zone_ready == 0U))
  {
    return 0U;
  }
  *black_zone = completed_black_zone;
  completed_black_zone_ready = 0U;
  return 1U;
}

uint32_t VisionProtocol_GetErrorCount(void)
{
  return protocol_error_count;
}

uint16_t VisionProtocol_FormatCommand(char *buffer, uint16_t buffer_size,
                                      char mode, uint8_t color)
{
  char payload[16];
  int payload_length;
  int packet_length;
  uint8_t checksum;

  if ((buffer == NULL) || (buffer_size == 0U))
  {
    return 0U;
  }

  payload_length = snprintf(payload, sizeof(payload), "C,%c,%u",
                            mode, (unsigned int)color);
  if ((payload_length <= 0) || ((uint16_t)payload_length >= sizeof(payload)))
  {
    return 0U;
  }

  checksum = VisionProtocol_Checksum(payload);
  packet_length = snprintf(buffer, buffer_size, "$%s*%02X\r\n",
                           payload, (unsigned int)checksum);
  if ((packet_length <= 0) || ((uint16_t)packet_length >= buffer_size))
  {
    return 0U;
  }
  return (uint16_t)packet_length;
}
