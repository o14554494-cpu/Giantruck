#include "jy901.h"
#include <string.h>

static volatile JY901Snapshot data;
static uint8_t packet[11], used;
static uint32_t last_byte_tick;

static int16_t ReadSigned(const uint8_t *p)
{
  int32_t value = (int32_t)p[0] | ((int32_t)p[1] << 8);
  return (int16_t)(value >= 32768 ? value - 65536 : value);
}

void JY901_Init(void)
{
  const JY901Snapshot empty = {0};
  data = empty;
  used = 0;
  last_byte_tick = 0U;
}

void JY901_ResetStream(void) { used = 0; }

void JY901_RxByte(uint8_t byte, uint32_t now)
{
  uint8_t i, sum = 0;
  data.bytes++;
  if (used != 0U &&
      (uint32_t)(now - last_byte_tick) > JY901_INTERBYTE_TIMEOUT_MS)
  {
    used = 0U;
    data.stream_timeouts++;
  }
  last_byte_tick = now;
  if (!used && byte != 0x55U) return;
  packet[used++] = byte;
  if (used < sizeof(packet)) return;
  for (i = 0; i < 10U; i++) sum = (uint8_t)(sum + packet[i]);
  if (sum == packet[10] && packet[1] >= 0x50U && packet[1] <= 0x59U)
  {
    data.frames++;
    if (packet[1] == 0x53U)
    {
      for (i = 0; i < 3U; i++) data.angle[i] = ReadSigned(&packet[2U + 2U*i]);
      data.angle_tick = now;
      data.angle_frames++;
    }
    else if (packet[1] == 0x52U)
    {
      for (i = 0; i < 3U; i++) data.gyro[i] = ReadSigned(&packet[2U + 2U*i]);
      data.gyro_tick = now;
      data.gyro_frames++;
    }
    used = 0;
    return;
  }
  if (sum != packet[10]) data.checksum_errors++;
  else data.type_errors++;
  /* Retain an embedded header after noise, truncation or a dropped byte. */
  for (i = 1; i < sizeof(packet); i++)
    if (packet[i] == 0x55U) break;
  used = (uint8_t)(sizeof(packet) - i);
  if (used) memmove(packet, packet + i, used);
}

void JY901_Snapshot(JY901Snapshot *out) { *out = data; }

const char *JY901_Status(const JY901Snapshot *s, uint32_t now)
{
  if (!s->bytes) return "NO_DATA";
  if (!s->frames) return "NO_FRAME";
  if (!s->angle_frames) return "NO_ANGLE";
  return (uint32_t)(now - s->angle_tick) > JY901_STALE_MS ? "STALE" : "OK";
}

int32_t JY901_AngleCdeg(int32_t raw) { return raw * 18000 / 32768; }

int32_t JY901_RelativeYawRaw(int16_t yaw, int16_t reference)
{
  int32_t delta = (int32_t)yaw - reference;
  if (delta > 32767) delta -= 65536;
  if (delta < -32768) delta += 65536;
  return delta;
}
