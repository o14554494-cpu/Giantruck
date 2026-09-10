#ifndef JY901_H
#define JY901_H
#include <stdint.h>

/* Receiver setting only: this does not reconfigure the sensor. */
#define JY901_UART_BAUD 9600U
#define JY901_STALE_MS 500U
#define JY901_REPORT_MS 500U
#define JY901_INTERBYTE_TIMEOUT_MS 25U

typedef struct {
  uint32_t bytes, frames, angle_frames, gyro_frames, checksum_errors;
  uint32_t type_errors, stream_timeouts;
  uint32_t angle_tick, gyro_tick;
  int16_t angle[3]; /* roll, pitch, yaw: raw * 180 / 32768 degrees */
  int16_t gyro[3];  /* raw * 2000 / 32768 degrees/second */
} JY901Snapshot;

void JY901_Init(void);
/* Constant-sized parser, one producer (USART1 ISR); no printing or floats. */
void JY901_RxByte(uint8_t byte, uint32_t now);
void JY901_ResetStream(void);
/* Caller masks the producer interrupt while copying a coherent snapshot. */
void JY901_Snapshot(JY901Snapshot *out);
const char *JY901_Status(const JY901Snapshot *s, uint32_t now);
int32_t JY901_AngleCdeg(int32_t raw);
int32_t JY901_RelativeYawRaw(int16_t yaw, int16_t reference);
#endif
