#include "jy901.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t p[11];
static void packet(uint8_t type, int16_t x, int16_t y, int16_t z)
{
  unsigned i;
  const int16_t xyz[3] = {x,y,z};
  memset(p,0,sizeof(p)); p[0]=0x55; p[1]=type;
  for (i=0;i<3;i++) { p[2+2*i]=(uint16_t)xyz[i]&255; p[3+2*i]=(uint16_t)xyz[i]>>8; }
  for (i=0;i<10;i++) p[10]=(uint8_t)(p[10]+p[i]);
}
static void feed(uint32_t now)
{
  unsigned i;
  for (i=0;i<11;i++) JY901_RxByte(p[i],now);
}
int main(void)
{
  JY901Snapshot s;
  unsigned i;
  JY901_Init(); JY901_Snapshot(&s);
  assert(!strcmp(JY901_Status(&s,0),"NO_DATA"));
  JY901_RxByte(0xAA,1); JY901_Snapshot(&s);
  assert(!strcmp(JY901_Status(&s,1),"NO_FRAME"));
  packet(0x51,0,0,2048); feed(2); JY901_Snapshot(&s);
  assert(s.frames==1 && !strcmp(JY901_Status(&s,2),"NO_ANGLE"));
  packet(0x52,-32768,32767,16384); feed(3);
  packet(0x53,8192,-16384,-32768); feed(4); JY901_Snapshot(&s);
  assert(s.gyro_frames==1 && s.gyro[0]==-32768 && s.gyro[1]==32767);
  assert(s.angle_frames==1 && s.angle[2]==-32768);
  assert(JY901_AngleCdeg(s.angle[0])==4500 && JY901_AngleCdeg(s.angle[1])==-9000);
  assert(!strcmp(JY901_Status(&s,4),"OK"));
  assert(!strcmp(JY901_Status(&s,1505),"STALE"));
  packet(0x51,0,0,0); feed(1600); JY901_Snapshot(&s);
  assert(!strcmp(JY901_Status(&s,1600),"STALE")); /* Non-angle packets cannot freshen yaw. */
  packet(0x53,0,0,0); p[10]^=1; feed(1700); JY901_Snapshot(&s);
  assert(s.checksum_errors==1 && s.angle_frames==1 && s.angle_tick==4);
  packet(0x53,1,2,3); feed(1800); JY901_Snapshot(&s);
  assert(s.angle_frames==2 && s.angle[2]==3);
  /* Truncated packet immediately followed by a whole packet. */
  packet(0x53,10,20,30);
  for (i=0;i<5;i++) JY901_RxByte(p[i],1801);
  feed(1802); JY901_Snapshot(&s);
  assert(s.angle_frames==3 && s.angle[2]==30);
  /* Header byte in valid payload must not restart the parser. */
  packet(0x53,0x5555,0x0055,0x5500); feed(1803); JY901_Snapshot(&s);
  assert(s.angle_frames==4 && s.angle[0]==0x5555);
  packet(0x53,0,0,0); feed(UINT32_MAX-10); JY901_Snapshot(&s);
  assert(!strcmp(JY901_Status(&s,20),"OK"));
  assert(!strcmp(JY901_Status(&s,1600),"STALE"));
  assert(JY901_RelativeYawRaw(-32700,32700)==136);
  assert(JY901_RelativeYawRaw(32700,-32700)==-136);
  packet(0x53,0,0,16384);
  for(i=0;i<5;i++) JY901_RxByte(p[i],2000);
  JY901_ResetStream(); feed(2001); JY901_Snapshot(&s);
  assert(s.angle[2]==16384);
  puts("PASS: JY901 signed angles/gyro, checksum, resync, embedded header, independent freshness and tick/yaw wrap");
  return 0;
}
