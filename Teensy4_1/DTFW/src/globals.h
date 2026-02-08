#ifndef GLOBALS_H
#define GLOBALS_H
#include <string.h>
#include <IntervalTimer.h>
#include "SerialBuffer.h"
#include "utils.h"
#include "fro_t4.h"
volatile bool inISR = false;
volatile bool newData = false;
FRO_T4 fro;

DMAMEM uint8_t rbmem[8192] = {0}; // ring buffer memory (128KB = 2048 packets)
// 16MB in external PSRAM: 699,050 rows, each row = 6 floats (24 bytes)
EXTMEM float trajectorybuffer[MAX_ROWS][NUM_AXIS];

volatile uint32_t droppedBytes = 0; // track serial overflow

TrajectoryTable trajectory(trajectorybuffer, MAX_ROWS);

IntervalTimer myTimer;
elapsedMicros timerMicros;

SerialBuffer rb(rbmem, sizeof(rbmem));
rawPacket pkt;



float Pose[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
uint32_t lastSeq = 0;
uint32_t receivedPackets = 0;
uint32_t processedPackets = 0;
uint32_t lostPackets  = 0;

bool hasData = false;
// globals
static float frRemainder = 0.0f;   // [0,1)
static uint32_t idx = 0;           // integer trajectory index
static float outPose[NUM_AXIS];    // 6-float lerped pose

static inline void lerp6(float* out, const float* a, const float* b, float u)
{
  out[0] = a[0] + (b[0] - a[0]) * u;
  out[1] = a[1] + (b[1] - a[1]) * u;
  out[2] = a[2] + (b[2] - a[2]) * u;
  out[3] = a[3] + (b[3] - a[3]) * u;
  out[4] = a[4] + (b[4] - a[4]) * u;
  out[5] = a[5] + (b[5] - a[5]) * u;
}

void timerISR()
{
inISR = true;
  fro.tick();
  float feedRate = fro.getFeedrate();   // guaranteed 0..1
  uint32_t n = trajectory.length();
    if(!n || !hasData)
    {
      inISR = false;
      return;
    }
  frRemainder += feedRate;
  if (frRemainder >= 1.0f) 
  {
    frRemainder -= 1.0f;
    if (++idx >= n) idx = 0;
  }
  float u = frRemainder;
  uint32_t idx1 = idx + 1;
  if (idx1 >= n) idx1 = 0;
  const float* a = trajectory[idx];
  const float* b = trajectory[idx1];
  lerp6(outPose, a, b, u);
  // newData = true;
  inISR = false;
}
#endif // GLOBALS_H