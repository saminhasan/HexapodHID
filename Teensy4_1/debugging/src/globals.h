#ifndef GLOBALS_H
#define GLOBALS_H
#include <stdint.h>
#include <string.h>
#include <IntervalTimer.h>
#include "fro_t4.h"
#include <USBHost_t36.h>



static constexpr uint32_t NUM_AXIS = 6u;
static constexpr uint32_t EXT_BYTES = 16u * 1024u * 1024u;   // 16,777,216
static constexpr uint32_t MAX_ROWS  = EXT_BYTES / NUM_AXIS / sizeof(float); // 699,050

#include "utils.h"


FRO_T4 fro;

EXTMEM float trajectorybuffer[MAX_ROWS][NUM_AXIS];
TrajectoryTable trajectory(trajectorybuffer, MAX_ROWS);


static volatile float    frRemainder = 0.0f;    // fractional index [0,1)
static volatile uint32_t idx = 0;               // integer trajectory index
static volatile float  outPose[NUM_AXIS] = {};            // scratch buffer (ISR only)

uint32_t counter =0;
IntervalTimer myTimer;


static inline void lerp6(volatile float* out, const float* a, const float* b, float u)
{
    out[0] = a[0] + (b[0] - a[0]) * u;
    out[1] = a[1] + (b[1] - a[1]) * u;
    out[2] = a[2] + (b[2] - a[2]) * u;
    out[3] = a[3] + (b[3] - a[3]) * u;
    out[4] = a[4] + (b[4] - a[4]) * u;
    out[5] = a[5] + (b[5] - a[5]) * u;
}

static constexpr float PARK_ANGLE = -1.0471975512f;  


void timerISR()
{

uint32_t n = trajectory.length();
if (n == 0 ) return;

fro.tick();
float feedRate = fro.getFeedrate();

float frBefore = frRemainder;
uint32_t idxBefore = idx;

frRemainder += feedRate;
if (frRemainder >= 1.0f)
{
    frRemainder -= 1.0f;
    if (++idx >= n)
        idx = 0;
}

uint32_t idx1 = idx + 1;
if (idx1 >= n) idx1 = 0;

lerp6(outPose, trajectory[idx], trajectory[idx1], frRemainder);

Serial.printf(
    "cnt=%lu n=%lu "
    "idxBefore=%lu idx=%lu "
    "feedRate=%f frBefore=%f frRemainder=%f "
    "P0=[%f %f %f %f %f %f] "
    "OUT(fr=%f)=[%f %f %f %f %f %f] "
    "idx1=%lu P1=[%f %f %f %f %f %f]\n",
    (unsigned long)counter,
    (unsigned long)n,
    (unsigned long)idxBefore,
    (unsigned long)idx,
    feedRate,
    frBefore,
    frRemainder,
    trajectory[idx][0],  trajectory[idx][1],  trajectory[idx][2],
    trajectory[idx][3],  trajectory[idx][4],  trajectory[idx][5],
    frRemainder,
    outPose[0], outPose[1], outPose[2],
    outPose[3], outPose[4], outPose[5],
    (unsigned long)idx1,
    trajectory[idx1][0], trajectory[idx1][1], trajectory[idx1][2],
    trajectory[idx1][3], trajectory[idx1][4], trajectory[idx1][5]
);

counter++;
}




#endif // GLOBALS_H