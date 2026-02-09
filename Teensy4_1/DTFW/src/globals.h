#ifndef GLOBALS_H
#define GLOBALS_H
#include <string.h>
#include <IntervalTimer.h>
#include "SerialBuffer.h"
#include "utils.h"
#include "fro_t4.h"
#include <USBHost_t36.h>

// ============================================================================
// System State Tracking
// ============================================================================
// Not a state machine — just tracks what mode the system is in so that
// command handlers and loop() can make sensible decisions.
enum class SystemState : uint8_t
{
    IDLE,           // Motors disabled, nothing happening
    ARMED,          // Motors enabled, low-gain, accepting JOG / CALIBRATE
    CALIBRATING,    // Calibration routine in progress (staged across slaves)
    CALIBRATED,     // Calibration complete, ready for trajectory ops
    STAGING,        // Jogging to trajectory[0]
    STAGED,         // At trajectory[0], ready for PLAY
    PLAYING,        // Trajectory playback active at 1kHz
    PAUSED,         // Feedrate zeroed, trajectory index held
    ERROR           // Fault detected — motors disabled
};

volatile SystemState sysState = SystemState::IDLE;

// ============================================================================
// Calibration Orchestration
// ============================================================================
// The slaves calibrate in phases (see STFW delf.h CalibrationPhase).
// Both axes on each slave must finish the current phase before the pair
// can advance.  The master must additionally keep ALL THREE slaves in
// sync — it sends CALIBRATE repeatedly; each time a slave advances its
// pair one phase and replies OK:CALIB_PHASE_N or OK:ALREADY_CALIBRATED.
// We track per-slave completion here so the master knows when to send
// the next round of CALIBRATE broadcasts.
// ============================================================================
static constexpr uint8_t NUM_SLAVES = 3;

struct CalibrationTracker
{
    bool slaveReady[NUM_SLAVES] = {false, false, false};   // slave replied "ready for next phase"
    bool slaveDone[NUM_SLAVES]  = {false, false, false};   // slave replied "ALREADY_CALIBRATED"
    uint8_t currentPhase = 0;                              // master-side phase counter
    bool waitingForUser = false;                             // true = all slaves ready, waiting for next CALIBRATE press

    void reset()
    {
        for (uint8_t i = 0; i < NUM_SLAVES; i++)
        {
            slaveReady[i] = false;
            slaveDone[i] = false;
        }
        currentPhase = 0;
        waitingForUser = false;
    }

    bool allReady() const
    {
        for (uint8_t i = 0; i < NUM_SLAVES; i++)
            if (!slaveReady[i] && !slaveDone[i]) return false;
        return true;
    }

    bool allDone() const
    {
        for (uint8_t i = 0; i < NUM_SLAVES; i++)
            if (!slaveDone[i]) return false;
        return true;
    }

    void clearReady()
    {
        for (uint8_t i = 0; i < NUM_SLAVES; i++)
            slaveReady[i] = false;
    }
};

CalibrationTracker calTracker;

// ============================================================================
// USB Host — three downstream slave Teensy devices
// ============================================================================
USBHost usb;
USBHub hub1(usb);

USBSerial_BigBuffer dev1(usb);
USBSerial_BigBuffer dev2(usb);
USBSerial_BigBuffer dev3(usb);

// Convenience array for iteration
USBSerial_BigBuffer* slaves[NUM_SLAVES] = {&dev1, &dev2, &dev3};

// ============================================================================
// Feedrate Override
// ============================================================================
FRO_T4 fro;

// ============================================================================
// Trajectory Storage (16MB PSRAM)
// ============================================================================
EXTMEM float trajectorybuffer[MAX_ROWS][NUM_AXIS];
TrajectoryTable trajectory(trajectorybuffer, MAX_ROWS);
bool hasData = false;

// ============================================================================
// Trajectory Playback State (owned by ISR, read by loop for status)
// ============================================================================
static volatile float    frRemainder = 0.0f;    // fractional index [0,1)
static volatile uint32_t idx = 0;               // integer trajectory index
static float  outPose[NUM_AXIS] = {};            // scratch buffer (ISR only)

// ============================================================================
// ISR Communication
// ============================================================================
// ISR does trajectory interpolation + JOG broadcast directly at 1kHz.
// No handoff needed — 600MHz M7 handles lerp6 + packet build + 3× write easily.

// ============================================================================
// Serial / Packet Buffers
// ============================================================================
DMAMEM uint8_t rbmem[8192] = {0};
SerialBuffer rb(rbmem, sizeof(rbmem));
rawPacket pkt;

// ============================================================================
// Stats
// ============================================================================
volatile uint32_t droppedBytes = 0;
uint32_t receivedPackets = 0;

// ============================================================================
// HexLink instance (master → slaves)
// ============================================================================
HexLink masterLink(MASTER_ID, PC_ID);

// ============================================================================
// Timers
// ============================================================================
IntervalTimer myTimer;

// ============================================================================
// Inline Helpers
// ============================================================================
static inline void lerp6(float* out, const float* a, const float* b, float u)
{
    out[0] = a[0] + (b[0] - a[0]) * u;
    out[1] = a[1] + (b[1] - a[1]) * u;
    out[2] = a[2] + (b[2] - a[2]) * u;
    out[3] = a[3] + (b[3] - a[3]) * u;
    out[4] = a[4] + (b[4] - a[4]) * u;
    out[5] = a[5] + (b[5] - a[5]) * u;
}

// Park position: all axes at -60° = -π/3
static constexpr float PARK_ANGLE = -1.0471975512f;  // -60° in radians

// ============================================================================
// ISR — runs at 1kHz
// Ticks FRO, advances trajectory, interpolates pose, builds & sends JOG.
// Everything in one place — no handoff, no sync issues.
// ============================================================================
void timerISR()
{
    fro.tick();

    if (sysState != SystemState::PLAYING || !hasData)
        return;

    uint32_t n = trajectory.length();
    if (n == 0) return;

    float feedRate = fro.getFeedrate();
    if (feedRate <= 0.0f)
        return;

    frRemainder += feedRate;
    if (frRemainder >= 1.0f)
    {
        frRemainder -= 1.0f;
        if (++idx >= n) idx = 0;
    }

    uint32_t idx1 = idx + 1;
    if (idx1 >= n) idx1 = 0;

    lerp6(outPose, trajectory[idx], trajectory[idx1], frRemainder);

    // Build JOG packet directly and broadcast
    HexLink link(MASTER_ID, 0x00);
    rawPacket p = link.jog(outPose);
    if (dev1) dev1.write(p.buffer, PACKET_SIZE);
    if (dev2) dev2.write(p.buffer, PACKET_SIZE);
    if (dev3) dev3.write(p.buffer, PACKET_SIZE);
}

// ============================================================================
// USB Device Helpers
// ============================================================================
static const char* sstr(const uint8_t* p)
{
    return (p && *p) ? (const char*)p : "";
}

static void printDev(USBSerial_BigBuffer& d, const char* name)
{
    logInfof("%s: VID=%04X PID=%04X SER='%s' PROD='%s'",
             name, d.idVendor(), d.idProduct(),
             sstr(d.serialNumber()), sstr(d.product()));
}

#endif // GLOBALS_H