#include <Arduino.h>
#include <FastCRC.h>
#include <FlexCAN_T4.h>
#include "constants.h"
#include "hexlink.h"
#include "SerialBuffer.h"
#include "delf.h"

#ifndef SLAVE_ID
#define SLAVE_ID 0
#endif

// ---- ring buffer for incoming serial data (same as DTFW) ----
static uint8_t rbmem[1024] = {0};
static SerialBuffer rb(rbmem, sizeof(rbmem));
static rawPacket pkt;

volatile uint32_t droppedBytes = 0;

// ============================================================================
// CAN Bus Instances
// ============================================================================
FlexCAN_T4<CAN1, RX_SIZE_8, TX_SIZE_8> canBus1;
FlexCAN_T4<CAN2, RX_SIZE_8, TX_SIZE_8> canBus2;

// CAN bus wrappers for polymorphic access
CANBusWrapper<FlexCAN_T4<CAN1, RX_SIZE_8, TX_SIZE_8>> can1Wrapper(canBus1);
CANBusWrapper<FlexCAN_T4<CAN2, RX_SIZE_8, TX_SIZE_8>> can2Wrapper(canBus2);

// ============================================================================
// Motor Pair Controller
// ============================================================================
AxisPair motors(SLAVE_ID);

// Axis indices for position array access
constexpr uint8_t AXIS_L_IDX = SLAVE_ID * 2 - 2;  // 0, 2, or 4
constexpr uint8_t AXIS_R_IDX = SLAVE_ID * 2 - 1;  // 1, 3, or 5

// ============================================================================
// Timer ISR
// ============================================================================
IntervalTimer controlTimer;

void controlISR()
{
    motors.update();
}

// ============================================================================
// Command Handlers — all reply via logInfof (framed hexlink packets)
// ============================================================================
void cmdPing()
{
    logInfof("PONG");
}

void cmdPong()
{
    logInfof("PING");
}

void cmdEnable()
{
    motors.enable();
    logInfof("OK:ENABLED");
}

void cmdDisable()
{
    motors.disable();
    logInfof("OK:DISABLED");
}

void cmdCalibrate()
{
    if (motors.isCalibrated())
    {
        logInfof("OK:ALREADY_CALIBRATED");
        return;
    }

    if (!motors.isCalibrating())
    {
        motors.startCalibration();
        logInfof("OK:CALIB_STARTED");
        return;
    }

    if (motors.advanceCalibration())
    {
        uint8_t phase = static_cast<uint8_t>(motors.getCalibrationPhase());
        logInfof("OK:CALIB_PHASE_%u", phase);
    }
    else
    {
        logInfof("BUSY:CALIB_IN_PROGRESS");
    }
}

void cmdEstop()
{
    motors.disable();
    logInfof("OK:ESTOP");
}

void cmdReset()
{
    motors.disable();
    motors.init();
    logInfof("OK:RESET");
}

void cmdStatus()
{
    motors.printStatus();
    motors.printCANStats();
}

void cmdJog(const uint8_t *buffer)
{
    // payload starts at byte 5, same as DTFW
    float pose[NUM_AXIS];
    memcpy(pose, buffer + 5, sizeof(pose));

    motors.setPositions(pose[AXIS_L_IDX], pose[AXIS_R_IDX]);
}

void cmdPlay()
{
    motors.setHighGain();
    motors.setPlaying(true);
    logInfof("OK:PLAYING");
}

void cmdPause()
{
    logInfof("OK:PAUSED");
}

void cmdStop()
{
    motors.setLowGain();
    motors.setPlaying(false);
    logInfof("OK:STOPPED");
}

// ============================================================================
// Packet Dispatcher
// ============================================================================
static void handlePacket(const uint8_t *buffer)
{
    const uint8_t msgId = buffer[4];

    switch (msgId)
    {
        case MSGID_PING:      cmdPing();        break;
        case MSGID_PONG:      cmdPong();        break;
        case MSGID_ENABLE:    cmdEnable();      break;
        case MSGID_DISABLE:   cmdDisable();     break;
        case MSGID_CALIBRATE: cmdCalibrate();   break;
        case MSGID_PLAY:      cmdPlay();        break;
        case MSGID_PAUSE:     cmdPause();       break;
        case MSGID_STOP:      cmdStop();        break;
        case MSGID_ESTOP:     cmdEstop();       break;
        case MSGID_RESET:     cmdReset();       break;
        case MSGID_JOG:       cmdJog(buffer);   break;
        case MSGID_STATUS:    cmdStatus();      break;
        default:
            logInfof("WARN:UNKNOWN_MSGID 0x%02X", msgId);
            break;
    }
}

// ---- parseSerial: identical to DTFW ----
static bool parseSerial(SerialBuffer &sb, rawPacket &p)
{
    if (sb.size() && sb[0] != START_BYTE)
    {
        if (!sb.readBytesUntil(START_BYTE))
            return false;
    }

    if (sb.size() < PACKET_SIZE)
        return false;

    if (!(sb[PACKET_SIZE - 1] == END_BYTE) && (sb.crc16(PACKET_SIZE - 1) == CRC_RESIDUE))
    {
        sb.discard(1);
        sb.readBytesUntil(START_BYTE);
        return false;
    }
    sb.readBytes(p.buffer, PACKET_SIZE);
    return true;
}

// ---- serialEvent: read serial into ring buffer ----
void serialEvent()
{
    size_t dropped = 0;
    rb.readStream(Serial, &dropped);
    if (dropped)
        droppedBytes += dropped;
}

// ============================================================================
// Setup
// ============================================================================
void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(0);
    while (!Serial)
        yield();

    logInfof("Slave %d starting (Axes %d, %d)", SLAVE_ID, motors.axisL.axisId, motors.axisR.axisId);

    // Attach CAN buses to motor pair
    motors.attachCAN(&can1Wrapper, &can2Wrapper);

    // Initialize CAN buses
    motors.initCAN(500000);

    // Initialize motor states
    motors.init();

    // Start control loop at 1kHz
    controlTimer.priority(128);
    controlTimer.begin(controlISR, 1000);

    logInfof("OK:READY");
}

// ============================================================================
// Loop
// ============================================================================
void loop()
{
    // Read serial into ring buffer & parse
    serialEvent();
    if (parseSerial(rb, pkt))
    {
        handlePacket(pkt.buffer);
    }

    // Non-time-critical: telemetry + CAN events
    motors.tick();

    // Error state handler
    if (motors.globalError)
    {
        motors.disable();
        logInfof("ERR:FATAL - MOTORS DISABLED");
        while (motors.globalError)
        {
            delay(1000);
            logInfof("ERR:FATAL - RESET REQUIRED");
            yield();
        }
    }
}

// ============================================================================
// CAN RX Interrupt Hook (Weak function from FlexCAN_T4)
// ============================================================================
void ext_output1(const CAN_message_t &msg)
{
    motors.routeCANResponse(msg);
}
