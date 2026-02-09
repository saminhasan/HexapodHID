#include "globals.h"

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.setTimeout(0);
    while (!Serial)
        yield();
    usb.begin();

    dev1.begin(115200);
    dev2.begin(115200);
    dev3.begin(115200);

    logInfof("Waiting for 3 USB-Serial slaves...");
    bool p1 = false, p2 = false, p3 = false;

    while (1)
    {
        usb.Task();
        if (dev1 && !p1) { p1 = true; printDev(dev1, "dev1"); }
        if (dev2 && !p2) { p2 = true; printDev(dev2, "dev2"); }
        if (dev3 && !p3) { p3 = true; printDev(dev3, "dev3"); }
        if (p1 && p2 && p3) break;
        yield();
    }

    logInfof("All 3 connected.");
    logInfof("System Initialized.");
    myTimer.begin(timerISR, 1000); // 1kHz
}

// ============================================================================
// Ring Buffers for Slave Replies
// ============================================================================
static uint8_t slaveRbMem1[512] = {0};
static uint8_t slaveRbMem2[512] = {0};
static uint8_t slaveRbMem3[512] = {0};
static SerialBuffer slaveRb1(slaveRbMem1, sizeof(slaveRbMem1));
static SerialBuffer slaveRb2(slaveRbMem2, sizeof(slaveRbMem2));
static SerialBuffer slaveRb3(slaveRbMem3, sizeof(slaveRbMem3));
static rawPacket slavePkt;

// ============================================================================
// Broadcast: send raw packet to all 3 slaves
// ============================================================================
static void broadcast_serial(const uint8_t* data, uint32_t len)
{
    if (!data || len == 0) return;
    if (dev1) dev1.write(data, len);
    if (dev2) dev2.write(data, len);
    if (dev3) dev3.write(data, len);
}

// ============================================================================
// Emergency: disable all slaves immediately
// ============================================================================
static void disableAllSlaves()
{
    HexLink link(MASTER_ID, 0x00);
    rawPacket p = link.disable();
    broadcast_serial(p.buffer, PACKET_SIZE);
    fro.setFeedrate(0.0f);
    sysState = SystemState::ERROR;
    logInfof("ERR:ALL_SLAVES_DISABLED");
}

// ============================================================================
// Slave Reply Handler
// ============================================================================
// Maps FROM byte (slave ID 1-3) to array index 0-2.
// Returns 0xFF if not a valid slave ID.
static uint8_t slaveIndex(uint8_t fromByte)
{
    if (fromByte >= 1 && fromByte <= NUM_SLAVES) return fromByte - 1;
    return 0xFF;
}

static void handleSlavePacket(const char* tag, const uint8_t* buffer)
{
    const uint8_t msgid = buffer[4];
    const uint8_t from  = buffer[1];

    // Forward STATUS and INFO packets directly to PC (binary passthrough)
    if (msgid == MSGID_STATUS || msgid == MSG_ID_INFO)
    {
        Serial.write(buffer, PACKET_SIZE);

        // --- Error detection from slave INFO text ---
        if (msgid == MSG_ID_INFO)
        {
            // INFO payload: [5]=flags, [6..60]=text
            const char* text = (const char*)(buffer + 6);
            // Only fatal errors trigger emergency disable.
            // ERR:FATAL comes from slave globalError (motor/CAN fault).
            // Non-fatal prefixes (BUSY:, WARN:, ERR:CALIB etc.) are ignored here.
            if (strncmp(text, "ERR:FATAL", 9) == 0)
            {
                logInfof("SLAVE_FATAL from %s: %s", tag, text);
                disableAllSlaves();
                return;
            }

            // --- Calibration response tracking ---
            if (sysState == SystemState::CALIBRATING)
            {
                uint8_t si = slaveIndex(from);
                if (si < NUM_SLAVES)
                {
                    if (strncmp(text, "OK:CALIB_PHASE_", 15) == 0)
                    {
                        calTracker.slaveReady[si] = true;
                    }
                    else if (strncmp(text, "OK:CALIB_STARTED", 16) == 0)
                    {
                        calTracker.slaveReady[si] = true;
                    }
                    else if (strncmp(text, "OK:ALREADY_CALIBRATED", 21) == 0)
                    {
                        calTracker.slaveDone[si] = true;
                    }
                }
            }
        }
        return;
    }

    logInfof("SLV[%s] recv MSGID=0x%02X", tag, msgid);
}

// ============================================================================
// Command Handlers
// ============================================================================

void ping()
{
    logInfof("PONG");
}

void pong()
{
    logInfof("PONG");
}

void enable()
{
    if (sysState == SystemState::ERROR)
    {
        logInfof("ERR:CLEAR_ERROR_FIRST");
        return;
    }
    sysState = SystemState::ARMED;
    logInfof("OK:ENABLED state=ARMED");
}

void disable()
{
    fro.setFeedrate(0.0f);
    sysState = SystemState::IDLE;
    logInfof("OK:DISABLED state=IDLE");
}

void calibrate()
{
    if (sysState != SystemState::ARMED && sysState != SystemState::CALIBRATING)
    {
        logInfof("ERR:MUST_BE_ARMED_TO_CALIBRATE state=%u", (uint8_t)sysState);
        return;
    }

    // Build the broadcast packet once
    HexLink link(MASTER_ID, 0x00);
    rawPacket p = link.calibrate();

    if (sysState == SystemState::ARMED)
    {
        // First press — start calibration, send first broadcast
        calTracker.reset();
        sysState = SystemState::CALIBRATING;
        broadcast_serial(p.buffer, PACKET_SIZE);
        logInfof("OK:CALIB_STARTING phase=0");
        return;
    }

    // Already CALIBRATING — check if all slaves finished current phase
    if (!calTracker.allReady())
    {
        logInfof("BUSY:CALIB_WAITING phase=%u ready=[%d,%d,%d]",
                 calTracker.currentPhase,
                 calTracker.slaveReady[0] || calTracker.slaveDone[0],
                 calTracker.slaveReady[1] || calTracker.slaveDone[1],
                 calTracker.slaveReady[2] || calTracker.slaveDone[2]);
        return;
    }

    // All slaves ready — advance phase and broadcast next step
    calTracker.currentPhase++;
    calTracker.clearReady();
    broadcast_serial(p.buffer, PACKET_SIZE);
    logInfof("OK:CALIB_ADVANCING phase=%u", calTracker.currentPhase);
}

void estop()
{
    disableAllSlaves();
    logInfof("OK:ESTOP");
}

void reset()
{
    // Clear error state, go back to IDLE
    fro.setFeedrate(0.0f);
    hasData = false;
    idx = 0;
    frRemainder = 0.0f;
    calTracker.reset();
    sysState = SystemState::IDLE;
    logInfof("OK:RESET state=IDLE");
}

void jog(const uint8_t* buffer)
{
    float pose[NUM_AXIS];
    memcpy(pose, buffer + 5, sizeof(float) * NUM_AXIS);
    logInfof("JOG: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
             pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    // JOG from PC gets broadcast directly to slaves
}

void stage()
{
    if (!hasData)
    {
        logInfof("ERR:NO_TRAJECTORY_DATA");
        return;
    }
    if (sysState == SystemState::IDLE || sysState == SystemState::ERROR)
    {
        logInfof("ERR:MUST_BE_ENABLED state=%u", (uint8_t)sysState);
        return;
    }

    // Build a JOG to trajectory[0]
    const float* firstRow = trajectory[0];
    HexLink link(MASTER_ID, 0x00);
    rawPacket p = link.jog(firstRow);
    broadcast_serial(p.buffer, PACKET_SIZE);

    idx = 0;
    frRemainder = 0.0f;
    sysState = SystemState::STAGED;
    logInfof("OK:STAGED pos=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
             firstRow[0], firstRow[1], firstRow[2],
             firstRow[3], firstRow[4], firstRow[5]);
}

void park()
{
    float parkPose[NUM_AXIS];
    for (uint8_t i = 0; i < NUM_AXIS; i++)
        parkPose[i] = PARK_ANGLE;

    HexLink link(MASTER_ID, 0x00);
    rawPacket p = link.jog(parkPose);
    broadcast_serial(p.buffer, PACKET_SIZE);

    logInfof("OK:PARKING angle=%.4f", PARK_ANGLE);
}

void upload(const uint8_t* buffer)
{
    if (!trajectory.pushRowBytes(buffer + 5))
    {
        logInfof("ERR:TRAJECTORY_FULL at %u rows", trajectory.length());
    }
}

void validate(const uint8_t* buffer)
{
    logInfof("VALIDATE:");

    uint32_t pktLen, pktCrc;
    memcpy(&pktLen, buffer + 5, 4);
    memcpy(&pktCrc, buffer + 9, 4);
    uint32_t ramCrc = trajectory.checksum();
    uint32_t ramLen = trajectory.length();

    logInfof("Received packets: %u, Dropped bytes: %u", receivedPackets, droppedBytes);

    if (ramLen > 0)
    {
        const uint8_t* firstRow = (const uint8_t*)trajectory[0];
        char hexline[3 * 24 + 1] = {0};
        size_t h = 0;
        for (int i = 0; i < 24 && h + 2 < sizeof(hexline); i++)
            h += snprintf(hexline + h, sizeof(hexline) - h, "%02X", firstRow[i]);
        logInfof("First row bytes: %s", hexline);
    }

    if (pktLen != ramLen)
        logInfof("ERR:LENGTH_MISMATCH expected=%u actual=%u", pktLen, ramLen);
    else if (pktCrc != ramCrc)
        logInfof("ERR:CRC32_MISMATCH expected=0x%08X actual=0x%08X", pktCrc, ramCrc);
    else
    {
        hasData = true;
        logInfof("OK:VALIDATED length=%u", ramLen);
    }
}

void play()
{
    if (!hasData)
    {
        logInfof("ERR:NO_TRAJECTORY_DATA");
        return;
    }
    // Allow play from STAGED or PAUSED
    if (sysState != SystemState::STAGED && sysState != SystemState::PAUSED)
    {
        logInfof("ERR:MUST_BE_STAGED_OR_PAUSED state=%u", (uint8_t)sysState);
        return;
    }
    fro.setFeedrate(1.0f);
    sysState = SystemState::PLAYING;
    logInfof("OK:PLAYING fro=%.3f trajLen=%u idx=%u hasData=%d",
             fro.getFeedrate(), trajectory.length(), idx, hasData);
}

void pause()
{
    fro.setFeedrate(0.0f);
    if (sysState == SystemState::PLAYING)
        sysState = SystemState::PAUSED;
    logInfof("OK:PAUSED");
}

void stop()
{
    fro.setFeedrate(0.0f);
    idx = 0;
    frRemainder = 0.0f;
    hasData = false;
    trajectory.reset();
    receivedPackets = 0;
    droppedBytes = 0;

    // Go back to ARMED (motors stay enabled)
    if (sysState != SystemState::IDLE && sysState != SystemState::ERROR)
        sysState = SystemState::ARMED;
    logInfof("OK:STOPPED state=%u", (uint8_t)sysState);
}

void status()
{
    logInfof("STATE=%u hasData=%d trajLen=%u idx=%u fro=%.3f",
             (uint8_t)sysState, hasData, trajectory.length(), idx, fro.getFeedrate());
}

// ============================================================================
// Packet Dispatcher
// ============================================================================
static void handlePacket(const uint8_t* buffer)
{
    const uint8_t msgid = buffer[4];
    if (msgid != MSGID_UPLOAD) receivedPackets++;

    switch (msgid)
    {
    case MSGID_PING:
        ping();
        break;

    case MSGID_PONG:
        pong();
        break;

    case MSGID_ENABLE:
        enable();
        broadcast_serial(buffer, PACKET_SIZE);
        break;

    case MSGID_DISABLE:
        disable();
        broadcast_serial(buffer, PACKET_SIZE);
        break;

    case MSGID_CALIBRATE:
        calibrate();
        break;

    case MSGID_UPLOAD:
        upload(buffer);
        break;

    case MSGID_VALIDATE:
        validate(buffer);
        break;

    case MSGID_STAGE:
        stage();
        break;

    case MSGID_PARK:
        park();
        break;

    case MSGID_PLAY:
        play();
        broadcast_serial(buffer, PACKET_SIZE);
        break;

    case MSGID_PAUSE:
        pause();
        broadcast_serial(buffer, PACKET_SIZE);
        break;

    case MSGID_STOP:
        stop();
        broadcast_serial(buffer, PACKET_SIZE);
        break;

    case MSGID_ESTOP:
        estop();
        broadcast_serial(buffer, PACKET_SIZE);
        break;

    case MSGID_RESET:
        reset();
        broadcast_serial(buffer, PACKET_SIZE);
        break;

    case MSGID_JOG:
        jog(buffer);
        broadcast_serial(buffer, PACKET_SIZE);
        break;

    case MSGID_STATUS:
        status();
        broadcast_serial(buffer, PACKET_SIZE);
        break;

    default:
        logInfof("ERR:UNKNOWN_MSGID 0x%02X", msgid);
        break;
    }
}

// ============================================================================
// Serial Parsing (identical framing logic)
// ============================================================================
static bool parseSerial(SerialBuffer& sb, rawPacket& p)
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

void serialEvent(SerialBuffer& sb)
{
    size_t dropped = 0;
    sb.readStream(Serial, &dropped);
    if (dropped) droppedBytes += dropped;
}

// ============================================================================
// Calibration Completion Check — runs in loop()
// ============================================================================
// No auto-pump. User presses CALIBRATE for each phase.
// This just detects when all slaves report done.
// ============================================================================
static void checkCalibrationDone()
{
    if (sysState != SystemState::CALIBRATING)
        return;

    if (calTracker.allDone())
    {
        sysState = SystemState::CALIBRATED;
        logInfof("OK:CALIBRATION_COMPLETE");
    }
}



// ============================================================================
// Main Loop
// ============================================================================
void loop()
{
    usb.Task();

    // ---- PC serial ----
    serialEvent(rb);
    if (parseSerial(rb, pkt))
    {
        handlePacket(pkt.buffer);
    }

    // ---- Slave serial replies ----
    if (dev1) { slaveRb1.readStream(dev1); if (parseSerial(slaveRb1, slavePkt)) handleSlavePacket("A", slavePkt.buffer); }
    if (dev2) { slaveRb2.readStream(dev2); if (parseSerial(slaveRb2, slavePkt)) handleSlavePacket("B", slavePkt.buffer); }
    if (dev3) { slaveRb3.readStream(dev3); if (parseSerial(slaveRb3, slavePkt)) handleSlavePacket("C", slavePkt.buffer); }

    // ---- Calibration completion check ----
    checkCalibrationDone();
}
