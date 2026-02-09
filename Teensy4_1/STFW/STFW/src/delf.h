#ifndef DELF_H
#define DELF_H

#include "motctrl_prot.h"
#include <Bounce2.h>
#include <FlexCAN_T4.h>
#include "constants.h"
#include "hexlink.h"
// ============================================================================
// Forward Declarations
// ============================================================================
class AxisPair;  // Forward declaration for paired motor coordination

// ============================================================================
// CAN Bus Abstraction - Wraps FlexCAN_T4 for runtime polymorphism
// ============================================================================
class CANBusInterface
{
public:
    virtual ~CANBusInterface() = default;
    virtual int write(const CAN_message_t &msg) = 0;
    virtual void begin() = 0;
    virtual void setBaudRate(uint32_t rate) = 0;
    virtual void setMaxMB(uint8_t count) = 0;
    virtual void enableFIFO() = 0;
    virtual void enableFIFOInterrupt() = 0;
    virtual void mailboxStatus() = 0;
    virtual void events() = 0;
};

template <typename T>
class CANBusWrapper : public CANBusInterface
{
public:
    CANBusWrapper(T &canInstance) : can(canInstance) {}
    
    int write(const CAN_message_t &msg) override { return can.write(msg); }
    void begin() override { can.begin(); }
    void setBaudRate(uint32_t rate) override { can.setBaudRate(rate); }
    void setMaxMB(uint8_t count) override { can.setMaxMB(count); }
    void enableFIFO() override { can.enableFIFO(); }
    void enableFIFOInterrupt() override { can.enableFIFOInterrupt(); }
    void mailboxStatus() override { can.mailboxStatus(); }
    void events() override { can.events(); }

private:
    T &can;
};

// ============================================================================
// CAN Communication Statistics
// ============================================================================
struct CANStats
{
    volatile uint32_t txCount = 0;
    volatile uint32_t rxCount = 0;
    volatile uint32_t txErrors = 0;
    volatile uint32_t rxErrors = 0;
    volatile uint32_t timeouts = 0;
    volatile uint32_t lastTxMicros = 0;
    volatile uint32_t lastRxMicros = 0;
    volatile uint32_t roundTripMicros = 0;      // Last measured round-trip time
    volatile uint32_t maxRoundTripMicros = 0;   // Maximum round-trip time observed
    volatile bool awaitingResponse = false;
    
    void reset()
    {
        txCount = rxCount = txErrors = rxErrors = timeouts = 0;
        lastTxMicros = lastRxMicros = roundTripMicros = maxRoundTripMicros = 0;
        awaitingResponse = false;
    }
};

// ============================================================================
// Calibration State Machine
// ============================================================================
enum class CalibrationPhase : uint8_t
{
    IDLE = 0,
    SEEK_SENSOR,        // Stage 1: Move toward sensor until detected
    FIND_EDGE_1,        // Stage 2: Continue past sensor, capture falling edge 1
    REVERSE_TO_SENSOR,  // Stage 3: Reverse direction until sensor detected
    FIND_EDGE_2,        // Stage 4: Continue past sensor, capture falling edge 2
    MOVE_TO_CENTER,     // Stage 5: Move to calculated center position
    COMPLETE            // Stage 6: Calibration complete
};

struct CalibrationState
{
    CalibrationPhase currentPhase = CalibrationPhase::IDLE;
    CalibrationPhase targetPhase = CalibrationPhase::IDLE;
    float firstEdgeAngle = 0.0f;
    float secondEdgeAngle = 0.0f;
    float calculatedOffset = 0.0f;
    bool enabled = false;
    bool completed = false;
    
    void reset()
    {
        currentPhase = CalibrationPhase::IDLE;
        targetPhase = CalibrationPhase::IDLE;
        firstEdgeAngle = secondEdgeAngle = calculatedOffset = 0.0f;
        enabled = false;
        completed = false;
    }
    
    bool inProgress() const 
    { 
        return enabled && !completed && currentPhase < targetPhase; 
    }
    
    bool atTargetPhase() const
    {
        return currentPhase >= targetPhase;
    }
    
    void advanceTarget()
    {
        if (targetPhase < CalibrationPhase::COMPLETE)
            targetPhase = static_cast<CalibrationPhase>(static_cast<uint8_t>(targetPhase) + 1);
    }
};

// ============================================================================
// Motor State
// ============================================================================
struct MotorState
{
    volatile float position = 0.0f;         // theta - current position (rad)
    volatile float velocity = 0.0f;         // omega - current velocity (rad/s)
    volatile float torque = 0.0f;           // tau - current torque
    volatile int8_t temperature = 0;
    volatile bool armed = false;
    volatile bool hasError = false;
    
    void updateFromResponse(int8_t temp, float pos, float vel, float trq)
    {
        temperature = temp;
        position = pos;
        velocity = vel;
        torque = trq;
    }
};

// ============================================================================
// PD Controller Parameters
// ============================================================================
struct PDController
{
    float kP = 5.0f;                // Proportional gain
    float kD = 2.0f;                // Derivative gain
    float velocityLimit = 1.0f;     // Max velocity command
    uint8_t mode = 0;               // Control mode identifier
    
    void setHighGain()
    {
        kP = 100.0f;
        velocityLimit = 12.0f;
        mode = 1;
    }
    
    void setLowGain()
    {
        kP = 5.0f;
        velocityLimit = 0.5f;
        mode = 0;
    }
    
    float compute(float positionError, float velocity) const
    {
        float cmd = (positionError * kP) - (velocity * kD);
        return constrain(cmd, -velocityLimit, velocityLimit);
    }
};

// ============================================================================
// Telemetry Packet Structures (Packed for binary transmission)
// ============================================================================
#pragma pack(push, 1)

// Per-axis telemetry data (23 bytes)
struct AxisTelemetry
{
    uint8_t mode;           // Controller mode (0=low gain, 1=high gain)
    uint8_t flags;          // Packed status flags (see below)
    float setpoint;         // Target position (body frame, rad)
    float theta;            // Current position (body frame, rad)
    float omega;            // Current velocity (rad/s)
    float tau;              // Current torque
    int8_t temperature;     // Motor temperature (°C)
    uint16_t rttMicros;     // Round-trip time (µs, capped at 65535)
    uint8_t txErrors;       // TX error count (capped at 255)
    uint8_t timeouts;       // Timeout count (capped at 255)
    
    // Flags bit layout:
    //   Bit 0: armed
    //   Bit 1: calibrated
    //   Bit 2: calibrating (in progress)
    //   Bit 3: hasError
    //   Bit 4: awaitingResponse
    //   Bit 5-7: calibrationPhase (0-6)
};
static_assert(sizeof(AxisTelemetry) == 23, "AxisTelemetry must be 23 bytes");

// Full telemetry payload (50 bytes, fits in 55-byte payload area)
struct TelemetryPayload
{
    uint32_t timestamp;     // micros() timestamp
    uint8_t globalFlags;    // Bit 0: globalError, Bit 1-7: reserved
    AxisTelemetry left;     // Left axis data (23 bytes)
    AxisTelemetry right;    // Right axis data (23 bytes)
    // Total: 4 + 1 + 23 + 23 = 51 bytes (fits in 55 byte payload)
};
static_assert(sizeof(TelemetryPayload) == 51, "TelemetryPayload must be 51 bytes");

#pragma pack(pop)

// ============================================================================
// Axis Class - Single Motor Controller
// ============================================================================
class Axis
{
public:
    // ========== Constants ==========
    static constexpr float DELTA_THETA = 0.05f;         // Calibration step increment (rad)
    static constexpr float QUARTER_CIRCLE = M_PI / 2.0f;// 90 degrees position limit
    static constexpr uint32_t CAN_TIMEOUT_US = 5000;    // 5ms timeout for CAN response
    static constexpr uint32_t CAN_MSG_ID = 0x01;
    
    // ========== Identification ==========
    const uint8_t axisId;
    const int8_t direction;         // +1 for even IDs (right), -1 for odd IDs (left)
    
    // ========== State ==========
    MotorState motor;
    CANStats canStats;
    CalibrationState calibration;
    PDController controller;
    
    // ========== Target ==========
    volatile float targetPosition = 0.0f;
    volatile bool newResponse = false;
    
    // ========== Proximity Sensor ==========
    Bounce proxSensor;
    const int proxPin;
    
    // ========== Pre-built Commands ==========
    uint8_t cmdStart[MOTCTRL_FRAME_SIZE];
    uint8_t cmdStop[MOTCTRL_FRAME_SIZE];

private:
    CANBusInterface *canBus = nullptr;
    AxisPair *pair = nullptr;  // Reference to paired axis for coordination

public:
    // ========== Constructor ==========
    Axis(uint8_t id)
        : axisId(id)
        , direction((id % 2 == 1) ? -1 : 1)
        , proxPin((id % 2 == 1) ? 38 : 2)  // Odd IDs use pin 38, even use pin 2
    {
        MCReqStartMotor(cmdStart);
        MCReqStopMotor(cmdStop);
        
        proxSensor.attach(proxPin, INPUT_PULLUP);
        proxSensor.interval(2);
    }
    
    // ========== Configuration ==========
    void attachCAN(CANBusInterface *bus)
    {
        canBus = bus;
    }
    
    void attachPair(AxisPair *axisPair)
    {
        pair = axisPair;
    }
    
    void init()
    {
        motor = MotorState();
        canStats.reset();
        calibration.reset();
        controller.setLowGain();
        targetPosition = 0.0f;
        newResponse = false;
    }
    
    // ========== Coordinate Transformations ==========
    float motorToBodyFrame(float motorAngle) const
    {
        return (motorAngle - calibration.calculatedOffset) / direction;
    }
    
    float bodyToMotorFrame(float bodyAngle) const
    {
        return (bodyAngle * direction) + calibration.calculatedOffset;
    }
    
    // ========== Position Control ==========
    void setPositionSetpoint(float bodyPosition)
    {
        float constrained = constrain(bodyPosition, -QUARTER_CIRCLE, QUARTER_CIRCLE);
        targetPosition = bodyToMotorFrame(constrained);
    }
    
    // ========== Calibration Step - Call from coordinated update ==========
    void runCalibrationStep()
    {
        if (!calibration.enabled || calibration.completed)
            return;
        
        proxSensor.update();
        bool sensorActive = proxSensor.read();
        
        if (calibration.currentPhase >= calibration.targetPhase)
            return;  // Wait for target to advance
        
        switch (calibration.targetPhase)
        {
            case CalibrationPhase::SEEK_SENSOR:
                if (sensorActive)
                {
                    targetPosition = motor.position + direction * DELTA_THETA * 2.0f;
                    calibration.currentPhase = CalibrationPhase::SEEK_SENSOR;
                }
                else
                {
                    targetPosition = motor.position + direction * DELTA_THETA;
                }
                break;
                
            case CalibrationPhase::FIND_EDGE_1:
                if (sensorActive)
                {
                    targetPosition = motor.position + direction * DELTA_THETA;
                }
                else  // Falling edge 1 detected
                {
                    calibration.firstEdgeAngle = motor.position;
                    targetPosition = motor.position + direction * DELTA_THETA * 2.0f;
                    calibration.currentPhase = CalibrationPhase::FIND_EDGE_1;
                }
                break;
                
            case CalibrationPhase::REVERSE_TO_SENSOR:
                if (sensorActive)
                {
                    targetPosition = motor.position - direction * DELTA_THETA * 2.0f;
                    calibration.currentPhase = CalibrationPhase::REVERSE_TO_SENSOR;
                }
                else
                {
                    targetPosition = motor.position - direction * DELTA_THETA;
                }
                break;
                
            case CalibrationPhase::FIND_EDGE_2:
                if (sensorActive)
                {
                    targetPosition = motor.position - direction * DELTA_THETA;
                }
                else  // Falling edge 2 detected
                {
                    calibration.secondEdgeAngle = motor.position;
                    calibration.calculatedOffset = 
                        (calibration.firstEdgeAngle + calibration.secondEdgeAngle) / 2.0f;
                    targetPosition = motor.position - direction * DELTA_THETA * 2.0f;
                    calibration.currentPhase = CalibrationPhase::FIND_EDGE_2;
                }
                break;
                
            case CalibrationPhase::MOVE_TO_CENTER:
                targetPosition = calibration.calculatedOffset;
                // Check if we've essentially reached the target
                if (fabs(motor.position - calibration.calculatedOffset) < 0.01f)
                    calibration.currentPhase = CalibrationPhase::MOVE_TO_CENTER;
                else
                    calibration.currentPhase = CalibrationPhase::COMPLETE;
                break;
                
            case CalibrationPhase::COMPLETE:
                calibration.completed = true;
                calibration.enabled = false;
                calibration.currentPhase = CalibrationPhase::COMPLETE;
                break;
                
            default:
                break;
        }
    }
    
    // ========== Motor Control Commands ==========
    void enable()
    {
        if (!motor.armed)
            sendCommand(cmdStart);
    }
    
    void disable()
    {
        sendCommand(cmdStop);
        controller.setLowGain();

        
    }
    
    void sendVelocityCommand(float velocity)
    {
        uint8_t buf[MOTCTRL_FRAME_SIZE];
        velocity = constrain(velocity, -controller.velocityLimit, controller.velocityLimit);
        MCReqSpeedControl(buf, velocity);
        sendCommand(buf);
    }
    
    void sendPositionCommand()
    {
        uint8_t buf[MOTCTRL_FRAME_SIZE];
        MCReqPositionControl(buf, targetPosition);
        sendCommand(buf);
    }
    
    // ========== PD Control Execution ==========
    void executePDControl()
    {
        float posError = targetPosition - motor.position;
        float velCmd = controller.compute(posError, motor.velocity);
        sendVelocityCommand(velCmd);
    }
    
    // ========== CAN Communication ==========
    void sendCommand(const uint8_t *cmdBuffer)
    {
        if (!canBus)
            return;
        
        // Check for timeout on previous command
        if (canStats.awaitingResponse)
        {
            uint32_t elapsed = micros() - canStats.lastTxMicros;
            if (elapsed > CAN_TIMEOUT_US)
            {
                canStats.timeouts++;
                canStats.awaitingResponse = false;
            }
        }
        
        CAN_message_t msg;
        msg.id = CAN_MSG_ID;
        msg.len = MOTCTRL_FRAME_SIZE;
        memcpy(msg.buf, cmdBuffer, MOTCTRL_FRAME_SIZE);
        
        if (canBus->write(msg) > 0)
        {
            canStats.lastTxMicros = micros();
            canStats.txCount++;
            canStats.awaitingResponse = true;
        }
        else
        {
            canStats.txErrors++;
        }
    }
    
    void processResponse(const CAN_message_t &msg)
    {
        uint8_t buf[MOTCTRL_FRAME_SIZE];
        memcpy(buf, msg.buf, MOTCTRL_FRAME_SIZE);
        
        canStats.lastRxMicros = micros();
        canStats.rxCount++;
        
        // Calculate round-trip time
        if (canStats.awaitingResponse)
        {
            canStats.roundTripMicros = canStats.lastRxMicros - canStats.lastTxMicros;
            if (canStats.roundTripMicros > canStats.maxRoundTripMicros)
                canStats.maxRoundTripMicros = canStats.roundTripMicros;
            canStats.awaitingResponse = false;
        }
        
        uint8_t cmdId = buf[0];
        uint8_t result = buf[1];
        
        if (result == MOTCTRL_RES_SUCCESS)
        {
            switch (cmdId)
            {
                case MOTCTRL_CMD_START_MOTOR:
                    motor.armed = true;
                    break;
                    
                case MOTCTRL_CMD_STOP_MOTOR:
                    motor.armed = false;
                    break;
                    
                case MOTCTRL_CMD_POSITION_CONTROL:
                case MOTCTRL_CMD_SPEED_CONTROL:
                case MOTCTRL_CMD_TORQUE_CONTROL:
                {
                    float pos, vel, trq;
                    int8_t temp;
                    MCResControl(buf, &temp, &pos, &vel, &trq);
                    motor.updateFromResponse(temp, pos, vel, trq);
                    break;
                }
                
                default:
                    break;
            }
        }
        else
        {
            motor.hasError = true;
            canStats.rxErrors++;
            logInfof("AXIS %d ERR: CMD 0x%02X RES 0x%02X", 
                          axisId, cmdId, result);
        }
        
        newResponse = true;
    }
    
    // ========== Status ==========
    bool isReady() const
    {
        return motor.armed && (calibration.completed || calibration.enabled);
    }
    
    bool hasError() const
    {
        return motor.hasError || canStats.timeouts > 10;
    }
    
    // ========== Telemetry ==========
    uint8_t packFlags() const
    {
        uint8_t flags = 0;
        if (motor.armed)                flags |= (1 << 0);
        if (calibration.completed)      flags |= (1 << 1);
        if (calibration.enabled)        flags |= (1 << 2);
        if (motor.hasError)             flags |= (1 << 3);
        if (canStats.awaitingResponse)  flags |= (1 << 4);
        flags |= (static_cast<uint8_t>(calibration.currentPhase) & 0x07) << 5;
        return flags;
    }
    
    void fillTelemetry(AxisTelemetry &telem) const
    {
        telem.mode = controller.mode;
        telem.flags = packFlags();
        telem.setpoint = motorToBodyFrame(targetPosition);
        telem.theta = motorToBodyFrame(motor.position);
        telem.omega = motor.velocity / direction;  // Convert to body frame sign
        telem.tau = motor.torque;
        telem.temperature = motor.temperature;
        telem.rttMicros = (uint16_t)min(canStats.roundTripMicros, 65535UL);
        telem.txErrors = (uint8_t)min(canStats.txErrors, 255UL);
        telem.timeouts = (uint8_t)min(canStats.timeouts, 255UL);
    }
};

// ============================================================================
// AxisPair - Coordinated Dual Motor Controller
// ============================================================================
class AxisPair
{
public:
    static constexpr uint8_t SLAVE_ID_MIN = 1;
    static constexpr uint8_t SLAVE_ID_MAX = 3;
    
    Axis axisL;  // Odd axis ID (1, 3, or 5)
    Axis axisR;  // Even axis ID (2, 4, or 6)
    
    const uint8_t slaveId;
    bool globalError = false;
    
private:
    CANBusInterface *can1 = nullptr;
    CANBusInterface *can2 = nullptr;
    
    // Alternating update pattern to balance execution time
    volatile bool updateLeftFirst = true;
    
    // Telemetry state
    HexLink link;
    volatile bool telemetryReady = false;  // Set by update(), cleared by tick()
    volatile bool playing = false;          // Set by play cmd, cleared by stop cmd

public:
    AxisPair(uint8_t slaveId_)
        : axisL(slaveId_ * 2 - 1)   // Odd: 1, 3, or 5
        , axisR(slaveId_ * 2)       // Even: 2, 4, or 6
        , slaveId(slaveId_)
    {
        // Validate slave ID at construction
        // Note: Static assert not possible with runtime value, 
        // but we can set error state
        if (slaveId_ < SLAVE_ID_MIN || slaveId_ > SLAVE_ID_MAX)
        {
            globalError = true;
        }
        
        // Cross-reference for coordination
        axisL.attachPair(this);
        axisR.attachPair(this);
    }
    
    // ========== CAN Bus Configuration ==========
    void attachCAN(CANBusInterface *bus1, CANBusInterface *bus2)
    {
        can1 = bus1;
        can2 = bus2;
        
        // Left axis (odd ID) uses CAN1, Right axis (even ID) uses CAN2
        axisL.attachCAN(can1);
        axisR.attachCAN(can2);
    }
    
    void initCAN(uint32_t baudRate = 500000)
    {
        if (can1)
        {
            can1->begin();
            can1->setBaudRate(baudRate);
            can1->setMaxMB(16);
            can1->enableFIFO();
            can1->enableFIFOInterrupt();
            can1->mailboxStatus();
        }
        
        if (can2)
        {
            can2->begin();
            can2->setBaudRate(baudRate);
            can2->setMaxMB(16);
            can2->enableFIFO();
            can2->enableFIFOInterrupt();
            can2->mailboxStatus();
        }
    }
    
    void init()
    {
        axisL.init();
        axisR.init();
        initTelemetry();
    }
    
    // ========== Telemetry Initialization ==========
    void initTelemetry()
    {
        link = HexLink(slaveId, MASTER_ID);
        telemetryReady = false;
    }
    
    // ========== Main Update Loop (1kHz) ==========
    void update()
    {
        // Check for global error state
        if (globalError || axisL.hasError() || axisR.hasError())
        {
            globalError = true;
            return;
        }
        
        // Alternate which axis runs first to balance execution time
        if (updateLeftFirst)
        {
            updateAxis(axisL);
            updateAxis(axisR);
        }
        else
        {
            updateAxis(axisR);
            updateAxis(axisL);
        }
        updateLeftFirst = !updateLeftFirst;
        
        // Only publish telemetry when playing (trajectory active)
        if (playing)
        {
            telemetryReady = true;
        }
    }
    
    // ========== Coordinated Calibration ==========
    void startCalibration()
    {
        if (axisL.calibration.completed && axisR.calibration.completed)
            return;  // Already calibrated
        
        // Initialize both axes for calibration simultaneously
        axisL.calibration.reset();
        axisR.calibration.reset();
        axisL.calibration.enabled = true;
        axisR.calibration.enabled = true;
        axisL.calibration.targetPhase = CalibrationPhase::SEEK_SENSOR;
        axisR.calibration.targetPhase = CalibrationPhase::SEEK_SENSOR;
    }
    
    bool advanceCalibration()
    {
        // Only advance if BOTH axes have reached their current target
        if (!axisL.calibration.atTargetPhase() || !axisR.calibration.atTargetPhase())
            return false;  // Still in progress
        
        // Both ready - advance both simultaneously
        axisL.calibration.advanceTarget();
        axisR.calibration.advanceTarget();
        return true;
    }
    
    bool isCalibrated() const
    {
        return axisL.calibration.completed && axisR.calibration.completed;
    }
    
    bool isCalibrating() const
    {
        return (axisL.calibration.enabled && !axisL.calibration.completed) ||
               (axisR.calibration.enabled && !axisR.calibration.completed);
    }
    
    CalibrationPhase getCalibrationPhase() const
    {
        // Return the minimum phase (the one lagging behind)
        return (axisL.calibration.currentPhase < axisR.calibration.currentPhase) 
               ? axisL.calibration.currentPhase 
               : axisR.calibration.currentPhase;
    }
    
    // ========== Motor Control ==========
    void enable()
    {
        axisL.enable();
        axisR.enable();
    }
    
    void disable()
    {
        axisL.disable();
        axisR.disable();
    }
    
    void setHighGain()
    {
        axisL.controller.setHighGain();
        axisR.controller.setHighGain();
    }
    
    void setLowGain()
    {
        axisL.controller.setLowGain();
        axisR.controller.setLowGain();
    }
    
    void setPositions(float leftPos, float rightPos)
    {
        axisL.setPositionSetpoint(leftPos);
        axisR.setPositionSetpoint(rightPos);
    }
    
    void setPlaying(bool state)
    {
        playing = state;
    }
    
    bool isPlaying() const
    {
        return playing;
    }
    
    // ========== CAN Response Routing ==========
    void routeCANResponse(const CAN_message_t &msg)
    {
        // Route based on CAN bus ID in message
        switch (msg.bus)
        {
            case 1:  // CAN1 -> Left axis
                axisL.processResponse(msg);
                break;
            case 2:  // CAN2 -> Right axis
                axisR.processResponse(msg);
                break;
            default:
                break;
        }
    }
    
    // ========== Tick Handler - Telemetry Transmission ==========
    void tick()
    {
        // Only publish when update() signals fresh data from both motors
        if (!telemetryReady)
            return;
        telemetryReady = false;
        
        // Build status packet via HexLink
        rawPacket pkt = link.status();
        
        // Fill telemetry payload at bytes 5..55 (51 bytes fits in 55-byte payload)
        TelemetryPayload* payload = reinterpret_cast<TelemetryPayload*>(&pkt.buffer[5]);
        payload->timestamp = micros();
        payload->globalFlags = globalError ? 0x01 : 0x00;
        axisL.fillTelemetry(payload->left);
        axisR.fillTelemetry(payload->right);
        
        // Re-finalize CRC after filling payload (status() already set framing)
        FastCRC16 crc16;
        uint16_t c = crc16.xmodem(pkt.buffer, 61);
        pkt.buffer[61] = (c >> 8) & 0xFF;
        pkt.buffer[62] = c & 0xFF;
        
        // Transmit
        Serial.write(pkt.buffer, PACKET_SIZE);
    }
    
    // ========== Diagnostics ==========
    void printStatus()
    {
        logInfof("Slave %d | L(ax%d): armed=%d cal=%d pos=%.3f | R(ax%d): armed=%d cal=%d pos=%.3f",
                      slaveId,
                      axisL.axisId, axisL.motor.armed, axisL.calibration.completed, axisL.motor.position,
                      axisR.axisId, axisR.motor.armed, axisR.calibration.completed, axisR.motor.position);
    }
    
    void printCANStats()
    {
        logInfof("CAN Stats L: tx=%lu rx=%lu err=%lu to=%lu rtt=%luus max=%luus",
                      axisL.canStats.txCount, axisL.canStats.rxCount,
                      axisL.canStats.txErrors + axisL.canStats.rxErrors,
                      axisL.canStats.timeouts,
                      axisL.canStats.roundTripMicros, axisL.canStats.maxRoundTripMicros);
        logInfof("CAN Stats R: tx=%lu rx=%lu err=%lu to=%lu rtt=%luus max=%luus",
                      axisR.canStats.txCount, axisR.canStats.rxCount,
                      axisR.canStats.txErrors + axisR.canStats.rxErrors,
                      axisR.canStats.timeouts,
                      axisR.canStats.roundTripMicros, axisR.canStats.maxRoundTripMicros);
    }

private:
    void updateAxis(Axis &axis)
    {
        // Run calibration state machine if active
        if (axis.calibration.enabled && !axis.calibration.completed)
        {
            axis.runCalibrationStep();
        }
        
        // Execute control if armed and (calibrating or calibrated)
        if (axis.motor.armed && (axis.calibration.enabled || axis.calibration.completed))
        {
            axis.executePDControl();
        }
    }
};

#endif // DELF_H