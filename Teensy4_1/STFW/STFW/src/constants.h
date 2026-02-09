#pragma once
#include <stdint.h>

// ---- packet framing ----
#define PACKET_SIZE  64

#define START_BYTE   0xFE
#define MASTER_ID    0xFE
#define PC_ID        0xFF
#define END_BYTE     0xFF

// ---- message IDs ----
#define MSGID_PING    0x01
#define MSGID_PONG    0x02
#define MSGID_ENABLE  0x03
#define MSGID_DISABLE 0x04
#define MSGID_UPLOAD  0x05
#define MSGID_PLAY    0x06
#define MSGID_PAUSE   0x07
#define MSGID_STOP    0x08
#define MSGID_ESTOP   0x09
#define MSGID_RESET   0x0A
#define MSGID_JOG     0x0B
#define MSGID_CALIBRATE 0x0E
#define MSGID_STAGE 0x0F
#define MSGID_PARK   0x10
#define MSG_ID_INFO   0x11
#define MSGID_VALIDATE 0x0C
#define MSGID_STATUS  0xFF

// ---- raw packet buffer ----
struct rawPacket
{
  uint8_t buffer[PACKET_SIZE];
};

// ---- axis count (shared by hexlink jog, trajectory, etc.) ----
static constexpr uint32_t NUM_AXIS = 6u;

