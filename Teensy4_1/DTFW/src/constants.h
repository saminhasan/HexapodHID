#pragma once
#include <stdint.h>

#define PACKET_SIZE  64

#define START_BYTE   0xFE
#define MASTER_ID    0xFE
#define PC_ID        0xFF
#define END_BYTE     0xFF

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

static constexpr uint16_t CRC_RESIDUE = 0x0000; // verify once; change if needed


static constexpr uint32_t EXT_BYTES = 16u * 1024u * 1024u;   // 16,777,216
static constexpr uint32_t NUM_AXIS = 6u;
static constexpr uint32_t MAX_ROWS  = EXT_BYTES / NUM_AXIS / sizeof(float); // 699, 050
