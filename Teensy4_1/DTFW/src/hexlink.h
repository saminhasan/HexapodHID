#pragma once
#include <stdint.h>
#include <string.h>
#include <cstdarg>
#include <cstdio>
#include <FastCRC.h>
#include <Arduino.h>

#include "constants.h"
#include "utils.h"  // rawPacket definition

// Teensy-side packet builder mirroring Python hexlink.py (no RW byte).
// All packets are 64 bytes: [0]=START, [1]=FROM, [2]=TO, [3]=SEQ,
// [4]=MSGID, [5..60]=payload, [61..62]=CRC16-XMODEM over bytes 0..60, [63]=END.
class HexLink {
public:
    HexLink(uint8_t from_id = MASTER_ID, uint8_t to_id = PC_ID)
        : from(from_id), to(to_id), seq(0) {}

    rawPacket ping()        { return make_simple(MSGID_PING); }
    rawPacket pong()        { return make_simple(MSGID_PONG); }
    rawPacket enable()      { return make_simple(MSGID_ENABLE); }
    rawPacket disable()     { return make_simple(MSGID_DISABLE); }
    rawPacket calibrate()   { return make_simple(MSGID_CALIBRATE); }
    rawPacket stage()       { return make_simple(MSGID_STAGE); }
    rawPacket park()        { return make_simple(MSGID_PARK); }
    rawPacket play()        { return make_simple(MSGID_PLAY); }
    rawPacket pause()       { return make_simple(MSGID_PAUSE); }
    rawPacket stop()        { return make_simple(MSGID_STOP); }
    rawPacket estop()       { return make_simple(MSGID_ESTOP); }
    rawPacket reset()       { return make_simple(MSGID_RESET); }
    rawPacket status()      { return make_simple(MSGID_STATUS); }

    rawPacket jog(const float pose[NUM_AXIS]) {
        rawPacket pkt = base_packet(MSGID_JOG);
        memcpy(pkt.buffer + 5, pose, NUM_AXIS * sizeof(float));
        finalize(pkt);
        return pkt;
    }

    // Formatted info logger: chunks arbitrary text into 64-byte INFO packets and writes via Serial.
    // Layout per packet: [5]=flags (bit0=more chunks), [6..60]=ASCII payload (NUL in final chunk).
    void logInfo(const char* fmt, ...) {
        if (!fmt) return;

        va_list args;
        va_start(args, fmt);
        int len = vsnprintf(nullptr, 0, fmt, args);
        va_end(args);
        if (len < 0) return;

        const size_t total = static_cast<size_t>(len) + 1; // include NUL terminator
        char msg[total];
        va_start(args, fmt);
        vsnprintf(msg, total, fmt, args);
        va_end(args);

        constexpr size_t payloadBytes = 55; // bytes 6..60
        const size_t numPackets = (total + payloadBytes - 1) / payloadBytes;

        uint8_t out[numPackets * PACKET_SIZE];
        size_t offset = 0;

        for (size_t i = 0; i < numPackets; ++i) {
            uint8_t* p = out + i * PACKET_SIZE;
            memset(p, 0, PACKET_SIZE);

            p[0] = START_BYTE;
            p[1] = from;
            p[2] = to;
            p[3] = seq++;
            p[4] = MSG_ID_INFO;

            size_t remaining = total - offset;
            size_t chunk = (remaining < payloadBytes) ? remaining : payloadBytes;
            bool more = (offset + chunk) < total;

            p[5] = more ? 0x01 : 0x00; // flags
            memcpy(p + 6, msg + offset, chunk);

            const uint16_t c = crc16.xmodem(p, 61); // bytes 0..60
            p[61] = (c >> 8) & 0xFF;
            p[62] = c & 0xFF;
            p[63] = END_BYTE;

            offset += chunk;
        }

        Serial.write(out, numPackets * PACKET_SIZE);
    }



private:
    uint8_t from;
    uint8_t to;
    uint8_t seq;
    FastCRC16 crc16;

    rawPacket make_simple(uint8_t msgid) {
        rawPacket pkt = base_packet(msgid);
        finalize(pkt);
        return pkt;
    }

    rawPacket base_packet(uint8_t msgid) {
        rawPacket pkt{};
        memset(pkt.buffer, 0, PACKET_SIZE);
        pkt.buffer[0] = START_BYTE;
        pkt.buffer[1] = from;
        pkt.buffer[2] = to;
        pkt.buffer[3] = seq++;
        pkt.buffer[4] = msgid;
        // payload is zeroed already
        pkt.buffer[PACKET_SIZE - 1] = END_BYTE;
        return pkt;
    }

    void finalize(rawPacket &pkt) {
        const uint16_t c = crc16.xmodem(pkt.buffer, 61); // bytes 0..60
        pkt.buffer[61] = (c >> 8) & 0xFF;
        pkt.buffer[62] = c & 0xFF;
        // pkt.buffer[63] is END_BYTE set in base_packet
    }

    static inline void write_u32(uint8_t *dst, uint32_t v) {
        dst[0] = (uint8_t)(v & 0xFF);
        dst[1] = (uint8_t)((v >> 8) & 0xFF);
        dst[2] = (uint8_t)((v >> 16) & 0xFF);
        dst[3] = (uint8_t)((v >> 24) & 0xFF);
    }
};
