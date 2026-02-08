#ifndef UTILS_H
#define UTILS_H
#include <Arduino.h>

#include "constants.h"
#include <FastCRC.h>
struct rawPacket // todo, remove this
{
  uint8_t buffer[PACKET_SIZE];
};
void printRow(const float* row)
{
  Serial.printf("Row: [%f, %f, %f, %f, %f, %f]\n", row[0], row[1], row[2], row[3], row[4], row[5]);
}
struct TrajectoryTable
{
private:
    float (*buf)[NUM_AXIS];              // external: [MAX_ROWS][NUM_AXIS] // 6 axis
    uint32_t maxLength;
    uint32_t writeIndex;


public:
    TrajectoryTable(float (*externalBuffer)[NUM_AXIS], uint32_t capacityRows)
        : buf(externalBuffer), maxLength(capacityRows), writeIndex(0)
    {
    }

    inline uint32_t capacity() const { return maxLength; }
    inline uint32_t length() const { return writeIndex; }
    inline bool isEmpty() const { return writeIndex == 0; }
    inline bool isFull()  const { return writeIndex >= maxLength; }

	inline const float* operator[](uint32_t index) const
	{
		static const float zero[NUM_AXIS] = {0.0f};
		uint32_t n = writeIndex;
		return n ? buf[index % n] : zero;
	}

	inline bool pushRow(const float* pose)
	{
		if (writeIndex >= maxLength) return false;
		if (!pose) return false;
		memcpy(buf[writeIndex], pose, sizeof(buf[0])); // 24 bytes
		writeIndex++;
		return true;
	}
    inline bool pushRowBytes(const uint8_t* payload)
    {
        if (writeIndex >= maxLength) return false;
        if (!payload) return false;
        memcpy(buf[writeIndex], payload, sizeof(buf[0])); // 24 bytes
        writeIndex++;
        return true;
    }

    inline void reset()
    {
        memset(buf, 0, (size_t)maxLength * sizeof(buf[0]));
        writeIndex = 0;
    }

    inline uint32_t checksum() const
    {
        // FastCRC's crc32() takes uint16_t length (max 65535 bytes)
        // Must process in chunks for large trajectories
        FastCRC32 crc;
        const uint8_t* data = (const uint8_t*)buf;
        size_t totalBytes = (size_t)writeIndex * sizeof(buf[0]);
        
        if (totalBytes == 0) return 0;
        
        // First chunk - use crc32() to initialize
        const size_t CHUNK_SIZE = 65534;  // Max safe size for uint16_t
        size_t firstChunk = (totalBytes < CHUNK_SIZE) ? totalBytes : CHUNK_SIZE;
        uint32_t result = crc.crc32(data, (uint16_t)firstChunk);
        
        // Remaining chunks - use crc32_upd() to continue
        size_t offset = firstChunk;
        while (offset < totalBytes) {
            size_t remaining = totalBytes - offset;
            size_t chunk = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
            result = crc.crc32_upd(data + offset, (uint16_t)chunk);
            offset += chunk;
        }
        
        return result;
    }

};







#endif // UTILS_H
