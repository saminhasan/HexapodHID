#ifndef SERIAL_BUFFER_H
#define SERIAL_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <FastCRC.h>

class SerialBuffer
{
private:
    uint8_t *buf;
    size_t SIZE; // must be power-of-two
    size_t MASK; // SIZE - 1
    size_t head;
    size_t tail;
    size_t count;

public:
    SerialBuffer(uint8_t *externalBuffer, size_t size) : buf(externalBuffer), SIZE(size), MASK(size - 1), head(0), tail(0), count(0)
    {
    }

    inline size_t size() const { return count; }
    inline bool isEmpty() const { return count == 0; }
    inline bool isFull() const { return count == SIZE; }

    // logical index from tail
    inline const uint8_t &operator[](size_t index) const
    {
        return buf[(tail + index) & MASK];
    }

    inline bool push(uint8_t item)
    {
        if (count >= SIZE)
            return false;
        buf[head] = item;
        head = (head + 1) & MASK;
        count++;
        return true;
    }

    inline bool pop(uint8_t &item)
    {
        if (!count)
            return false;
        item = buf[tail];
        tail = (tail + 1) & MASK;
        count--;
        return true;
    }

    inline size_t discard(size_t n)
    {
        size_t d = (n < count) ? n : count;
        tail = (tail + d) & MASK;
        count -= d;
        return d;
    }

    // Skip bytes UNTIL target is at tail (target is NOT consumed).
    // If not found, drops everything .
    bool readBytesUntil(uint8_t target)
    {
        if (!count)
            return false;

        size_t firstChunk = (count < (SIZE - tail)) ? count : (SIZE - tail);
        void *p = memchr(buf + tail, target, firstChunk);

        if (p)
        {
            size_t offset = (uint8_t *)p - (buf + tail);
            tail = (tail + offset) & MASK; // NOTE: no +1
            count -= offset;               // NOTE: target not consumed
            return true;
        }

        if (count > firstChunk)
        {
            size_t secondChunk = count - firstChunk;
            p = memchr(buf, target, secondChunk);
            if (p)
            {
                size_t offset = (uint8_t *)p - buf;
                tail = offset & MASK; // target at tail
                count -= (firstChunk + offset);
                return true;
            }
        }

        tail = (tail + count) & MASK;
        count = 0;
        return false;
    }
    // Copy n bytes starting at (tail + offset) into dest, WITHOUT consuming.
    // Returns bytes copied (0 if not enough data).
    size_t peekBytes(uint8_t *dest, size_t n, size_t offset = 0) const
    {
        if (!dest || n == 0)
            return 0;
        if (offset + n > count)
            return 0;

        size_t start = (tail + offset) & MASK;

        size_t toEnd = SIZE - start;
        size_t c1 = (n < toEnd) ? n : toEnd;
        memcpy(dest, buf + start, c1);

        if (c1 == n)
            return n;

        size_t c2 = n - c1;
        memcpy(dest + c1, buf, c2);
        return n;
    }
    uint16_t crc16(size_t n) const
    {
        if (n > count)
            return 0;

        static FastCRC16 crc; // reused, no re-init cost
        uint16_t crc_residue;
        size_t start = tail;
        size_t toEnd = SIZE - start;
        size_t c1 = (n < toEnd) ? n : toEnd;

        crc_residue = crc.xmodem_upd(buf + start, c1);

        if (c1 < n)
        {
            crc_residue = crc.xmodem_upd(buf, n - c1);
        }

        return crc_residue;
    }
    size_t readBytes(uint8_t *dest, size_t n)
    {
        if (!dest || n == 0 || count == 0)
            return 0;

        size_t toRead = (n < count) ? n : count;

        size_t tailToEnd = SIZE - tail;
        size_t chunk1 = (toRead < tailToEnd) ? toRead : tailToEnd;
        memcpy(dest, buf + tail, chunk1);
        tail = (tail + chunk1) & MASK;
        count -= chunk1;

        if (chunk1 == toRead)
            return chunk1;

        size_t chunk2 = toRead - chunk1;
        memcpy(dest + chunk1, buf + tail, chunk2);
        tail = (tail + chunk2) & MASK;
        count -= chunk2;

        return chunk1 + chunk2;
    }

    size_t writeBytes(const uint8_t *src, size_t n)
    {
        if (!src || n == 0)
            return 0;

        size_t space = SIZE - count;
        if (!space)
            return 0;

        size_t toWrite = (n < space) ? n : space;

        size_t headToEnd = SIZE - head;
        size_t chunk1 = (toWrite < headToEnd) ? toWrite : headToEnd;
        memcpy(buf + head, src, chunk1);
        head = (head + chunk1) & MASK;
        count += chunk1;

        if (chunk1 == toWrite)
            return chunk1;

        size_t chunk2 = toWrite - chunk1;
        memcpy(buf + head, src + chunk1, chunk2);
        head = (head + chunk2) & MASK;
        count += chunk2;

        return chunk1 + chunk2;
    }

    // Works best with serial.setTimeout(0)
    // Returns number of bytes read, sets dropped to bytes that couldn't fit
    template <typename StreamType>
    size_t readStream(StreamType &serial, size_t* dropped = nullptr)
    {
        size_t avail = (size_t)serial.available();
        size_t space = SIZE - count;
        if (!avail)
            return 0;
        
        // Track overflow
        if (dropped && avail > space) {
            *dropped += (avail - space);
        }
        
        if (!space)
            return 0;

        size_t toRead = (avail < space) ? avail : space;

        size_t headToEnd = SIZE - head;
        size_t chunk1 = (toRead < headToEnd) ? toRead : headToEnd;

        size_t n1 = serial.readBytes((char *)(buf + head), chunk1);
        head = (head + n1) & MASK;
        count += n1;

        if (n1 < chunk1 || n1 == toRead)
            return n1;

        size_t chunk2 = toRead - n1;
        size_t n2 = serial.readBytes((char *)(buf + head), chunk2);
        head = (head + n2) & MASK;
        count += n2;

        return n1 + n2;
    }
};

#endif // SERIAL_BUFFER_H
