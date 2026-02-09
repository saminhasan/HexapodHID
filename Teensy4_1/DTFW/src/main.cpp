#include "globals.h"

static bool onRawHID1(uint32_t usage, const uint8_t* data, uint32_t len);
static bool onRawHID2(uint32_t usage, const uint8_t* data, uint32_t len);
static bool onRawHID3(uint32_t usage, const uint8_t* data, uint32_t len);
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  myTimer.begin(timerISR, 1000); // 1ms interval
  while (!Serial)
    yield();
  Serial.setTimeout(0);
  myusb.begin();
  hid1.attachReceive(onRawHID1);
  hid2.attachReceive(onRawHID2);
  hid3.attachReceive(onRawHID3);
  
  // Serial.printf("RX buffer: %lu bytes\n", usb_serial_rx_buffer_size);
  logInfof("TX buffer: %d bytes", Serial.availableForWrite());
  logInfof("System Initialized.");
}

static bool handleRawHIDPacket(const char* tag, const uint8_t* data, uint32_t len)
{
  if (!data || len == 0) return false;
  logInfof("RHID[%s] recv %lu bytes", tag, (unsigned long)len);
  // TODO: validate and forward hexlink packets to PC or aggregate
  return true;
}

static bool onRawHID1(uint32_t usage, const uint8_t* data, uint32_t len)
{
  return handleRawHIDPacket("A", data, len);
}
static bool onRawHID2(uint32_t usage, const uint8_t* data, uint32_t len)
{
  return handleRawHIDPacket("B", data, len);
}
static bool onRawHID3(uint32_t usage, const uint8_t* data, uint32_t len)
{
  return handleRawHIDPacket("C", data, len);
}
//-------------------------------------------------------------------------------------------------------

void ping()
{
  Serial.println("PING");
  return;
}
void pong()
{
  logInfof("PONG");
  return;
}
void enable()
{
  logInfof("ENABLE");
  return;
}
void disable()
{
  logInfof("DISABLE");
  return;
}
void calibrate()
{
  logInfof("CALIBRATE");
  return;
}
void estop()
{
  logInfof("ESTOP");
  return;
}
void reset()
{
  logInfof("RESET");
  return;
}
void jog(const uint8_t *buffer)
{
  float pose[6];
  memcpy(pose, buffer + 5, sizeof(float) * 6);
  logInfof("Pose: [%f, %f, %f, %f, %f, %f]", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
  return;
}
void upload(const uint8_t *buffer)
{
  if (!trajectory.pushRowBytes(buffer + 5))
  {
    logInfof("Trajectory buffer full at %u rows.", trajectory.length());
  }
  // Don't print every row - it slows down processing and causes buffer overflow!
  return;
}
void status()
{
  logInfof("STATUS");
  return;
}
void play()
{ // stage first
  // set gain high, feed rate to 100%, mode : play
  fro.setFeedrate(1.0f);
  logInfof("PLAY");
  return;
}
void pause()
{ // set feedrate to zero -> pause motion -> mode : pause
  fro.setFeedrate(0.0f);
  logInfof("PAUSE");
  return;
}
void stop()
{
  fro.setFeedrate(0.0f);
  // override feedrate to zero -> stop motion -> mode : 0 ->park()
  trajectory.reset();
  // Reset counters for next upload
  receivedPackets = 0;
  droppedBytes = 0;
  logInfof("STOP");
  return;
}

void validate(const uint8_t *buffer)
{

  logInfof("VALIDATE :");

  uint32_t pktLen, pktCrc;
  memcpy(&pktLen, buffer + 5, 4);
  memcpy(&pktCrc, buffer + 9, 4);
  uint32_t ramCrc = trajectory.checksum();
  uint32_t ramLen = trajectory.length();
  
  // Debug: print stats
  logInfof("Received packets: %u, Dropped bytes: %u", receivedPackets, droppedBytes);
  
  // Debug: print first row's raw bytes
  if (ramLen > 0) {
    const uint8_t* firstRow = (const uint8_t*)trajectory[0];
    char hexline[3 * 24 + 1] = {0};
    size_t h = 0;
    for (int i = 0; i < 24 && h + 2 < sizeof(hexline); i++) {
      h += snprintf(hexline + h, sizeof(hexline) - h, "%02X", firstRow[i]);
    }
    logInfof("First row bytes: %s", hexline);
  }
  
  if (pktLen != ramLen)
    logInfof("Length Mismatch! Expected: %u, Actual: %u", pktLen, ramLen);
  else if (pktCrc != ramCrc)
    logInfof("CRC32 Mismatch! Expected: 0x%08X, Actual: 0x%08X", pktCrc, ramCrc);
  else
  {
    hasData = true;
    logInfof("OK");
    logInfof("Trajectory Validated: Length=%u", ramLen);
  }

  return;
}
//-------------------------------------------------------------------------------------------------------

static void handlePacket(const uint8_t *buffer)
{
  receivedPackets++;
  const uint8_t seq = buffer[3];
  // if (seq != ((lastSeq + 1) & 0xFF))
  // {
  //   Serial.printf("Packet Loss Detected: lastSeq=%u, currentSeq=%u\n", lastSeq, seq);
  //   lostPackets ++;
  // }
  const uint8_t msgid = buffer[4];

  switch (msgid)
  {
  case MSGID_PING:
  {
    ping();
    break;
  }
  case MSGID_PONG:
  {
    pong();
    break;
  }
  case MSGID_ENABLE:
  {
    enable();
    break;
  }
  case MSGID_DISABLE:
  {
    disable();
    break;
  }
  case MSGID_CALIBRATE:
  {
    calibrate();
    break;
  }

  case MSGID_UPLOAD:
  {
    upload(buffer);
    break;
  }
  case MSGID_PLAY:
  {
    play();
    break;
  }
  case MSGID_PAUSE:
  {
    pause();
    break;
  }
  case MSGID_STOP:
  {
    stop();
    hasData = false;
    break;
  }
  case MSGID_ESTOP:
  {
    estop();
    break;
  }
  case MSGID_RESET:
  {
    reset();
    break;
  }
  case MSGID_JOG:
  {
    jog(buffer);
    break;
  }
  case MSGID_VALIDATE:
  {
    validate(buffer);
    break;
  }
  case MSGID_STAGE:
  {
    logInfof("STAGE");
    break;
  }
  case MSGID_PARK:
  {
    logInfof("PARK");
    break;
  }

  case MSGID_STATUS:
  {
    status();
    break;
  }
  default:
  {
    logInfof("Unknown MSGID: 0x%02X", msgid);
  }
    // one print at the end
    // Serial.printf("MSGID: 0x%02X, seq=%u\n", msgid, seq);
    lastSeq = seq;
    processedPackets++;
  }
}

static bool parseSerial(SerialBuffer &sb, rawPacket &pkt)
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
  sb.readBytes(pkt.buffer, PACKET_SIZE);
  return true;
}

void serialEvent(SerialBuffer &sb)
{
  size_t dropped = 0;
  sb.readStream(Serial, &dropped);
  if (dropped) droppedBytes += dropped;
}

void loop()
{
  myusb.Task();

  serialEvent(rb);
  if (parseSerial(rb, pkt))
  {
    handlePacket(pkt.buffer);
  }
  if (newData)
  {
    logInfof("POSE: [%f, %f, %f, %f, %f, %f]", outPose[0], outPose[1], outPose[2], outPose[3], outPose[4], outPose[5]);
    newData = false;
  }
}
