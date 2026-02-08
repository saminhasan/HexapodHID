#include <usb_desc.h>
#include "globals.h"

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  myTimer.begin(timerISR, 1000); // 1ms interval
  while (!Serial)
    yield();
  Serial.setTimeout(0);

  
  // Serial.printf("RX buffer: %lu bytes\n", usb_serial_rx_buffer_size);
  Serial.printf("TX buffer: %d bytes\n", Serial.availableForWrite());
  Serial.printf("System Initialized.\n");
}
//-------------------------------------------------------------------------------------------------------

void ping()
{
  Serial.println("PING");
  return;
}
void pong()
{
  Serial.println("PONG");
  return;
}
void enable()
{
  Serial.println("ENABLE");
  return;
}
void disable()
{
  Serial.println("DISABLE");
  return;
}
void calibrate()
{
  Serial.println("CALIBRATE");
  return;
}
void estop()
{
  Serial.println("ESTOP");
  return;
}
void reset()
{
  Serial.println("RESET");
  return;
}
void jog(const uint8_t *buffer)
{
  float pose[6];
  memcpy(pose, buffer + 6, sizeof(float) * 6);
  Serial.printf("Pose: [%f, %f, %f, %f, %f, %f]\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
  return;
}
void upload(const uint8_t *buffer)
{
  if (!trajectory.pushRowBytes(buffer + 6))
  {
    Serial.printf("Trajectory buffer full at %u rows.\n", trajectory.length());
  }
  // Don't print every row - it slows down processing and causes buffer overflow!
  return;
}
void status()
{
  Serial.println("STATUS");
  return;
}
void play()
{ // stage first
  // set gain high, feed rate to 100%, mode : play
  fro.setFeedrate(1.0f);
  Serial.println("PLAY");
  return;
}
void pause()
{ // set feedrate to zero -> pause motion -> mode : pause
  fro.setFeedrate(0.0f);
  Serial.println("PAUSE");
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
  Serial.println("STOP");
  return;
}

void validate(const uint8_t *buffer)
{

  Serial.print("VALIDATE : ");

  uint32_t pktLen, pktCrc;
  memcpy(&pktLen, buffer + 6, 4);
  memcpy(&pktCrc, buffer + 10, 4);
  uint32_t ramCrc = trajectory.checksum();
  uint32_t ramLen = trajectory.length();
  
  // Debug: print stats
  Serial.printf("\n  Received packets: %u, Dropped bytes: %u\n", receivedPackets, droppedBytes);
  
  // Debug: print first row's raw bytes
  if (ramLen > 0) {
    const uint8_t* firstRow = (const uint8_t*)trajectory[0];
    Serial.print("  First row bytes: ");
    for (int i = 0; i < 24; i++) {
      Serial.printf("%02X", firstRow[i]);
    }
    Serial.println();
  }
  
  if (pktLen != ramLen)
    Serial.printf("Length Mismatch! Expected: %u, Actual: %u\n", pktLen, ramLen);
  else if (pktCrc != ramCrc)
    Serial.printf("CRC32 Mismatch! Expected: 0x%08X, Actual: 0x%08X\n", pktCrc, ramCrc);
  else
  {
    hasData = true;
    Serial.print("OK\n");
    Serial.printf("Trajectory Validated: Length=%u, \n", ramLen);
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
  // const uint8_t rw = buffer[5];

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
    Serial.println("STAGE");
    break;
  }
  case MSGID_PARK:
  {
    Serial.println("PARK");
    break;
  }

  case MSGID_STATUS:
  {
    status();
    break;
  }
  default:
  {
    Serial.printf("Unknown MSGID: 0x%02X\n", msgid);
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
  serialEvent(rb);
  if (parseSerial(rb, pkt))
  {
    handlePacket(pkt.buffer);
  }
  if (newData)
  {
    Serial.printf("POSE: [%f, %f, %f, %f, %f, %f]\n", outPose[0], outPose[1], outPose[2], outPose[3], outPose[4], outPose[5]);
    newData = false;
  }
}
