#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "ublox.h"

UBloxInterpreter uBlox;

#ifdef _QUEUE_H
int UBloxInterpreter::find(Queue *q, Header *header)
{
  int offset;
  unsigned short length = 0;

  lasterror = 0;

  while (qlength(q) > 0)
  {
    // find Start Delimiter
    if ((offset = qfind(q, UBLOX_SYNC)) == -QUEUE_ERROR_NOTFOUND)
    {
      // no Start Delimiter => clear queue
      qclear(q);
      return 0;
    }
    else if (offset > 0)
    {
      // discard first <offset> bytes from queue
      // printf("uBlox: discarded %d bytes from queue\n", offset);
      qread(q, NULL, offset);
    }

    if (qlength(q) < 8) return 0;
    if (header != NULL) qcopy(q, header, 4, 2);
    qcopy(q, &length, 2, 4);
    length += 8;
    if (length > UBLOX_MAXLENGTH)
    {
      qread(q, NULL, 2);    // read Start Delimiter and discard package
      continue;
    }
    break;
  }

  if (qlength(q) < length) return 0;      // wait until whole packet is received
  return length;
}

int UBloxInterpreter::decode(Queue *q, void *payload, unsigned *payloadLength, Header *header)
{
  int length;
  char *check;
  char checksumA = 0, checksumB = 0;
  int i;
  Header header_local;
  if (header == NULL) header = &header_local;

  lasterror = 0;

  if ((length = findInQueue(q, NULL)) <= 0) return length;

  qread(q, NULL, 2);    // sync
  qread(q, header, sizeof(*header));
  if (header->payloadLength > *payloadLength) return error(UBLOX_ERROR_BIGPACKET);

  // checksum calculations
  checksumA += header->classId;
  checksumB += checksumA;
  checksumA += header->messageId;
  checksumB += checksumA;
  checksumA += (length & 0xFF);
  checksumB += checksumA;
  checksumA += (length >> 8);
  checksumB += checksumA;

  qread(q, payload, header->payloadLength);
  for (check = (char *) payload, i = 0; i < header->payloadLength; i++)
  {
    checksumA += check[i];
    checksumB += checksumA;
  }
  if (checksumA != qget(q) || checksumB != qget(q)) return error(UBLOX_ERROR_CHECKSUM);

  return length;
}
#endif // _QUEUE_H

#if defined(RINGBUFFER_H_)
int UBloxInterpreter::find(RingBuffer<char> &buffer, Header *header)
{
  int offset;
  unsigned short length = 0;

  lasterror = 0;

  while (buffer.length() > 0)
  {
    // find Start Delimiter
    if ((offset = buffer.find(UBLOX_SYNC, sizeof(UBLOX_SYNC))) < 0)
    {
      // no Start Delimiter => clear queue
      buffer.flush();
      return 0;

    }
    else if (offset > 0)
    {
      // discard first <offset> bytes from queue
      // printf("uBlox: discarded %d bytes from queue\n", offset);
      buffer.read(NULL, offset);
    }

    if (buffer.length() < 8) return 0;
    if (header != NULL) buffer.copy((char *) header, sizeof(*header), 2);
    buffer.copy((char *) &length, sizeof(length), 4);
    length += 8;
    if (length > UBLOX_MAXLENGTH)
    {
      buffer.read(NULL, 2);    // read Start Delimiter and discard package
      continue;
    }
    break;
  }

  if (buffer.length() < length) return 0;   // wait until whole packet is received
  return length;
}

int UBloxInterpreter::decode(RingBuffer<char> &buffer, void *payload, unsigned *payloadLength, Header *header)
{
  char *check;
  char checksumA = 0, checksumB = 0;
  int i;
  Header header_local;
  if (header == NULL) header = &header_local;

  lasterror = 0;

  if (buffer.find(UBLOX_SYNC, sizeof(UBLOX_SYNC)) != 0) return error(UBLOX_ERROR_NOUBLOX);
  if (buffer.length() < 8) return error(UBLOX_ERROR_NOUBLOX);

  buffer.copy((char *) header, sizeof(*header), 2);
  if (buffer.length() < (unsigned)(header->payloadLength + UBLOX_FRAMESIZE)) return error(UBLOX_ERROR_NOUBLOX);
  if (header->payloadLength > *payloadLength) return error(UBLOX_ERROR_BIGPACKET);
  *payloadLength = header->payloadLength;

  buffer.read(NULL, 2 + sizeof(*header));       // sync + header

  // checksum calculations
  checksumA += header->classId;
  checksumB += checksumA;
  checksumA += header->messageId;
  checksumB += checksumA;
  checksumA += (header->payloadLength & 0xFF);
  checksumB += checksumA;
  checksumA += (header->payloadLength >> 8);
  checksumB += checksumA;

  buffer.read((char *) payload, *payloadLength);
  for (check = (char *) payload, i = 0; i < (int) *payloadLength; i++)
  {
    checksumA += check[i];
    checksumB += checksumA;
  }
  if (checksumA != buffer.get() || checksumB != buffer.get()) return error(UBLOX_ERROR_CHECKSUM);

  return *payloadLength + UBLOX_FRAMESIZE;
}
#endif // RINGBUFFER_H_

int UBloxInterpreter::findNext(char **pos, unsigned *size)
{
  if ((*size >= UBLOX_FRAMESIZE) && ((*pos)[0] == UBLOX_SYNC[0]) && ((*pos)[1] == UBLOX_SYNC[1]))
  {
    uint16_t length = (uint8_t)(*pos)[4] + ((uint8_t)(*pos)[5] * 256) + UBLOX_FRAMESIZE;
    if (length > *size) length = *size;
    *pos += length;
    *size -= length;
  }

  return find(pos, size, NULL);
}

int UBloxInterpreter::find(char **pos, unsigned *size, Header *header)
{
  uint16_t payloadLength = 0;

  lasterror = 0;
  while (*size > 1)
  {
    if ((*pos)[0] == UBLOX_SYNC[0] && (*pos)[1] == UBLOX_SYNC[1]) break;
    (*pos) ++; (*size)--;
  }
  if (*size < UBLOX_FRAMESIZE) return 0;

  payloadLength = (uint8_t)(*pos)[4] + ((uint8_t)(*pos)[5] * 256);
  if (header != NULL)
  {
    header->classId = (*pos)[2];
    header->messageId = (*pos)[3];
    header->payloadLength = payloadLength;
  }

  return payloadLength + UBLOX_FRAMESIZE;
}

int UBloxInterpreter::decodePacket(char *buffer, unsigned size, Packet **packet)
{
  int ret;

  ret = decode(buffer, size, NULL, NULL, NULL);
  if (ret > 0)
  {
    *packet = (Packet *) buffer;
  }
  return ret;
}

int UBloxInterpreter::decode(char *buffer, unsigned size, void **payload, unsigned *payloadLength, Header *header)
{
  char checksumA = 0, checksumB = 0;
  int i = 0;
  unsigned pLength;

  lasterror = 0;
  if (size < 8) return 0;

  if (buffer[i++] != UBLOX_SYNC[0]) return error(UBLOX_ERROR_NOUBLOX);
  if (buffer[i++] != UBLOX_SYNC[1]) return error(UBLOX_ERROR_NOUBLOX);
  if (header != NULL)
  {
    header->classId = buffer[i++];
    header->messageId = buffer[i++];
  }
  else
  {
    i += 2;
  }
  pLength  = (uint8_t) buffer[i++];
  pLength |= (uint8_t) buffer[i++] << 8;
  if (header != NULL)
  {
    header->payloadLength = pLength;
  }

  if (size < pLength + UBLOX_FRAMESIZE) return 0;
  if (payload != NULL) *payload = &buffer[i];
  if (payloadLength != NULL) *payloadLength = pLength;

  i += pLength;
  checksumA = buffer[i++];
  checksumB = buffer[i++];

  // checksum calculations
  for (i -= 3; i >= 2; i--)
  {
    checksumB -= checksumA;
    checksumA -= buffer[i];
  }
  if (checksumA != 0 || checksumB != 0) return error(UBLOX_ERROR_CHECKSUM);

  return pLength + UBLOX_FRAMESIZE;
}

int UBloxInterpreter::remove(char *buffer, unsigned *size, char *pos)
{
  if (pos == buffer) return 0;
  if (pos > buffer + *size) pos = buffer + *size;
  *size -= pos - buffer;
  memmove(buffer, pos, *size);
  return (pos - buffer);
}

int UBloxInterpreter::decodeAttributes(Packet *packet, unsigned offset, Attribute* attributes, uint8_t attributesMax)
{
  int i = 0;
  unsigned c = offset;

  lasterror = 0;

  while (i < attributesMax && c < packet->payloadLength)
  {
    attributes[i].type = * (packet->payload + c);
    attributes[i].length = * (packet->payload + c + 1);
    attributes[i].value = (void *)(packet->payload + c + 2);
    c += attributes[i].length + 2;
    i++;
  }

  return (c == packet->payloadLength ? i : -1);
}

int UBloxInterpreter::encodeAttributes(Packet *packet, unsigned offset, Attribute* attributes, uint8_t attributesCount)
{
  int i;
  unsigned length = 0;
  char *start = packet->payload + offset;

  lasterror = 0;

  for (i = 0; i < attributesCount; i++)
  {
    if (offset + length + attributes->length + 2 > UBLOX_MAXPAYLOAD) return 0;
    * (start++) = attributes[i].type;
    * (start++) = attributes[i].length;
    memcpy(start, attributes[i].value, attributes[i].length);
    start += attributes[i].length;
    length += attributes[i].length + 2;
  }

  return length;
}

void UBloxInterpreter::calculate_checksum(Packet *packet)
{
  char checksumA = 0, checksumB = 0;
  int i;

  lasterror = 0;

  for (i = 2; i < packet->payloadLength + 6; i++)
  {
    checksumA = checksumA + * ((char *) packet + i);
    checksumB += checksumA;
  }

  // packet->checksumA = checksumA;
  // packet->checksumB = checksumB;
  * (packet->payload + packet->payloadLength + 0) = checksumA;
  * (packet->payload + packet->payloadLength + 1) = checksumB;
}

int UBloxInterpreter::sendPacket(Packet *packet, callback_function func)
{
  lasterror = 0;

  packet->sync1 = UBLOX_SYNC[0];
  packet->sync2 = UBLOX_SYNC[1];
  calculate_checksum(packet);
  return func((char *) packet, packet->payloadLength + UBLOX_FRAMESIZE);
}

int UBloxInterpreter::send(void *message, unsigned length, uint8_t classId, uint8_t messageId, callback_function func)
{
  Packet packet;

  lasterror = 0;
  if (length > UBLOX_MAXPAYLOAD) return error(UBLOX_ERROR_BIGPACKET);

  packet.classId = classId;
  packet.messageId = messageId;
  packet.payloadLength = length;
  memcpy(packet.payload, message, length);
  return sendPacket(&packet, func);
}

#ifdef _QUEUE_H
int UBloxInterpreter::sendToQueue(void *message, unsigned length, uint8_t classId, uint8_t messageId, Queue *q)
{
  Packet packet;

  lasterror = 0;
  if (length > UBLOX_MAXPAYLOAD) return error(UBLOX_ERROR_BIGPACKET);

  packet.sync1 = UBLOX_SYNC[0];
  packet.sync2 = UBLOX_SYNC[1];
  packet.classId = classId;
  packet.messageId = messageId;
  packet.payloadLength = length;
  memcpy(packet.payload, message, length);
  calculate_checksum(&packet);

  return qwrite(q, &packet, length + UBLOX_FRAMESIZE);
}
#endif // _QUEUE_H

int UBloxInterpreter::encode(const void *message, const unsigned length, const uint8_t classId, const uint8_t messageId, char *buffer, const unsigned size)
{
  Packet *packet = (Packet *) buffer;

  lasterror = 0;
  if (length > UBLOX_MAXPAYLOAD || size < length + 8) return error(UBLOX_ERROR_BIGPACKET);

  if (message != NULL) memcpy(packet->payload, message, length);
  packet->sync1 = UBLOX_SYNC[0];
  packet->sync2 = UBLOX_SYNC[1];
  packet->classId = classId;
  packet->messageId = messageId;
  packet->payloadLength = length;

  calculate_checksum(packet);
  return length + UBLOX_FRAMESIZE;
}
