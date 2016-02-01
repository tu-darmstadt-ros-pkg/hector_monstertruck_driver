/*! @file ublox.h
 *  @brief uBlox decoding and encoding
 *
 *  functions used for decoding and encoding uBlox packets with respect to synchronization, checksums etc.
 */

#ifndef _UBLOX_H
#define _UBLOX_H

#include <stdint.h>

// individual structure alignment (linux- gcc)
#ifdef __GNUC__
 #ifndef GCC3X_PACK8
  #define GCC3X_PACK8 __attribute__((__packed__))
 #endif
#else
 #ifndef GCC3X_PACK8
  // individual structure alignment (windows visual studio 2005)
  #define GCC3X_PACK8
 #endif
 #pragma pack(push,1)
#endif

// General Constants
#define UBLOX_MAXLENGTH 1024                //!< maximum length of uBlox packet
#define UBLOX_FRAMESIZE 8
#define UBLOX_MAXPAYLOAD (UBLOX_MAXLENGTH - UBLOX_FRAMESIZE)

static const char UBLOX_DEFAULT_SYNC[] = { 0x37, 0x42 };          //!< synchronization characters at the start of a uBlox frame

// Type Definitions
typedef enum UBloxError { // error constants
  UBLOX_ERROR_CHECKSUM = 1,             //!< wrong checksum
  UBLOX_ERROR_BIGPACKET = 2,            //!< packet too big (see UBLOX_MAXPAYLOADLENGTH)
  UBLOX_ERROR_NOUBLOX = 3,              //!< frame invalid
} UBloxError;

typedef struct GCC3X_PACK8 UBloxAttribute {
  uint8_t type;                                   //!< Type of attribute
  uint8_t length;                                 //!< Length of attribute value
  void *value;                                          //!< Pointer to attribute value
} UBloxAttribute;

typedef struct GCC3X_PACK8 UBloxPacket {
  char sync1;                                  //!< Synchronization character
  char sync2;                                  //!< Synchronization character
  uint8_t  classId;                               //!< Class identifier of the packet
  uint8_t  messageId;                             //!< Message identifier of the packet
  uint16_t payloadLength;                         //!< Length of the payload (fixed + variable part)
  char  payload[UBLOX_MAXPAYLOAD + 2];         //!< Payload (2 additional bytes for the checksum)
  char checksumA;                              //!< Checksum A
  char checksumB;                              //!< Checksum B
} UBloxPacket;

typedef struct GCC3X_PACK8 UBloxHeader {
  uint8_t  classId;                               //!< Class identifier of the packet
  uint8_t  messageId;                             //!< Message identifier of the packet
  uint16_t payloadLength;                         //!< Length of the payload (fixed + variable part)
} UBloxHeader;

typedef int (*ublox_callback_function)(char *, unsigned);

#ifdef __cplusplus
#include "RingBuffer.h"

class UBloxInterpreter {
  public:
    typedef enum UBloxError                Error;
    typedef struct UBloxAttribute          Attribute;
    typedef struct UBloxPacket             Packet;
    typedef struct UBloxHeader             Header;
    typedef ublox_callback_function callback_function;

    UBloxInterpreter() {
      UBLOX_SYNC[0] = UBLOX_DEFAULT_SYNC[0];
      UBLOX_SYNC[1] = UBLOX_DEFAULT_SYNC[1];
      lasterror = 0;
    }

    #ifdef _QUEUE_H
    int find(Queue *q, Header *header);
    /*!< find the first uBlox packet in a queue. The preceding bytes will be removed from the queue.
        @param q      pointer to Queue object
        @param ublox if not NULL, pointer to a Header struct, where header information will be stored
        @retval 0    no (full) packet found in queue
        @retval >0   length of the packet, packet is at top position in the queue now
    */

    int decode(Queue *q, void *payload, unsigned *payloadLength, Header *header);
    /*!< decode the first packet in a queue. The packet will be removed from the queue.
        @param q      pointer to Queue object
        @param payload pointer to a buffer, where payload will be copied to
        @param size   size of the payload buffer
        @param ublox if not NULL, pointer to a Header struct, where header information will be stored
        @retval 0    no (full) packet found in queue
        @retval <0   decoding error (see Error Constants)
        @retval >0   length of the packet's payload
    */
    #endif // _QUEUE_H
    
    #if defined(RINGBUFFER_H_)
    int find(RingBuffer<char> &buffer, Header *header);
    int decode(RingBuffer<char> &buffer, void *payload, unsigned *payloadLength, Header *header);
    #endif // RINGBUFFER_H_
    
    int findNext(char **pos, unsigned *size);
    int find(char **pos, unsigned *size, Header *header);
    /*!< find the first uBlox packet in a queue. The preceding bytes will be removed from the queue.
        @param buffer  pointer to incoming uBlox buffer
        @param size    size of incoming buffer
        @param ublox if not NULL, pointer to a Header struct, where header information will be stored
        @retval 0    no (full) packet found
        @retval >0   length of the packet, packet is at top position in the queue now
    */
    
    int decodePacket(char *buffer, unsigned size, Packet **packet);
    int decode(char *buffer, unsigned size, void **payload, unsigned *payloadLength, Header *header);
    /*!< decode the first packet in a queue. The packet will be removed from the queue.
        @param buffer  pointer to incoming uBlox buffer
        @param size    size of incoming buffer
        @param payload pointer to a pointer to a buffer, will point to the payload in the buffer
        @param ublox if not NULL, pointer to a Header struct, where header information will be stored
        @retval 0    no (full) packet found in queue
        @retval <0   decoding error (see Error Constants)
        @retval >0   length of the packet's payload
    */
    
    int remove(char *buffer, unsigned *size, char *pos);
    
    int decodeAttributes(Packet *packet, unsigned offset, Attribute* attributes, uint8_t attributesMax);
    /*!< fill an array of Attribute structures out of a payload
        @param packet pointer to a uBlox packet structure
        @param offset size of the fixed part of the payload
        @param attributes array of Attribute structures
        @param attributesMax size of the array
        @retval <0   decoding error (see Error Constants)
        @retval >=0  number of attributes
    */
    
    int encodeAttributes(Packet *packet, unsigned offset, Attribute* attributes, uint8_t attributesCount);
    /*!< serialize an array of Attribute structures
        @param packet pointer to a uBlox packet structure
        @param offset size of the fixed part of the payload
        @param attributes array of Attribute structures
        @param attributesCount number of attributes to be encoded
        @retval <0  encoding error (see Error Constants)
        @retval >=0  length of the attribute part of the payload (<= lengthMax)
    */
    
    int sendPacket(Packet *packet, callback_function func);
    /*!< compose and send a uBlox packet
        @param packet pointer to the uBlox packet to be send
        @param func   function used for sending
        @retval <0   an error occured (see Error Constants)
        @retval >=0  return value of *func
    */
    
    int send(void *message, unsigned length, uint8_t classId, uint8_t messageId, callback_function func);
    /*!< compose and send a uBlox packet using a predefined handler for the underlying layer (see UBLOX_SEND_FUNCTION)
        @param message pointer to the uBlox message to be send (the payload)
        @param length length of the message (= length of the payload)
        @param classId Class ID of the message
        @param messageId Message ID of the message
        @param func   function used for sending
        @retval <0   an error occured (see Error Constants)
        @retval >=0  return value of *func
    */
    
    #ifdef _QUEUE_H
    int sendToQueue(void *message, unsigned length, uint8_t classId, uint8_t messageId, Queue *q);
    /*!< compose and send a uBlox packet using a predefined handler for the underlying layer (see UBLOX_SEND_FUNCTION)
        @param message pointer to the uBlox message to be send (the payload)
        @param length length of the message (= length of the payload)
        @param classId Class ID of the message
        @param messageId Message ID of the message
        @param q      pointer to Queue object
        @retval <0   an error occured (see Error Constants)
        @retval >=0  return value of qwrite
    */
    #endif // _QUEUE_H
    
    int encode(const void *message, const unsigned length, const uint8_t classId, const uint8_t messageId, char *buffer, const unsigned size);
    /*!< compose a uBlox packet using a predefined handler for the underlying layer (see UBLOX_SEND_FUNCTION)
        @param message pointer to the uBlox message to be send (the payload)
        @param length length of the message (= length of the payload)
        @param classId Class ID of the message
        @param messageId Message ID of the message
        @param buffer pointer to the buffer for the encoded packet (can be equal message pointer)
        @param size   size of the output buffer in bytes
        @retval <0   an error occured (see Error Constants)
        @retval >=0  return value of *func
        @see send
    */
    
    void setSync(const char sync[]) { UBLOX_SYNC[0] = sync[0]; UBLOX_SYNC[1] = sync[1]; }
    const char *getSync() { return UBLOX_SYNC; }

    const char *strlasterror ( void )
    {
      return strerror(lasterror );
    }
    
    const char *strerror(int err)
    {
      const char *errstr[] = { "no error",
                               "wrong checksum", // [UBLOX_ERROR_CHECKSUM]
                               "illegal packet size", // [UBLOX_ERROR_BIGPACKET]
                               "invalid uBlox frame", // [UBLOX_ERROR_NOUBLOX]
                             };
      if ( err < 0 || (size_t) err >= sizeof ( errstr ) /sizeof ( *errstr ) ) return "unkown error";
      return errstr[err];
    }
    
  private:
    char UBLOX_SYNC[2];          //!< snchronization characters at the start of a uBlox frame
    int  lasterror;

    void calculate_checksum(Packet *packet);

    int error(int c)
    {
      lasterror = c;
      return -c;
    }
};

extern UBloxInterpreter uBlox;

#else // __cplusplus

#endif // __cplusplus

#ifndef __GNUC__
  #pragma pack(pop)
#endif

#endif // _UBLOX_H
