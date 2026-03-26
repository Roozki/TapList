// C combatible transcoder for server and client side

#include <stdint.h>
#include <string.h>

#define DEBUG

// Constants:
#define MAX_POST_REQUEST_BODY_SIZE 64
#define MAX_ARGS 5
#define OPENER_MSG_FROM_SERVER 0xAA
#define OPENER_MSG_FROM_CLIENT 0xAC
#define CLOSER 0xBB

#define SERVER_DEVICE_ID 0xBEEF

#define MSG_CARD_TAPPED_LENGTH_BYTES 4u // 4 byte uuid? 

typedef enum {
    PING, // Debug / test purposes.
    CARD_TAPPED_ON_ADD_PAD,
    CARD_TAPPED_ON_REMOVE_PAD,
} MsgId; // Message Id


typedef enum {
    OK,
    NOT_OK,
} TranscoderRet;


// typedef encoder_buffer = char data[MAX_POST_REQUEST_BODY_SIZE];

// Message Structure
/* START OF DATA
Header: (Always 6 bytes)
    Byte 0: Generic Opener for client or server. (not needed?)
    Byte 1 - 5: Device ID that is sending the command (designates user - can i use hardcoded stuff?)

Content: (Variable Length, byte 0 here corresponds to the next byte after header)
    Byte 0: Message ID
    Byte 1: Length ////(0xFF means default length)
    Byte 2 - ( 1 + length): Data
End:
    Byte 0: Generic Closer byte



*/
#ifdef DEBUG
static void printData(const char *data[MAX_POST_REQUEST_BODY_SIZE], size_t len)
{
        printf("Data Encoded: [");
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", *data[i]);
    }
    printf("]\n");
}
#endif

static TranscoderRet encode(uint32_t DeviceId, MsgId msgId, const char args[MAX_ARGS], char* data[MAX_POST_REQUEST_BODY_SIZE])
{
    uint32_t startpos = (uint32_t)data;

    uint32_t pos = startpos;
    uint8_t opener;

    if(DeviceId == SERVER_DEVICE_ID)
    {
        opener = OPENER_MSG_FROM_SERVER;
    } else
    {
        opener = OPENER_MSG_FROM_CLIENT;
    }

    memcpy(data + pos, &opener, sizeof(opener));
    pos += sizeof(opener);

    memcpy(data + pos, &DeviceId, sizeof(DeviceId));
    pos += sizeof(DeviceId);



    switch (msgId)
    {
    case PING:
            
        break;
    
    default:
        break;
    }

    #ifdef DEBUG
    printData((const char**)data, pos - startpos);
    #endif

    return OK;
}



