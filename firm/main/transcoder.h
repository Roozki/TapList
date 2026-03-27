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


// #define ENCODE(x)

typedef enum {
    MSGID_PING, // Debug / test purposes.
    MSGID_CARD_TAP_EVENT,
} MsgId; // Message Id


typedef enum {
    OK,
    NOT_OK,
} TranscoderRet;


typedef struct {
    uint32_t nuid;
    uint8_t tap; // Info for tapping card. LSB is reserved for if it was tapped on add pad or remove pad
} CardTapPayload;

typedef struct {
    uint32_t reserved; // Idk
}  PingPayload;

typedef union {
    PingPayload ping;
    CardTapPayload card_tap;
} Payload;

typedef struct {
    uint32_t device_id;
    MsgId id;
    Payload payload;
} Msg;

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
static void printEncodedData(const char *data, size_t len)
{
        printf("Data Encoded: [");
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("]\n");
}

// Print a message
static void printDecodedData(Msg* msg)
{
    printf("Data Decoded:  \n   \
            Device ID: %04X  \n  \
            Message ID: %04X  \n  \
            ", (size_t)msg->device_id, msg->id);

}
#endif

static TranscoderRet encode(Msg* msg, char* data)
{
    uint32_t pos = 0;
    uint8_t opener;

    if(msg->device_id == SERVER_DEVICE_ID)
    {
        opener = OPENER_MSG_FROM_SERVER;
    } else
    {
        opener = OPENER_MSG_FROM_CLIENT;
    }

    memcpy(data + pos, &opener, sizeof(opener));
    pos += sizeof(opener);

    memcpy(data + pos, &msg->device_id, sizeof(msg->device_id));
    pos += sizeof(msg->device_id);

    switch (msg->id)
    {
    case MSGID_PING:
        memcpy(data + pos, &msg->payload.ping, sizeof(msg->payload.ping.reserved));
        pos += sizeof(msg->payload.ping.reserved);
        break;
    case MSGID_CARD_TAP_EVENT:
        memcpy(data + pos, &msg->payload.ping, sizeof(msg->payload.card_tap.nuid));
        pos += sizeof(msg->payload.card_tap.nuid);

        memcpy(data + pos, &msg->payload.card_tap.tap, sizeof(msg->payload.card_tap.tap));
        pos += sizeof(msg->payload.card_tap.tap);
        break;
    default:
        break;
    }
    uint32_t len = pos;

    #ifdef DEBUG
    printEncodedData((const char*)data, len);
    #endif

    return OK;
}

static TranscoderRet decode(Msg* msg, const char* data, size_t len)
{
    uint32_t pos = 0;

    uint8_t opener;
    memcpy(&opener, data + pos, sizeof(opener));
    pos += sizeof(opener);
    // Or skip opener: pos++;

    memcpy(&msg->device_id, data + pos, sizeof(msg->device_id));
    pos += sizeof(msg->device_id);

    memcpy(&msg->id, data + pos, sizeof(msg->id));
    pos += sizeof(msg->id);

    

    // memcpy(data + pos, &opener, sizeof(opener));
    // pos += sizeof(opener);

    // memcpy(data + pos, &DeviceId, sizeof(DeviceId));
    // pos += sizeof(DeviceId);



    // switch (msgId)
    // {
    // case PING:
            
    //     break;
    
    // default:
    //     break;
    // }
    len = pos;
    if(len > 44)
    {
        return  NOT_OK;
    }

    #ifdef DEBUG
    
    #endif

    return OK;
}






