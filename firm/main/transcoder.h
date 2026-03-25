// C combatible transcoder for server and client side

// Constants:

#define OPENER_MSG_FROM_SERVER 0xAA
#define OPENER_MSG_FROM_CLIENT 0xAC
#define CLOSER 0xBB


#define MSG_CARD_TAPPED_LENGTH_BYTES 4u // 4 byte uuid? 



typedef enum {
    CARD_TAPPED_ON_ADD_PAD,
    CARD_TAPPED_ON_REMOVE_PAD,
} MsgId; // Message Id


typedef enum {
    OK,
    NOT_OK,
} TranscoderRet;


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

{
typedef TranscoderRet ret;

static ret encode(MsgId, const char* data)
{

}



}