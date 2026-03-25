// C combatible transcoder for server and client side

// Constants:

#define OPENER 0xAA
// #define 

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




*/