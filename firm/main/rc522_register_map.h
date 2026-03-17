// MFRC522 register map
#define RC522_REG_COMMAND        0x01
#define RC522_REG_COM_I_EN       0x02
#define RC522_REG_DIV_I_EN       0x03
#define RC522_REG_COM_IRQ        0x04
#define RC522_REG_DIV_IRQ        0x05
#define RC522_REG_ERROR          0x06
#define RC522_REG_STATUS1        0x07
#define RC522_REG_STATUS2        0x08
#define RC522_REG_FIFO_DATA      0x09
#define RC522_REG_FIFO_LEVEL     0x0A
#define RC522_REG_CONTROL        0x0C
#define RC522_REG_BIT_FRAMING    0x0D
#define RC522_REG_COLL           0x0E

#define RC522_REG_MODE           0x11
#define RC522_REG_TX_MODE        0x12
#define RC522_REG_RX_MODE        0x13
#define RC522_REG_TX_CONTROL     0x14
#define RC522_REG_TX_ASK         0x15
#define RC522_REG_CRC_RESULT_H   0x21
#define RC522_REG_CRC_RESULT_L   0x22
#define RC522_REG_T_MODE         0x2A
#define RC522_REG_T_PRESCALER    0x2B
#define RC522_REG_T_RELOAD_H     0x2C
#define RC522_REG_T_RELOAD_L     0x2D
#define RC522_REG_VERSION        0x37

// Commands
// PCD commands
#define RC522_CMD_IDLE           0x00
#define RC522_CMD_MEM            0x01
#define RC522_CMD_CALC_CRC       0x03
#define RC522_CMD_TRANSMIT       0x04
#define RC522_CMD_NO_CMD_CHANGE  0x07
#define RC522_CMD_RECEIVE        0x08
#define RC522_CMD_TRANSCEIVE     0x0C
#define RC522_CMD_MF_AUTHENT     0x0E
#define RC522_CMD_SOFT_RESET     0x0F

// PICC commands
#define PICC_CMD_REQA            0x26
#define PICC_CMD_WUPA            0x52