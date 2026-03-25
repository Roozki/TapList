#include <stdio.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "rc522_register_map.h"

#define MAX_CARDS 32
uint32_t known_cards[MAX_CARDS];
uint32_t num_known_cards = 0;

typedef enum {
    RESET, // Reset registers and stuff
    IDLE,
    ERROR,
    POWER_UP, // Turn on antenna
    POWER_DOWN,
    CHECK_CARD_TYPE,
    CHECK_CARD_NUID,
} ReaderState;

typedef struct {
    spi_device_handle_t spi;
    gpio_num_t rst_gpio;
    ReaderState state;
} rc522_t;

static esp_err_t rc522_spi_init(rc522_t *dev,
                               spi_host_device_t host,
                               gpio_num_t sclk, gpio_num_t mosi, gpio_num_t miso,
                               gpio_num_t cs, gpio_num_t rst)
{
    dev->rst_gpio = rst;

    // Optional reset pin
    if (rst >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << rst,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        gpio_set_level(rst, 1);
    }

    spi_bus_config_t buscfg = {
        .sclk_io_num = sclk,
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64, // plenty for reg + small bursts
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000, // start 2 MHz
        .mode = 0,                         // SPI mode 0 is typical
        .spics_io_num = cs,
        .queue_size = 2,
        .flags = 0,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg, &dev->spi));

    return ESP_OK;
}

// RC522 address byte helpers (common convention)
static inline uint8_t rc522_addr_write(uint8_t reg) { return (uint8_t)((reg << 1) & 0x7E); }
static inline uint8_t rc522_addr_read (uint8_t reg) { return (uint8_t)(((reg << 1) & 0x7E) | 0x80); }

static esp_err_t rc522_write_reg(rc522_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { rc522_addr_write(reg), value };

    spi_transaction_t t = {0};
    t.length = 8 * sizeof(tx);
    t.tx_buffer = tx;

    return spi_device_transmit(dev->spi, &t);
}

static esp_err_t rc522_read_reg(rc522_t *dev, uint8_t reg, uint8_t *out)
{
    uint8_t tx[2] = { rc522_addr_read(reg), 0x00 };
    uint8_t rx[2] = {0};

    spi_transaction_t t = {0};
    t.length = 8 * sizeof(tx);
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(dev->spi, &t);
    if (err == ESP_OK) *out = rx[1];
    return err;
}
// 8 bit send
static esp_err_t rc522_start_send_8bit(rc522_t *dev)
{
    rc522_write_reg(dev, RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);

    return rc522_write_reg(dev, RC522_REG_BIT_FRAMING, 0x80);
}


static esp_err_t rc522_clear_irq_flags(rc522_t *dev)
{
    return rc522_write_reg(dev, RC522_REG_COM_IRQ, 0x7F);
}



static esp_err_t rc522_check_version(rc522_t *dev, uint8_t *version)
{
    return rc522_read_reg(dev, RC522_REG_VERSION, version);
}

static esp_err_t rc522_flush_fifo(rc522_t *dev)
{
    return rc522_write_reg(dev, RC522_REG_FIFO_LEVEL, RC522_FLUSH_FIFO_VALUE);
}

static esp_err_t rc522_flush_command_reg(rc522_t *dev)
{
    return rc522_write_reg(dev, RC522_REG_COMMAND, RC522_CMD_IDLE);
}

static esp_err_t rc522_block_until_rx_cplt(rc522_t *dev)
{
    uint32_t start_ticks = xTaskGetTickCount();
    uint8_t irq_status = 0x00;
        printf(". 0x%X", irq_status);

    bool irq_triggered = false;
    while(irq_triggered == false)
    {
        irq_status = 0;
        rc522_read_reg(dev, RC522_REG_COM_IRQ, &irq_status);
        if(irq_status & RC522_COMIRQ_RX_MASK)
        {
            printf("RX Complete success \n");
            return ESP_OK;
        }

        if(irq_status & RC522_COMIRQ_ERR_MASK)
        {
            printf("RX Errored out \n");
            return ESP_FAIL; //TODO error handling
        }

        if(irq_status & RC522_COMIRQ_TIMER_MASK)
        {
            printf("RX timed out \n");
            return ESP_ERR_TIMEOUT; //TODO error handling
        }

        vTaskDelay(1);

        if(xTaskGetTickCount() - start_ticks > 10)
        {
            printf("receiver wait timemout (controller) \n");
            return ESP_ERR_TIMEOUT; //TODO error handling
        }
    }

    return ESP_OK;
}

static esp_err_t rc522_reqa(rc522_t *dev)
{
    esp_err_t err = ESP_OK;

    // Flush (clear) command reg
    err = rc522_flush_command_reg(dev);
    if(err != ESP_OK)
    {
        return err;
    } 
        printf("command flushed \n");

    // Flush FIFO
    err = rc522_flush_fifo(dev);
        // printf(" fifo flushed \n");

        // Set rf sending to 7 bit framing
        rc522_write_reg(dev, RC522_REG_BIT_FRAMING, 0x07);
        // Load FIFO with card-specific command
    err = rc522_write_reg(dev, RC522_REG_FIFO_DATA, PICC_CMD_REQA);
        // printf(" fifo loaded  \n");

    err = rc522_write_reg(dev, RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);
        // printf(" fifo transeive setup  \n");

        rc522_write_reg(dev, RC522_REG_BIT_FRAMING, 0x87); // 7 bits + StartSend
        // printf(" fifo transfer triggered  \n");


    uint8_t rx_byte_1 = 0;
    uint8_t rx_byte_2 = 0;
    uint8_t fifo_level = 1; // We loaded one fifo byte.
    // trigger send?
    // err = rc522_write_reg(dev, RC522_REG_BIT_FRAMING, )
    // Read
    rc522_block_until_rx_cplt(dev);
    err = rc522_read_reg(dev, RC522_REG_FIFO_DATA, &rx_byte_1);
    err = rc522_read_reg(dev, RC522_REG_FIFO_DATA, &rx_byte_2);
    err = rc522_read_reg(dev, RC522_REG_FIFO_LEVEL, &fifo_level);


    rc522_write_reg(dev, RC522_REG_COMMAND, RC522_CMD_IDLE);
    rc522_write_reg(dev, RC522_REG_BIT_FRAMING, 0x00);

    if(fifo_level > 0)
    {
        printf("More than two bytes received! %i \n", fifo_level);
    } else 
    {
        printf("two bytes recieved! 0x%X \n", rx_byte_1);
        printf("two bytes recieved! 0x%X \n", rx_byte_2);
        err = (rx_byte_1 == 0x4 && rx_byte_2 == 0x0) ? ESP_OK : ESP_FAIL;
    }

    return err;
}

static esp_err_t rc522_energized_reset(rc522_t *dev)
{
    rc522_flush_fifo(dev);
    rc522_clear_irq_flags(dev);
    return ESP_OK;
}

static esp_err_t rc522_request_anticollision_cl1(rc522_t *dev)
{
    rc522_energized_reset(dev);
    rc522_write_reg(dev, RC522_REG_FIFO_DATA, PICC_CMD_CL1);
    rc522_write_reg(dev, RC522_REG_FIFO_DATA, PICC_CMD_ANTICOLLISION);
    rc522_start_send_8bit(dev);
    if (rc522_block_until_rx_cplt(dev) == ESP_OK)
    {
        uint8_t bytes_received = 0;
        rc522_read_reg(dev, RC522_REG_FIFO_LEVEL, &bytes_received);
        printf("Anticollision Data Received. Size: %d", bytes_received);

        uint8_t nuid_buf[4];
        uint8_t xor_crc; // bcc? 
        uint8_t calculated_crc = 0;
        
        rc522_read_reg(dev, RC522_REG_FIFO_DATA, nuid_buf);
        rc522_read_reg(dev, RC522_REG_FIFO_DATA, nuid_buf +1);
        rc522_read_reg(dev, RC522_REG_FIFO_DATA, nuid_buf +2);
        rc522_read_reg(dev, RC522_REG_FIFO_DATA, nuid_buf +3);
        rc522_read_reg(dev, RC522_REG_FIFO_DATA, &xor_crc);
        uint32_t nuid = nuid_buf[0] + (nuid_buf[1] << 8) + (nuid_buf[2] << 16) + (nuid_buf[3] << 24);
        printf("uuid: %ld\n", nuid);
        printf("uuid_0: %X \n", nuid_buf[0]);
        printf("uuid_1: %X \n", nuid_buf[1]);
        printf("uuid_2: %X\n", nuid_buf[2]);
        printf("uuid_3: %X\n", nuid_buf[3]);

        printf("bcc (crc i think?): %X\n", xor_crc);
        calculated_crc = nuid_buf[0] ^ nuid_buf[1] ^ nuid_buf[2] ^ nuid_buf[3];
        printf("calculated bcc (crc i think?): %X\n", calculated_crc);

        if(calculated_crc != xor_crc)
        {
            printf("CRC mismatch. Abort \n");
            return ESP_FAIL;
        }


        for(int i = 0; i < MAX_CARDS; i++)
        {
            if(known_cards[i] == nuid)
            {
                printf("Known Card Found! nuid: %lu", nuid);
                // vTaskDelay(10);
            }
            if(known_cards[i] == 0)
            {
                // Free slot found in card buffer
                printf("New Card Found! Registering.. nuid: %lu", nuid);
                known_cards[i] = nuid;
                num_known_cards++;
            }
            context.last_known_card_uuid = nuid;
            return ESP_OK; // Break and return OK
        }
        printf("Internal Card Memory Full. Where is that server?!?! OR there may be a firmware issue");
                return ESP_OK; // Break and return OK

    }
    return ESP_FAIL;
}



static esp_err_t rc522_turn_on_antenna(rc522_t *dev)
{
    uint8_t tx;
    rc522_read_reg(dev, RC522_REG_TX_CONTROL, &tx);
    if (!(tx & 0x03)) {
        rc522_write_reg(dev, RC522_REG_TX_CONTROL, tx | 0x03);
    }
    return ESP_OK;
}

static esp_err_t rc522_turn_off_antenna(rc522_t *dev)
{
    uint8_t tx;
    rc522_read_reg(dev, RC522_REG_TX_CONTROL, &tx);
    // if (!(tx ^ 0x03)) {
        rc522_write_reg(dev, RC522_REG_TX_CONTROL, tx & ~0x03);
    // }
    return ESP_OK;
}