// Meow.
// This is a birthday gift.

#include <stdio.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "rc522_register_map.h"
#include "main_definitions.h"

#define IRQ_READER_1 GPIO_NUM_35
#define CHIPSELECT_READER_1 GPIO_NUM_26
#define RESET_PIN_READER_1 GPIO_NUM_17
#define SCLK_PIN_READER_1 GPIO_NUM_18
#define MOSI_PIN_READER_1 GPIO_NUM_23
#define MISO_PIN_READER_1 GPIO_NUM_19

// #define USER_LED_PIN GPIO_NUM_2 // LED D2 Not working

typedef struct {
    spi_device_handle_t spi;
    gpio_num_t rst_gpio;
} rc522_t;

static esp_err_t rc522_spi_init(rc522_t *dev,
                               spi_host_device_t host,
                               gpio_num_t sclk, gpio_num_t mosi, gpio_num_t miso,
                               gpio_num_t cs, gpio_num_t rst);

void configure_gpios(void)
{
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << CHIPSELECT_READER_1),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&conf);

    // conf.pin_bit_mask = (1ULL << USER_LED_PIN);
    // gpio_config(&conf); //? Not working
}

rc522_t reader_1 = {
    .rst_gpio = RESET_PIN_READER_1,
};


static const char *TAG = "rc522";



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
    uint8_t irq_status = 0x00;
        printf(". 0x%X", irq_status);

    while(irq_status != RC522_IRQ_RXCPLT_MASK)
    {
        irq_status = 0;
        rc522_read_reg(dev, RC522_REG_COM_IRQ, &irq_status);
        printf(". 0x%X", irq_status);
        irq_status &= RC522_IRQ_RXCPLT_MASK;
        vTaskDelay(portTICK_PERIOD_MS / 50);
    }
    printf("\n ...rx complete \n");

    return ESP_OK;
}


static esp_err_t rc522_request_anticollision(rc522_t *dev)
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
        printf(" fifo flushed \n");

    // Load FIFO with card-specific command 
    err = rc522_write_reg(dev, RC522_REG_FIFO_DATA, PICC_CMD_REQA);
        printf(" fifo loaded  \n");

    err = rc522_write_reg(dev, RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);
        printf(" fifo transeive start  \n");

    uint8_t rx_byte = 0;
    uint8_t fifo_level = 1; // We loaded one fifo byte.

    // Read
    rc522_block_until_rx_cplt(dev);
    err = rc522_read_reg(dev, RC522_REG_FIFO_DATA, &rx_byte);
    err = rc522_read_reg(dev, RC522_REG_FIFO_LEVEL, &fifo_level);


    if(fifo_level > 2)
    {
        printf("More than one byte received! %i \n", fifo_level);
    } else 
    {
        printf("one byte recieved! 0x%X \n", rx_byte);
    }

    if(err != ESP_OK)
    {
        return err;
    } 

    return err;
 
}


void app_main(void)
{
    printf("Program start \n");
    // configure_gpios();

    rc522_spi_init(&reader_1, SPI2_HOST, SCLK_PIN_READER_1, MOSI_PIN_READER_1, MISO_PIN_READER_1, CHIPSELECT_READER_1, RESET_PIN_READER_1);

    // Check version. This also helps to make sure communication is OK
    uint8_t version = 0xFF;
    uint16_t err = rc522_read_reg(&reader_1, RC522_REG_VERSION, &version);

    if (err != ESP_OK) {
        printf("Version read failed: %s\n", esp_err_to_name(err));
        return;
    }
    
    

    // Infinite loop
    for (;;)
    {
        int level = gpio_get_level(IRQ_READER_1);
        if(level == 1)
        {
            printf("IRQ HIGH \n");
        } else if (level == 0) 
        {
            printf("IRQ LOW \n");
        }

        // printf("%X \n", version);

        rc522_request_anticollision(&reader_1);
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}










/*
static esp_err_t rc522_request_a(rc522_t *dev, uint8_t *atqa, uint8_t *atqa_len)
{
    uint8_t irq = 0;
    uint8_t err = 0;
    uint8_t fifo_len = 0;

    *atqa_len = 0;

    // Stop any active command
    ESP_ERROR_CHECK(rc522_write_reg_u8(dev, RC522_REG_COMMAND, RC522_CMD_IDLE));

    // Clear IRQ flags
    ESP_ERROR_CHECK(rc522_write_reg_u8(dev, RC522_REG_COM_IRQ, 0x7F));

    // Flush FIFO
    ESP_ERROR_CHECK(rc522_flush_fifo(dev));

    // REQA is 7 bits
    ESP_ERROR_CHECK(rc522_write_reg_u8(dev, RC522_REG_BIT_FRAMING, 0x07));

    // Write request byte to FIFO
    ESP_ERROR_CHECK(rc522_write_reg_u8(dev, RC522_REG_FIFO_DATA, PICC_CMD_REQA));

    // Start transceive
    ESP_ERROR_CHECK(rc522_write_reg_u8(dev, RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE));

    // StartSend bit = bit 7
    ESP_ERROR_CHECK(rc522_set_bits(dev, RC522_REG_BIT_FRAMING, 0x80));

    // Poll for RX / timeout / error completion
    for (int i = 0; i < 50; ++i) {
        ESP_ERROR_CHECK(rc522_read_reg_u8(dev, RC522_REG_COM_IRQ, &irq));

        if (irq & 0x20) { // RxIRq
            break;
        }
        if (irq & 0x01) { // TimerIRq
            return ESP_ERR_TIMEOUT;
        }
        if (irq & 0x02) { // ErrIRq
            break;
        }

        ets_delay_us(200);
    }

    ESP_ERROR_CHECK(rc522_read_reg_u8(dev, RC522_REG_ERROR, &err));
    if (err != 0x00) {
        printf("RC522 ErrorReg = 0x%02X\n", err);
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(rc522_read_reg_u8(dev, RC522_REG_FIFO_LEVEL, &fifo_len));
    if (fifo_len < 2) {
        printf("Unexpected FIFO len: %u\n", fifo_len);
        return ESP_FAIL;
    }

    if (fifo_len > 2) fifo_len = 2;

    for (uint8_t i = 0; i < fifo_len; ++i) {
        ESP_ERROR_CHECK(rc522_read_reg_u8(dev, RC522_REG_FIFO_DATA, &atqa[i]));
    }

    *atqa_len = fifo_len;

    // Clear StartSend and return to byte framing default
    ESP_ERROR_CHECK(rc522_clear_bits(dev, RC522_REG_BIT_FRAMING, 0x80));
    ESP_ERROR_CHECK(rc522_write_reg_u8(dev, RC522_REG_BIT_FRAMING, 0x00));
    ESP_ERROR_CHECK(rc522_write_reg_u8(dev, RC522_REG_COMMAND, RC522_CMD_IDLE));

    return ESP_OK;
}


*/