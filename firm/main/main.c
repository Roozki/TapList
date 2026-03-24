// Meow.
// This is a birthday gift.

#include <stdio.h>
#include <driver/gpio.h>
#include "main_definitions.h"
#include "rc522_reader.h"

#define IRQ_READER_1 GPIO_NUM_35
#define CHIPSELECT_READER_1 GPIO_NUM_26
#define RESET_PIN_READER_1 GPIO_NUM_17
#define SCLK_PIN_READER_1 GPIO_NUM_18
#define MOSI_PIN_READER_1 GPIO_NUM_23
#define MISO_PIN_READER_1 GPIO_NUM_19


// #define USER_LED_PIN GPIO_NUM_2 // LED D2 Not working

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

        rc522_turn_on_antenna(&reader_1);
        rc522_clear_irq_flags(&reader_1);
        if(rc522_reqa(&reader_1) == ESP_OK)
        {
            printf("Correct type of card identified. Moving forward. \n");
            rc522_request_anticollision_cl1(&reader_1);

        } else {
            printf("Incorrect type of card identified. Aborting read. \n");
        }
        printf("Num known cards: %lu \n", num_known_cards);


        rc522_turn_off_antenna(&reader_1);

        
        vTaskDelay(10 / portTICK_PERIOD_MS);
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