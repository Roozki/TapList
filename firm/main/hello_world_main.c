/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>

#define IRQ_READER_1 GPIO_NUM_35
#define CHIPSELECT_READER_1 GPIO_NUM_26
#define RESET_PIN_READER_1 GPIO_NUM_17
#define SCLK_PIN_READER_1 GPIO_NUM_18
#define MOSI_PIN_READER_1 GPIO_NUM_23
#define MISO_PIN_READER_1 GPIO_NUM_19

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
}

rc522_t reader_1 = {
    .rst_gpio = RESET_PIN_READER_1,
};


void app_main(void)
{
    printf("Program start \n");
    // configure_gpios();

    rc522_spi_init(&reader_1, SPI2_HOST, SCLK_PIN_READER_1, MOSI_PIN_READER_1, MISO_PIN_READER_1, CHIPSELECT_READER_1, RESET_PIN_READER_1);




}

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


