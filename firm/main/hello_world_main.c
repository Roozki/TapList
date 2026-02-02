/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <driver/gpio.h>

#define IRQ_READER_1 GPIO_NUM_35
#define CHIPSELECT_READER_1 GPIO_NUM_26

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

void app_main(void)
{
    printf("Program start \n");
    configure_gpios();




}


