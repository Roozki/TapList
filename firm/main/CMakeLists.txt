idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash driver esp_http_client esp_wifi nvs_flash mbedtls
                       INCLUDE_DIRS "")
