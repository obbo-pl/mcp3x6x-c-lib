/*
 * spi_bus_hal.c
 *
 * Created on: 7 kwi 2023
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2023 Krzysztof Markiewicz
 */

#include "spi_bus_hal.h"
#include <stdlib.h>
#include <string.h>

#if defined(_SPI_BUS_HAL_ARDUINO_)
#include <Arduino.h>
#elif defined(_SPI_BUS_HAL_ESP_)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
spi_device_handle_t spi;

#elif defined(_SPI_BUS_HAL_PICO_)
#include "hardware/spi.h"
#endif


int spi_bus_hal_Init()
{
    int result = SPI_BUS_HAL_OK;
#if defined(_SPI_BUS_HAL_ARDUINO_)
    pinMode(SPI_BUS_HAL_MOSI, OUTPUT);
    pinMode(SPI_BUS_HAL_MISO, INPUT);
    pinMode(SPI_BUS_HAL_CLK, OUTPUT);
    pinMode(SPI_BUS_HAL_CS, OUTPUT);
    digitalWrite(SPI_BUS_HAL_CS, HIGH);
#elif defined(_SPI_BUS_HAL_ESP_)
    gpio_config_t io_conf;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE ;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;
    io_conf.intr_type = GPIO_INTR_DISABLE ;
    io_conf.mode = GPIO_MODE_OUTPUT ;
    io_conf.pin_bit_mask = (1ULL << SPI_BUS_HAL_CS);
    result = gpio_config(&io_conf);
    result = gpio_set_level(SPI_BUS_HAL_CS, 1);
    if (result != SPI_BUS_HAL_OK) goto finish;
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_BUS_HAL_MISO,
        .mosi_io_num = SPI_BUS_HAL_MOSI,
        .sclk_io_num = SPI_BUS_HAL_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .flags = 0
    };
    result = spi_bus_initialize(SPI_BUS_HAL, &buscfg, SPI_BUS_HAL_DMA_CHAN);
    if (result == SPI_BUS_HAL_OK) {
        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = SPI_BUS_HAL_FREQ_HZ,
            .mode = 0,
            .spics_io_num = -1,
            .queue_size = 1,
            .flags = 0
        };
#ifdef SPI_BUS_HAL_MODE_HALFDUPLEX
        devcfg.flags |= SPI_DEVICE_HALFDUPLEX;
#endif
        result = spi_bus_add_device(SPI_BUS_HAL, &devcfg, &spi);
    }
#elif defined(_SPI_BUS_HAL_PICO_)
    spi_init(spi_default, SPI_BUS_HAL_FREQ_HZ);
    gpio_set_function(SPI_BUS_HAL_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_BUS_HAL_CLK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_BUS_HAL_MOSI, GPIO_FUNC_SPI);
    gpio_init(SPI_BUS_HAL_CS);
    gpio_set_dir(SPI_BUS_HAL_CS, GPIO_OUT);
    gpio_put(SPI_BUS_HAL_CS, 1);
#endif
finish:
    return result;
}

int spi_bus_hal_SendCmd(uint8_t* cmd_wr, uint8_t* cmd_rd, size_t cmd_size)
{
    if (cmd_size == 0) return SPI_BUS_HAL_OK;
    int result = SPI_BUS_HAL_OK;
    if (cmd_wr != NULL) {
#if defined(_SPI_BUS_HAL_ARDUINO_)
        digitalWrite(SPI_BUS_HAL_MOSI, LOW);
        uint8_t d = *cmd_wr;
        for (uint8_t i = 0; i < 8; i++) {
            if (d & 0x80)
                digitalWrite(SPI_BUS_HAL_MOSI, HIGH);
            else
                digitalWrite(SPI_BUS_HAL_MOSI, LOW);
            delayMicroseconds(2);
            digitalWrite(SPI_BUS_HAL_CLK, HIGH);
            d = d << 1;
            delayMicroseconds(2);
            digitalWrite(SPI_BUS_HAL_CLK, LOW);
        }
        delayMicroseconds(2);
        digitalWrite(SPI_BUS_HAL_MOSI, LOW);
        delayMicroseconds(5);
#elif defined(_SPI_BUS_HAL_ESP_)
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = cmd_size * 8;
        t.tx_buffer = cmd_wr;
#ifndef SPI_BUS_HAL_MODE_HALFDUPLEX
        if (cmd_rd == NULL) t.rxlength = 0;
        t.rx_buffer = cmd_rd;
#endif
        result = spi_device_polling_transmit(spi, &t);
#elif defined(_SPI_BUS_HAL_PICO_)
        int ret = spi_write_blocking(SPI_BUS_HAL, cmd_wr, cmd_size);
        if (ret != cmd_size) {
            result = SPI_BUS_HAL_ERR_GENERIC;
        }
        sleep_us(15);
#endif
    } else {
        result = SPI_BUS_HAL_ERR_INVALID_ARG;
    }
    return result;
}

int spi_bus_hal_ReadReg(uint8_t* cmd_wr, uint8_t* cmd_rd, size_t cmd_size, uint8_t* data_wr, uint8_t* data_rd, size_t data_size)
{
#if defined(_SPI_BUS_HAL_ARDUINO_)
    int result = SPI_BUS_HAL_OK;
    digitalWrite(SPI_BUS_HAL_CS, LOW);
    delayMicroseconds(5);
#elif defined(_SPI_BUS_HAL_ESP_)
    int result = gpio_set_level(SPI_BUS_HAL_CS, 0);
#elif defined(_SPI_BUS_HAL_PICO_)
    int result = SPI_BUS_HAL_OK;
    gpio_put(SPI_BUS_HAL_CS, 0);
    sleep_us(5);
#endif
    if (result == SPI_BUS_HAL_OK) {
        result = spi_bus_hal_SendCmd(cmd_wr, cmd_rd, cmd_size);
        if (result == SPI_BUS_HAL_OK) {
            if (data_size == 0) goto finish;
            if (data_rd != NULL) {
#if defined(_SPI_BUS_HAL_ARDUINO_)
                while (data_size--) {
                    uint8_t d = 0;
                    for (uint8_t i = 0; i < 8; i++) {
                        d = d << 1;
                        if (digitalRead(SPI_BUS_HAL_MISO)) d |= 0x01;
                        digitalWrite(SPI_BUS_HAL_CLK, HIGH);
                        delayMicroseconds(2);
                        digitalWrite(SPI_BUS_HAL_CLK, LOW);
                        delayMicroseconds(2);
                    }
                    *data_rd = d;
                    data_rd++;
                }
#elif defined(_SPI_BUS_HAL_ESP_)
                spi_transaction_t t;
                memset(&t, 0, sizeof(t));
                t.rx_buffer = data_rd;
                t.rxlength = data_size * 8;
#ifndef SPI_BUS_HAL_MODE_HALFDUPLEX
                t.length = data_size * 8;
                t.tx_buffer = data_wr;
#endif
                result = spi_device_polling_transmit(spi, &t);
#elif defined(_SPI_BUS_HAL_PICO_)
                int ret = spi_read_blocking(SPI_BUS_HAL, 0, data_rd, data_size);
                if (ret != data_size) {
                    result = SPI_BUS_HAL_ERR_GENERIC;
                }
#endif
            } else {
                result = SPI_BUS_HAL_ERR_INVALID_ARG;
            }
        }
    }
finish:
#if defined(_SPI_BUS_HAL_ARDUINO_)
    delayMicroseconds(5);
    digitalWrite(SPI_BUS_HAL_CS, HIGH);
#elif defined(_SPI_BUS_HAL_ESP_)
    gpio_set_level(SPI_BUS_HAL_CS, 1);
#elif defined(_SPI_BUS_HAL_PICO_)
    sleep_us(15);
    gpio_put(SPI_BUS_HAL_CS, 1);
#endif
    return result;
}

int spi_bus_hal_WriteReg(uint8_t* cmd_wr, uint8_t* cmd_rd, size_t cmd_size, uint8_t* data_wr, uint8_t* data_rd, size_t data_size)
{
#if defined(_SPI_BUS_HAL_ARDUINO_)
    int result = SPI_BUS_HAL_OK;
    digitalWrite(SPI_BUS_HAL_CS, LOW);
    delayMicroseconds(5);
#elif defined(_SPI_BUS_HAL_ESP_)
    int result = gpio_set_level(SPI_BUS_HAL_CS, 0);
#elif defined(_SPI_BUS_HAL_PICO_)
    int result = SPI_BUS_HAL_OK;
    gpio_put(SPI_BUS_HAL_CS, 0);
    sleep_us(5);
#endif
    if (result == SPI_BUS_HAL_OK) {
        result = spi_bus_hal_SendCmd(cmd_wr, cmd_rd, cmd_size);
        if (result == SPI_BUS_HAL_OK) {
            if (data_size == 0) goto finish;
            if (data_wr != NULL) {
#if defined(_SPI_BUS_HAL_ARDUINO_)
                digitalWrite(SPI_BUS_HAL_MOSI, LOW);
                while (data_size--) {
                    uint8_t d = *(data_wr++);
                    for (uint8_t i = 0; i < 8; i++) {
                        if (d & 0x80)
                            digitalWrite(SPI_BUS_HAL_MOSI, HIGH);
                        else
                            digitalWrite(SPI_BUS_HAL_MOSI, LOW);
                        delayMicroseconds(2);
                        digitalWrite(SPI_BUS_HAL_CLK, HIGH);
                        d = d << 1;
                        delayMicroseconds(2);
                        digitalWrite(SPI_BUS_HAL_CLK, LOW);
                    }
                    delayMicroseconds(2);
                    digitalWrite(SPI_BUS_HAL_MOSI, LOW);
                }
#elif defined(_SPI_BUS_HAL_ESP_)
                spi_transaction_t t;
                memset(&t, 0, sizeof(t));
                t.length = data_size * 8;
                t.tx_buffer = data_wr;
#ifndef SPI_BUS_HAL_MODE_HALFDUPLEX
                if (data_rd == NULL) t.rxlength = 0;
                t.rx_buffer = data_rd;
#endif
                result = spi_device_polling_transmit(spi, &t);
#elif defined(_SPI_BUS_HAL_PICO_)
                int ret = spi_write_blocking(SPI_BUS_HAL, data_wr, data_size);
                if (ret != data_size) {
                    result = SPI_BUS_HAL_ERR_GENERIC;
                }
#endif
            } else {
                result = SPI_BUS_HAL_ERR_INVALID_ARG;
            }
        }
    }
finish:
#if defined(_SPI_BUS_HAL_ARDUINO_)
    delayMicroseconds(5);
    digitalWrite(SPI_BUS_HAL_CS, HIGH);
#elif defined(_SPI_BUS_HAL_ESP_)
    gpio_set_level(SPI_BUS_HAL_CS, 1);
#elif defined(_SPI_BUS_HAL_PICO_)
    sleep_us(15);
    gpio_put(SPI_BUS_HAL_CS, 1);
#endif
    return result;
}
