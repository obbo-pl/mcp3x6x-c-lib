/*
 * spi_bus_hal.h
 *
 * Created on: 7 kwi 2023
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2023 Krzysztof Markiewicz
 */

#ifndef _SPI_BUS_HAL_H_
#define _SPI_BUS_HAL_H_


#include <stdio.h>
#include <stdint.h>


#if ARDUINO >= 100
#define _SPI_BUS_HAL_ARDUINO_
#define SPI_BUS_HAL_OK                  0
#define SPI_BUS_HAL_ERR_INVALID_ARG     1
#define SPI_BUS_HAL_MISO                15
#define SPI_BUS_HAL_MOSI                18
#define SPI_BUS_HAL_CLK                 19
#define SPI_BUS_HAL_CS                  23

#elif defined(ESP_PLATFORM)
#define _SPI_BUS_HAL_ESP_
#include <driver/gpio.h>
#include "driver/spi_master.h"
#define SPI_BUS_HAL_OK                  ESP_OK
#define SPI_BUS_HAL_ERR_INVALID_ARG     ESP_ERR_INVALID_ARG
#define SPI_BUS_HAL_MISO                GPIO_NUM_15       // CONFIG_SPI_BUS_HAL_MISO
#define SPI_BUS_HAL_MOSI                GPIO_NUM_18       // CONFIG_SPI_BUS_HAL_MOSI
#define SPI_BUS_HAL_CLK                 GPIO_NUM_19       // CONFIG_SPI_BUS_HAL_CLK
#define SPI_BUS_HAL_CS                  GPIO_NUM_23       // CONFIG_SPI_BUS_HAL_CS
#ifdef CONFIG_IDF_TARGET_ESP32
#define SPI_BUS_HAL                     HSPI_HOST
#define SPI_BUS_HAL_DMA_CHAN            2
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define SPI_BUS_HAL                     SPI2_HOST
#define SPI_BUS_HAL_DMA_CHAN            2
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define SPI_BUS_HAL                     SPI2_HOST
#define SPI_BUS_HAL_DMA_CHAN            2
#endif

#elif defined(PICO_BOARD)
#define _SPI_BUS_HAL_PICO_
#include "pico/stdlib.h"


#else
// Here you can put definitions specific to your framework
#endif

#define SPI_BUS_HAL_FREQ_HZ             100000
//#define SPI_BUS_HAL_MODE_HALFDUPLEX


#ifdef __cplusplus
extern "C" {
#endif

int spi_bus_hal_Init();
int spi_bus_hal_ReadReg(uint8_t* cmd_wr, uint8_t* cmd_rd, size_t cmd_size, uint8_t* data_wr, uint8_t* data_rd, size_t data_size);
int spi_bus_hal_WriteReg(uint8_t* cmd_wr, uint8_t* cmd_rd, size_t cmd_size, uint8_t* data_wr, uint8_t* data_rd, size_t data_size);

#ifdef __cplusplus
}
#endif


#endif /* _SPI_BUS_HAL_H_ */
