/*
 * mcp3x6x-scan-irq.c
 *
 * Created on: 19 paü 2023
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2023 Krzysztof Markiewicz
 */

#include <stdio.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "freertos/task.h"

#include "spi_bus_hal.h"
#include "spi_mcp3x6x.h"


#define MCP3X6X_SCAN_IRQ_IO         (13)

spi_mcp3x6x_t* adc;


void app_main(void)
{
    // Init SPI
    spi_bus_hal_Init();
    // Init IRQ GPIO
    gpio_config_t io_conf;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MCP3X6X_SCAN_IRQ_IO);
    gpio_config(&io_conf);
    // Init chip
    adc = spi_mcp3x6x_Create();
    spi_mcp3x6x_Init(adc,
                     SPI_MCP3X6X_VARIANT_MCP3564R,
                     SPI_MCP3X6X_DEVICE_ADDRESS_DEFAULT,
                     spi_bus_hal_ReadReg,
                     spi_bus_hal_WriteReg,
                     true,
                     SPI_BUS_HAL_OK);
    // Connectivity check
    if (spi_mcp3x6x_IsConnected(adc)) {
        printf("MCP3564R is connected. \n");
    } else {
        printf("The MCP3564R is not responding or the chip ID is not correct. \n");
    }
    // Basic setup
    spi_mcp3x6x_SetClock(adc, SPI_MCP3X6X_CLK_INT);
    spi_mcp3x6x_SetPrescaler(adc, SPI_MCP3X6X_PRESCALER_DIV2);
    spi_mcp3x6x_SetVref(adc, SPI_MCP3X6X_VREF_INT);
    // Enable calibration
    spi_mcp3x6x_EnableAutoZeroingMUX(adc, true);
    spi_mcp3x6x_EnableAutoZeroingVref(adc, true);
    // Data conversion setting
    spi_mcp3x6x_SetOversampling(adc, SPI_MCP3X6X_OSR_49152);
    spi_mcp3x6x_SetDataFormat(adc, SPI_MCP3X6X_DATA_RIGHT_CHID);
    spi_mcp3x6x_SetConversionMode(adc, SPI_MCP3X6X_CONV_CONTINUOUS);
    spi_mcp3x6x_SetADCMode(adc, SPI_MCP3X6X_MODE_CONVERSION);
    // Scan settings
    spi_mcp3x6x_SetScanDelay(adc, SPI_MCP3X6X_SCAN_DELAY_512);
    spi_mcp3x6x_SetTimerDelay(adc, 0xFFFFF);
    spi_mcp3x6x_SetScanSelection(adc, SPI_MCP3X6X_SCAN_AVDD | SPI_MCP3X6X_SCAN_TEMP);
    // With the above settings, the delay between conversions in a scan cycle is approximately 160ms,
    // so checking the status every 50ms will be sufficient.
    while (1)
    {
        const float vref = (float)SPI_MCP3X6X_INTERNAL_VREF_MV / 1000;
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (gpio_get_level(MCP3X6X_SCAN_IRQ_IO) == 0) {
            int32_t adc_data = 0;
            uint8_t ch_id = 0;
            spi_mcp3x6x_ReadADCData(adc, &adc_data, &ch_id);
            float value;
            switch (0x0001 << ch_id) {
                case SPI_MCP3X6X_SCAN_AVDD:
                    // AVDD transfer function
                    value = 3.0 * adc_data * vref / (1L << 23);
                    printf("AVDD:        %0.4f [V] \n", value);
                    break;
                case SPI_MCP3X6X_SCAN_TEMP:
                    // Temperature sensor transfer function for MCP3564R
                    value = 0.00040096 * adc_data * vref - 269.13;
                    printf("Temperature: %0.2f [C] \n", value);
                    break;
            }
        }
    }
}

