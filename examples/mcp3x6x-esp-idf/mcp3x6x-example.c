/*
 * mcp3x6x-example.c
 *
 * Created on: 21 maj 2023
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


spi_mcp3x6x_t* adc;


void app_main(void)
{
    // Init SPI
    spi_bus_hal_Init();
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
    spi_mcp3x6x_SetVref(adc, SPI_MCP3X6X_VREF_INT);
    // Enable calibration
    spi_mcp3x6x_EnableAutoZeroingMUX(adc, true);
    spi_mcp3x6x_EnableAutoZeroingVref(adc, true);
    // Data conversion setting
    spi_mcp3x6x_SetOversampling(adc, SPI_MCP3X6X_OSR_20480);
    spi_mcp3x6x_SetDataFormat(adc, SPI_MCP3X6X_DATA_RIGHT);
    spi_mcp3x6x_SetConversionMode(adc, SPI_MCP3X6X_CONV_ONE_STANDBY);
    spi_mcp3x6x_SetIRQMode(adc, SPI_MCP3X6X_MODE_IRQ_INACTIVE_LH);
    // Select the internal temperature sensor.
    spi_mcp3x6x_SetMux(adc, SPI_MCP3X6X_VIN_TEMPERATURE_P, SPI_MCP3X6X_VIN_TEMPERATURE_M);
    //
    const float vref = 2.4;
    while (1)
    {
        spi_mcp3x6x_Conversion(adc);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        int32_t adc_data;
        spi_mcp3x6x_ReadADCData(adc, &adc_data, NULL);
        // Temperature sensor transfer function for MCP3564R
        float t = (0.00040096 * adc_data * vref) - 269.13;
        printf("Temperature: %0.2f [C] \n", t);
    }
}

