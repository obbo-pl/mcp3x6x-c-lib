/*
 * spi_mcp3x6x.h
 *
 * Created on: 7 kwi 2023
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2023 Krzysztof Markiewicz
 */

#ifndef _SPI_MCP3X6X_H_
#define _SPI_MCP3X6X_H_

#include "spi_mcp3x6x_def.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


typedef enum spi_mcp3x6x_variant spi_mcp3x6x_variant_t;


// Some applications require SPI buffer alignment.
// 4 bytes aligned
#define SPI_MCP3X6X_ALIGNED_SPI_BUFFER(x)   (((x) & 0x3) == 0?(x):(((x) / 4) + 1) * 4)
// 2 bytes aligned
//#define SPI_MCP3X6X_ALIGNED_SPI_BUFFER(x)   (((x) & 0x1) == 0?(x):(((x) / 2) + 1) * 2)
// alignment is not required
//#define SPI_MCP3X6X_ALIGNED_SPI_BUFFER(x)   (x)


enum spi_mcp3x6x_vref {
    SPI_MCP3X6X_VREF_EXT            = SPI_MCP3X6X_CONFIG0_VREF_EXT,
    SPI_MCP3X6X_VREF_INT            = SPI_MCP3X6X_CONFIG0_VREF_INT
};

enum spi_mcp3x6x_clk {
    SPI_MCP3X6X_CLK_EXT             = SPI_MCP3X6X_CONFIG0_CLK_EXT,
    SPI_MCP3X6X_CLK_INT             = SPI_MCP3X6X_CONFIG0_CLK_INT,
    SPI_MCP3X6X_CLK_INT_OUT_AMCLK   = SPI_MCP3X6X_CONFIG0_CLK_INT_OUT
};

enum spi_mcp3x6x_current {
    SPI_MCP3X6X_CURRENT_0_0_uA,
    SPI_MCP3X6X_CURRENT_0_9_uA,
    SPI_MCP3X6X_CURRENT_3_7_uA,
    SPI_MCP3X6X_CURRENT_15_0_uA
};

enum spi_mcp3x6x_mode {
    SPI_MCP3X6X_MODE_SHUTDOWN       = SPI_MCP3X6X_CONFIG0_MODE_SHUTDOWN,
    SPI_MCP3X6X_MODE_STANDBY        = SPI_MCP3X6X_CONFIG0_MODE_STANDBY,
    SPI_MCP3X6X_MODE_CONVERSION     = SPI_MCP3X6X_CONFIG0_MODE_CONVERSION,
};

enum spi_mcp3x6x_prescaler {
    SPI_MCP3X6X_PRESCALER_DIV1,
    SPI_MCP3X6X_PRESCALER_DIV2,
    SPI_MCP3X6X_PRESCALER_DIV4,
    SPI_MCP3X6X_PRESCALER_DIV8,
};

enum spi_mcp3x6x_osr {
    SPI_MCP3X6X_OSR_32,
    SPI_MCP3X6X_OSR_64,
    SPI_MCP3X6X_OSR_128,
    SPI_MCP3X6X_OSR_256,
    SPI_MCP3X6X_OSR_512,
    SPI_MCP3X6X_OSR_1024,
    SPI_MCP3X6X_OSR_2048,
    SPI_MCP3X6X_OSR_4096,
    SPI_MCP3X6X_OSR_8192,
    SPI_MCP3X6X_OSR_16384,
    SPI_MCP3X6X_OSR_20480,
    SPI_MCP3X6X_OSR_24576,
    SPI_MCP3X6X_OSR_40960,
    SPI_MCP3X6X_OSR_49152,
    SPI_MCP3X6X_OSR_81920,
    SPI_MCP3X6X_OSR_98304
};

enum spi_mcp3x6x_boost {
    SPI_MCP3X6X_BOOST_x0_5,
    SPI_MCP3X6X_BOOST_x0_66,
    SPI_MCP3X6X_BOOST_x1,
    SPI_MCP3X6X_BOOST_x2
};

enum spi_mcp3x6x_gain {
    SPI_MCP3X6X_GAIN_div3,
    SPI_MCP3X6X_GAIN_x1,
    SPI_MCP3X6X_GAIN_x2,
    SPI_MCP3X6X_GAIN_x4,
    SPI_MCP3X6X_GAIN_x8,
    SPI_MCP3X6X_GAIN_x16,
    SPI_MCP3X6X_GAIN_x32,
    SPI_MCP3X6X_GAIN_x64
};

enum spi_mcp3x6x_conversion {
    SPI_MCP3X6X_CONV_CONTINUOUS     = SPI_MCP3X6X_CONFIG3_CONV_CONTINUOUS,
    SPI_MCP3X6X_CONV_ONE_STANDBY    = SPI_MCP3X6X_CONFIG3_CONV_ONE_STANDBY,
    SPI_MCP3X6X_CONV_ONE_SHUTDOWN   = SPI_MCP3X6X_CONFIG3_CONV_ONE_SHUTDOWN
};

enum spi_mcp3x6x_data_format {
    SPI_MCP3X6X_DATA_ADC_CODING,
    SPI_MCP3X6X_DATA_LEFT,
    SPI_MCP3X6X_DATA_RIGHT,
    SPI_MCP3X6X_DATA_RIGHT_CHID
};

enum spi_mcp3x6x_crc_format {
    SPI_MCP3X6X_CRC_16,
    SPI_MCP3X6X_CRC_32
};

enum spi_mcp3x6x_irq {
    SPI_MCP3X6X_MODE_IRQ_INACTIVE_Z,
    SPI_MCP3X6X_MODE_IRQ_INACTIVE_LH,
    SPI_MCP3X6X_MODE_MDAT_INACTIVE_Z,
    SPI_MCP3X6X_MODE_MDAT_INACTIVE_LH
};

enum spi_mcp3x6x_mux {
    SPI_MCP3X6X_VIN_CH0             = SPI_MCP3X6X_MUX_CH0,
    SPI_MCP3X6X_VIN_CH1,
    SPI_MCP3X6X_VIN_CH2,
    SPI_MCP3X6X_VIN_CH3,
    SPI_MCP3X6X_VIN_CH4,
    SPI_MCP3X6X_VIN_CH5,
    SPI_MCP3X6X_VIN_CH6,
    SPI_MCP3X6X_VIN_CH7,
    SPI_MCP3X6X_VIN_AGND,
    SPI_MCP3X6X_VIN_AVDD,
    SPI_MCP3X6X_VIN_REFIN_P         = SPI_MCP3X6X_MUX_REFIN_P,
    SPI_MCP3X6X_VIN_REFIN_M,
    SPI_MCP3X6X_VIN_TEMPERATURE_P,
    SPI_MCP3X6X_VIN_TEMPERATURE_M,
    SPI_MCP3X6X_VIN_VCM
};

enum spi_mcp3x6x_scan_delay {
    SPI_MCP3X6X_SCAN_DELAY_0,
    SPI_MCP3X6X_SCAN_DELAY_8,
    SPI_MCP3X6X_SCAN_DELAY_16,
    SPI_MCP3X6X_SCAN_DELAY_32,
    SPI_MCP3X6X_SCAN_DELAY_64,
    SPI_MCP3X6X_SCAN_DELAY_128,
    SPI_MCP3X6X_SCAN_DELAY_256,
    SPI_MCP3X6X_SCAN_DELAY_512
};

enum spi_mcp3x6x_scan {
    SPI_MCP3X6X_SCAN_OFFSET         = SPI_MCP3X6X_SCAN_SEL_OFFSET,
    SPI_MCP3X6X_SCAN_VCM            = SPI_MCP3X6X_SCAN_SEL_VCM,
    SPI_MCP3X6X_SCAN_AVDD           = SPI_MCP3X6X_SCAN_SEL_AVDD,
    SPI_MCP3X6X_SCAN_TEMP           = SPI_MCP3X6X_SCAN_SEL_TEMP,
    SPI_MCP3X6X_SCAN_CH6_CH7        = SPI_MCP3X6X_SCAN_SEL_CH6_CH7,
    SPI_MCP3X6X_SCAN_CH4_CH5        = SPI_MCP3X6X_SCAN_SEL_CH4_CH5,
    SPI_MCP3X6X_SCAN_CH2_CH3        = SPI_MCP3X6X_SCAN_SEL_CH2_CH3,
    SPI_MCP3X6X_SCAN_CH0_CH1        = SPI_MCP3X6X_SCAN_SEL_CH0_CH1,
    SPI_MCP3X6X_SCAN_CH7            = SPI_MCP3X6X_SCAN_SEL_CH7,
    SPI_MCP3X6X_SCAN_CH6            = SPI_MCP3X6X_SCAN_SEL_CH6,
    SPI_MCP3X6X_SCAN_CH5            = SPI_MCP3X6X_SCAN_SEL_CH5,
    SPI_MCP3X6X_SCAN_CH4            = SPI_MCP3X6X_SCAN_SEL_CH4,
    SPI_MCP3X6X_SCAN_CH3            = SPI_MCP3X6X_SCAN_SEL_CH3,
    SPI_MCP3X6X_SCAN_CH2            = SPI_MCP3X6X_SCAN_SEL_CH2,
    SPI_MCP3X6X_SCAN_CH1            = SPI_MCP3X6X_SCAN_SEL_CH1,
    SPI_MCP3X6X_SCAN_CH0            = SPI_MCP3X6X_SCAN_SEL_CH0
};




#ifndef SPI_MCP3X6X_ERR_OFFSET
#define SPI_MCP3X6X_ERR_OFFSET           1
#endif // SPI_MCP3X6X_ERR_OFFSET
enum spi_mcp3x6x_err {
    SPI_MCP3X6X_ERR_TIMEOUT = SPI_MCP3X6X_ERR_OFFSET,
    SPI_MCP3X6X_ERR_INVALID_STATE,
    SPI_MCP3X6X_ERR_NOT_INITIALISED,
    SPI_MCP3X6X_ERR_INVALID_ARG,
    SPI_MCP3X6X_ERR_INVALID_CRC
};

typedef struct {
    uint8_t config0;
    uint8_t config1;
    uint8_t config2;
    uint8_t config3;
    uint8_t irq;
    uint8_t mux;
    uint32_t scan;
} spi_mcp3x6x_config_t;

typedef struct {
    bool initialised;
    spi_mcp3x6x_variant_t variant;
    uint8_t spi_address;
    int (*spi_read)(uint8_t*, uint8_t*, size_t, uint8_t*, uint8_t*, size_t);
    int (*spi_write)(uint8_t*, uint8_t*, size_t, uint8_t*, uint8_t*, size_t);
    bool fullduplex;
    bool mdat;
    spi_mcp3x6x_config_t config;
    uint8_t resolution;
    uint8_t status;
    uint8_t adc_data_cmd;
    uint8_t adc_data_length;
    uint8_t adc_crc_length;
    int result_ok;
} spi_mcp3x6x_t;



spi_mcp3x6x_t* spi_mcp3x6x_Create();
void spi_mcp3x6x_Destroy(spi_mcp3x6x_t* chip);
int spi_mcp3x6x_Init(spi_mcp3x6x_t* chip,
                     spi_mcp3x6x_variant_t variant,
                     uint8_t spi_address,
                     int (*spi_read)(uint8_t*, uint8_t*, size_t, uint8_t*, uint8_t*, size_t),
                     int (*spi_write)(uint8_t*, uint8_t*, size_t, uint8_t*, uint8_t*, size_t),
                     bool fullduplex,
                     int result_ok);
int spi_mcp3x6x_Conversion(spi_mcp3x6x_t* chip);
int spi_mcp3x6x_Standby(spi_mcp3x6x_t* chip);
int spi_mcp3x6x_Shutdown(spi_mcp3x6x_t* chip);
int spi_mcp3x6x_Reset(spi_mcp3x6x_t* chip);
int spi_mcp3x6x_FullShutdown(spi_mcp3x6x_t* chip);
int spi_mcp3x6x_SetVref(spi_mcp3x6x_t* chip, uint8_t vref);
int spi_mcp3x6x_SetClock(spi_mcp3x6x_t* chip, uint8_t clk);
int spi_mcp3x6x_SetCurrentSource(spi_mcp3x6x_t* chip, uint8_t current);
int spi_mcp3x6x_SetADCMode(spi_mcp3x6x_t* chip, uint8_t mode);
int spi_mcp3x6x_SetPrescaler(spi_mcp3x6x_t* chip, uint8_t prescaler);
int spi_mcp3x6x_SetOversampling(spi_mcp3x6x_t* chip, uint8_t osr);
int spi_mcp3x6x_SetBoost(spi_mcp3x6x_t* chip, uint8_t boost);
int spi_mcp3x6x_SetGain(spi_mcp3x6x_t* chip, uint8_t gain);
int spi_mcp3x6x_EnableAutoZeroingMUX(spi_mcp3x6x_t* chip, bool mux);
int spi_mcp3x6x_EnableAutoZeroingVref(spi_mcp3x6x_t* chip, bool ref);
int spi_mcp3x6x_SetConversionMode(spi_mcp3x6x_t* chip, uint8_t mode);
int spi_mcp3x6x_SetDataFormat(spi_mcp3x6x_t* chip, uint8_t format);
int spi_mcp3x6x_SetCRCFormat(spi_mcp3x6x_t* chip, uint8_t format);
int spi_mcp3x6x_EnableCRCOnRead(spi_mcp3x6x_t* chip, bool on_read);
int spi_mcp3x6x_EnableCalibration(spi_mcp3x6x_t* chip, bool offset, bool gain);
int spi_mcp3x6x_SetIRQMode(spi_mcp3x6x_t* chip, uint8_t mode);
int spi_mcp3x6x_EnableFastCommand(spi_mcp3x6x_t* chip, bool fast_cmd);
int spi_mcp3x6x_EnableConversionStartInterruptOutput(spi_mcp3x6x_t* chip, bool stp);
int spi_mcp3x6x_SetMux(spi_mcp3x6x_t* chip, uint8_t mux_vin_p, uint8_t mux_vin_n);
int spi_mcp3x6x_SetScanDelay(spi_mcp3x6x_t* chip, uint8_t delay);
int spi_mcp3x6x_SetScanSelection(spi_mcp3x6x_t* chip, uint16_t selection);
int spi_mcp3x6x_SetTimerDelay(spi_mcp3x6x_t* chip, uint32_t delay);
int spi_mcp3x6x_SetOffsetCalibration(spi_mcp3x6x_t* chip, int32_t offset);
int spi_mcp3x6x_SetGainCalibration(spi_mcp3x6x_t* chip, uint32_t gain);
int spi_mcp3x6x_ReadIRQ(spi_mcp3x6x_t* chip, uint8_t* irq);
int spi_mcp3x6x_ReadChipID(spi_mcp3x6x_t* chip, uint16_t* id);
int spi_mcp3x6x_ReadCRCConfig(spi_mcp3x6x_t* chip, uint16_t* crc);
int spi_mcp3x6x_ReadStatus(spi_mcp3x6x_t* chip, uint8_t* status);
int spi_mcp3x6x_ReadCommand(spi_mcp3x6x_t* chip, uint8_t cmd, uint8_t* data);
int spi_mcp3x6x_ReadADCRawData(spi_mcp3x6x_t* chip, uint8_t* status, uint8_t* data_rd, size_t length);
int spi_mcp3x6x_ReadADCData(spi_mcp3x6x_t* chip, int32_t* data, uint8_t *ch_id);
size_t spi_mcp3x6x_GetADCRawDataSize(spi_mcp3x6x_t* chip);
size_t spi_mcp3x6x_GetCRCDataSize(spi_mcp3x6x_t* chip);
bool spi_mcp3x6x_IsConnected(spi_mcp3x6x_t* chip);



#ifdef __cplusplus
}
#endif


#endif /* _SPI_MCP3X6X_H_ */
