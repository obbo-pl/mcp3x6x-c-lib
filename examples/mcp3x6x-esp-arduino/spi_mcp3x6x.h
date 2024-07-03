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
    SPI_MCP3X6X_ERR_INVALID_CRC,
    SPI_MCP3X6X_ERR_RESULT_CODES_OVERLAP,
    SPI_MCP3X6X_ERRCODE_LAST,
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


/**
 * Memory allocation for the controller data structure. If the driver is no longer needed, it should be freed.
 * 
 * @return A pointer to the controller's data structure.
*/
spi_mcp3x6x_t* spi_mcp3x6x_Create();

/**
 * Freeing memory reserved for the controller data structure.
 * 
 * @param chip A pointer to the controller's data structure.
*/
void spi_mcp3x6x_Destroy(spi_mcp3x6x_t* chip);

/**
 * Initializes the controller data structure, resets the ADC, and retrieves configuration register values.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param variant A variation of the ADC IC.
 * @param spi_address ADC chip address hardcoded by manufacturer.
 * @param spi_read Function for reading system registers. It implements the hardware abstraction layer.
 * @param spi_write Function for writing to system registers. It implements the hardware abstraction layer.
 * @param fullduplex A property of the SPI bus.
 * @param result_ok Result code for correctly performed read (spi_read) or write (spi_write) functions.
 * @return Function execution status code.
*/
int spi_mcp3x6x_Init(spi_mcp3x6x_t* chip,
                     spi_mcp3x6x_variant_t variant,
                     uint8_t spi_address,
                     int (*spi_read)(uint8_t*, uint8_t*, size_t, uint8_t*, uint8_t*, size_t),
                     int (*spi_write)(uint8_t*, uint8_t*, size_t, uint8_t*, uint8_t*, size_t),
                     bool fullduplex,
                     int result_ok);

/**
 * ADC Conversion Start/Restart
 * 
 * @param chip A pointer to the controller's data structure.
 * @return Function execution status code.
*/
int spi_mcp3x6x_Conversion(spi_mcp3x6x_t* chip);

/**
 * Places the device in Standby mode.
 * 
 * @param chip A pointer to the controller's data structure.
 * @return Function execution status code.
*/
int spi_mcp3x6x_Standby(spi_mcp3x6x_t* chip);

/**
 * Puts the device into Shutdown mode.
 * 
 * @param chip A pointer to the controller's data structure.
 * @return Function execution status code.
*/
int spi_mcp3x6x_Shutdown(spi_mcp3x6x_t* chip);

/**
 * Resets the device and puts the entire register map in its default state.
 * 
 * @param chip A pointer to the controller's data structure.
 * @return Function execution status code.
*/
int spi_mcp3x6x_Reset(spi_mcp3x6x_t* chip);

/**
 * Puts the device into Full Shutdown mode.
 * 
 * @param chip A pointer to the controller's data structure.
 * @return Function execution status code.
*/
int spi_mcp3x6x_FullShutdown(spi_mcp3x6x_t* chip);

/**
 * Selection of the reference voltage. Applies to R version only.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param vref Voltage reference.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetVref(spi_mcp3x6x_t* chip, uint8_t vref);

/**
 * Clock selection.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param clk Clock source.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetClock(spi_mcp3x6x_t* chip, uint8_t clk);

/**
 * Current source/sink selection for sensor bias.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param current Current source.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetCurrentSource(spi_mcp3x6x_t* chip, uint8_t current);

/**
 * ADC operating mode selection.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param mode Operating mode.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetADCMode(spi_mcp3x6x_t* chip, uint8_t mode);

/**
 * Prescaler value selection for AMCLK.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param prescaler Prescaler value.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetPrescaler(spi_mcp3x6x_t* chip, uint8_t prescaler);

/**
 * Oversampling ratio for delta-sigma A/D conversion.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param osr Oversampling ratio.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetOversampling(spi_mcp3x6x_t* chip, uint8_t osr);

/**
 * ADC bias current selection.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param boost Current ratio.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetBoost(spi_mcp3x6x_t* chip, uint8_t boost);

/**
 * ADC gain selection.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param gain Gain.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetGain(spi_mcp3x6x_t* chip, uint8_t gain);

/**
 * Auto-zeroing MUX setting. Enabling auto-zeroing doubles the conversion time.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param mux Enable.
 * @return Function execution status code.
*/
int spi_mcp3x6x_EnableAutoZeroingMUX(spi_mcp3x6x_t* chip, bool mux);

/**
 * Auto-zeroing internal voltage reference buffer.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param ref Enable.
 * @return Function execution status code.
*/
int spi_mcp3x6x_EnableAutoZeroingVref(spi_mcp3x6x_t* chip, bool ref);

/**
 * Conversion mode selection.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param mode Mode.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetConversionMode(spi_mcp3x6x_t* chip, uint8_t mode);

/**
 * ADC output data format selection.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param format Format.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetDataFormat(spi_mcp3x6x_t* chip, uint8_t format);

/**
 * CRC checksum format selection on read communications.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param format CRC checksum fromat.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetCRCFormat(spi_mcp3x6x_t* chip, uint8_t format);

/**
 * Enable CRC checksum on read communications.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param on_read Enable.
 * @return Function execution status code.
*/
int spi_mcp3x6x_EnableCRCOnRead(spi_mcp3x6x_t* chip, bool on_read);

/**
 * Enable digital offset or gain calibration.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param offset Enable.
 * @param gain Enable.
 * @return Function execution status code.
*/
int spi_mcp3x6x_EnableCalibration(spi_mcp3x6x_t* chip, bool offset, bool gain);

/**
 * Configuration for the IRQ/MDAT pin.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param mode Mode.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetIRQMode(spi_mcp3x6x_t* chip, uint8_t mode);

/**
 * Enable Fast Commands.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param fast_cmd Enable.
 * @return Function execution status code.
*/
int spi_mcp3x6x_EnableFastCommand(spi_mcp3x6x_t* chip, bool fast_cmd);

/**
 * Enable conversion start interrupt output.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param stp Enable.
 * @return Function execution status code.
*/
int spi_mcp3x6x_EnableConversionStartInterruptOutput(spi_mcp3x6x_t* chip, bool stp);

/**
 * Set MUX Vin+ input and Vin- input.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param mux_vin_p Vin+ input.
 * @param mux_vin_n Vin- input.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetMux(spi_mcp3x6x_t* chip, uint8_t mux_vin_p, uint8_t mux_vin_n);

/**
 * Delay time between each conversion during a SCAN cycle.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param delay Delay.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetScanDelay(spi_mcp3x6x_t* chip, uint8_t delay);

/**
 * SCAN channel selection.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param selection ADC channel selection.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetScanSelection(spi_mcp3x6x_t* chip, uint16_t selection);

/**
 * Time interval between two consecutive SCAN cycles.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param delay Timer delay value.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetTimerDelay(spi_mcp3x6x_t* chip, uint32_t delay);

/**
 * Offset error digital calibration code.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param offset Offset calibration.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetOffsetCalibration(spi_mcp3x6x_t* chip, int32_t offset);

/**
 * Gain error digital calibration code. Default value is 0x800000, which provides a gain of 1.
 * The gain error calibration value range in equivalent voltage is [0; 2-LSB]
 * 
 * @param chip A pointer to the controller's data structure.
 * @param gain Gain calibration.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetGainCalibration(spi_mcp3x6x_t* chip, uint32_t gain);

/**
 * Returns the value of the IRQ register.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param irq Pointer to the variable to which the value of the IRQ register will be written.
 * @return Function execution status code.
*/
int spi_mcp3x6x_ReadIRQ(spi_mcp3x6x_t* chip, uint8_t* irq);

/**
 * Returns the chip ID value.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param id A pointer to the variable into which the ID value will be written.
 * @return Function execution status code.
*/
int spi_mcp3x6x_ReadChipID(spi_mcp3x6x_t* chip, uint16_t* id);

/**
 * Returns the CRC-16 configuration checksum. The checksum is continuously calculated internally when 
 * the device is locked.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param crc Pointer to the variable to which the value of the configuration CRC register will be entered.
 * @return Function execution status code.
*/
int spi_mcp3x6x_ReadCRCConfig(spi_mcp3x6x_t* chip, uint16_t* crc);

/**
 * Returns 3 status bits: DR_STATUS(bit 2), CRCREG_STATUS(bit 1), POR_STATUS(bit 0). 
 * When the SPI bus is in full-duplex mode, the status check only takes one byte to transmit. 
 * In the SPI half-duplex mode, the status bits are read from the IRQ register but presented in the same 
 * positions as for the full-duplex bus.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param status Pointer to the variable to which the value of the status bits will be written.
 * @return Function execution status code.
*/
int spi_mcp3x6x_ReadStatus(spi_mcp3x6x_t* chip, uint8_t* status);

/**
 * Reads the contents of the configuration register. If enabled, the checksum of the data will be checked. 
 * Available register addresses and length of returned data can be found in spi_mcp3x6x_def.h.
 * (SPI_MCP3X6X_CMD_ADR_xxxx and SPI_MCP3X6X_CMD_ADR_xxxx_LENGTH)
 * 
 * @param chip A pointer to the controller's data structure.
 * @param cmd Configuration register address.
 * @param data A pointer to the buffer to which the contents of the register will be written.
 * @return Function execution status code.
*/
int spi_mcp3x6x_ReadCommand(spi_mcp3x6x_t* chip, uint8_t cmd, uint8_t* data);

/**
 * Write access password entry code. Enter 0xA5 to unlock.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param pass Password.
 * @return Function execution status code.
*/
int spi_mcp3x6x_SetLockPassword(spi_mcp3x6x_t* chip, uint8_t pass);

/**
 * Reads the content of the ADCDATA register with minimal interference in data transfer transactions. 
 * The data buffer is exposed directly for reading from the SPI bus. The data CRC is not checked, but 
 * the CRC data is part of the transaction and may be sent to the buffer. The length of the data to be read 
 * can be determined as the sum of the lengths of the ADC data and the CRC data. The length of the read data 
 * may be shorter, you don't have to read all of it if you don't need it. If you are using the full-duplex 
 * SPI bus, the status data will also be read.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param status Pointer to the variable to which the value of the status bits will be written.
 * @param data_rd A pointer to the buffer to which the contents of the ADCDATA register and CRC will be written.
 * @param length The length of the data to read.
 * @return Function execution status code.
*/
int spi_mcp3x6x_ReadADCRawData(spi_mcp3x6x_t* chip, uint8_t* status, uint8_t* data_rd, size_t length);

/**
 * It reads the measurement value from the ADCDATA register and returns it as a 32-bit integer, 
 * regardless of the configured data format. Depending on the configured format, the range of values 
 * may be different, and the ID of the measured channel may also be transferred.
 * 
 * @param chip A pointer to the controller's data structure.
 * @param data Pointer to the variable to which the ADC measurement result will be written.
 * @param ch_id Pointer to the variable to which the channel ID will be written.
 * @return Function execution status code.
*/
int spi_mcp3x6x_ReadADCData(spi_mcp3x6x_t* chip, int32_t* data, uint8_t* ch_id);

/**
 * Determines the ADC data length based on the current configuration. 
 * It may be helpful to set the buffer size when reading RAW data.
 * 
 * @param chip A pointer to the controller's data structure.
 * @return ADC data length.
*/
size_t spi_mcp3x6x_GetADCRawDataSize(spi_mcp3x6x_t* chip);

/**
 * Determines the length of the CRC data based on the current configuration. 
 * It may be helpful to set the buffer size when reading RAW data.
 * 
 * @param chip A pointer to the controller's data structure.
 * @return CRC data length.
*/
size_t spi_mcp3x6x_GetCRCDataSize(spi_mcp3x6x_t* chip);

/**
 * Checks if the chip is connected. Reads the ID and compares it to the value for the configured IC.
 * 
 * @param chip A pointer to the controller's data structure.
 * @return True if the chip ID can be read and is valid for the configured IC.
*/
bool spi_mcp3x6x_IsConnected(spi_mcp3x6x_t* chip);



#ifdef __cplusplus
}
#endif


#endif /* _SPI_MCP3X6X_H_ */
