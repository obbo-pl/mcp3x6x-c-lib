/*
 * spi_mcp3x6x.c
 *
 * Created on: 7 kwi 2023
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2023 Krzysztof Markiewicz
 */

#include "spi_mcp3x6x.h"
#include <stdlib.h>
#include <string.h>


#define SPI_MCP3X6X_BSWAP_16(x)             ((uint16_t)((((x) >> 8) & 0xff) | (((x) & 0xff) << 8)))
#define SPI_MCP3X6X_BSWAP_24(x)             ((uint32_t)((((x) & 0x00ff0000) >> 16) | ((x) & 0x0000ff00) | (((x) & 0x000000ff) << 16)))
#define SPI_MCP3X6X_BSWAP_32(x)             ((uint32_t)((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) |        \
                                            (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24)))
#define SPI_MCP3X6X_GOTO_ON_ERROR(x,jump) do {            \
            result = x;                                   \
            if (result != chip->result_ok) {              \
                goto jump;                                \
            }                                             \
        } while(0)



// function prototype
int spi_mcp3x6x_GetConfig(spi_mcp3x6x_t* chip);
int spi_mcp3x6x_SendCommandFast(spi_mcp3x6x_t* chip, uint8_t cmd_wr);
int spi_mcp3x6x_SendCommand(spi_mcp3x6x_t* chip, uint8_t cmd, uint8_t* data);
int spi_mcp3x6x_GetChipID(spi_mcp3x6x_t* chip, uint16_t* id);
int spi_mcp3x6x_GetResolution(spi_mcp3x6x_t* chip, uint8_t* resolution);
int spi_mcp3x6x_GetChannelCount(spi_mcp3x6x_t* chip, uint8_t* count);
int spi_mcp3x6x_CheckCRC(spi_mcp3x6x_t* chip, uint8_t* buffer, size_t data_length);
uint16_t spi_mcp3x6x_AddCRC(uint16_t crc, uint8_t item);
int spi_mcp3x6x_CheckInitialization(spi_mcp3x6x_t* chip);
size_t spi_mcp3x6x_GetCommandLength(uint8_t cmd);
size_t spi_mcp3x6x_GetConfigLength(spi_mcp3x6x_t* chip, uint8_t start, uint8_t end);
bool spi_mcp3x6x_IsInternalVref(int variant);
bool spi_mcp3x6x_IsFloatingInput(uint8_t input, uint8_t ch_count);
bool spi_mcp3x6x_IsCRCOnReadEnabled(uint8_t config3);


uint8_t spi_mcp3x6x_cmd_length[16] = {
        0,
        SPI_MCP3X6X_CMD_ADR_CONFIG0_LENGTH,
        SPI_MCP3X6X_CMD_ADR_CONFIG1_LENGTH,
        SPI_MCP3X6X_CMD_ADR_CONFIG2_LENGTH,
        SPI_MCP3X6X_CMD_ADR_CONFIG3_LENGTH,
        SPI_MCP3X6X_CMD_ADR_IRQ_LENGTH,
        SPI_MCP3X6X_CMD_ADR_MUX_LENGTH,
        SPI_MCP3X6X_CMD_ADR_SCAN_LENGTH,
        SPI_MCP3X6X_CMD_ADR_TIMER_LENGTH,
        SPI_MCP3X6X_CMD_ADR_OFFSETCAL_LENGTH,
        SPI_MCP3X6X_CMD_ADR_GAINCAL_LENGTH,
        SPI_MCP3X6X_CMD_ADR_RESERVED1_LENGTH,
        SPI_MCP3X6X_CMD_ADR_RESERVED2_LENGTH,
        SPI_MCP3X6X_CMD_ADR_LOCK_LENGTH,
        SPI_MCP3X6X_CMD_ADR_RESERVED3_LENGTH,
        SPI_MCP3X6X_CMD_ADR_CRCCFG_LENGTH
};



spi_mcp3x6x_t* spi_mcp3x6x_Create()
{
    return calloc(1, sizeof(spi_mcp3x6x_t));
}

void spi_mcp3x6x_Destroy(spi_mcp3x6x_t* chip)
{
    if(chip != NULL) {
        free(chip);
        chip = NULL;
    }
}

int spi_mcp3x6x_Init(spi_mcp3x6x_t* chip,
                     spi_mcp3x6x_variant_t variant,
                     uint8_t spi_address,
                     int (*spi_read)(uint8_t*, uint8_t*, size_t, uint8_t*, uint8_t*, size_t),
                     int (*spi_write)(uint8_t*, uint8_t*, size_t, uint8_t*, uint8_t*, size_t),
                     bool fullduplex,
                     int result_ok)
{
    int result = SPI_MCP3X6X_ERR_NOT_INITIALISED;
    if ((chip != NULL) && (spi_read!= NULL) && (spi_write!= NULL)) {
        chip->variant = variant;
        chip->spi_address = (spi_address << SPI_MCP3X6X_DEVICE_ADDRESS_BITPOS) & SPI_MCP3X6X_DEVICE_ADDRESS_MASK;
        chip->spi_read = spi_read;
        chip->spi_write = spi_write;
        chip->fullduplex = fullduplex;
        chip->result_ok = result_ok;
        chip->initialised = true;
        chip->mdat = false;
        result = spi_mcp3x6x_GetResolution(chip, &chip->resolution);
        if (result == chip->result_ok) {
            result = spi_mcp3x6x_Reset(chip);
            if (result == chip->result_ok) {
                spi_mcp3x6x_cmd_length[0] = chip->resolution / 8;
                result = spi_mcp3x6x_GetConfig(chip);
                if (result == chip->result_ok) {
                    spi_mcp3x6x_cmd_length[0] = spi_mcp3x6x_GetADCRawDataSize(chip);
                    chip->adc_data_cmd = chip->spi_address
                                         | (SPI_MCP3X6X_CMD_ADR_ADCDATA << SPI_MCP3X6X_CMD_ADR_BITPOS)
                                         | SPI_MCP3X6X_CMD_TYPE_STATIC_READ;
                    chip->adc_data_length = spi_mcp3x6x_GetADCRawDataSize(chip);
                    chip->adc_crc_length = spi_mcp3x6x_GetCRCDataSize(chip);
                } else {
                    chip->initialised = false;
                }
            }
        }
    }
    return result;
}

int spi_mcp3x6x_GetConfig(spi_mcp3x6x_t* chip)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        uint8_t buffer[SPI_MCP3X6X_ALIGNED_SPI_BUFFER(9)] = {0};
        uint8_t cmd = chip->spi_address
                      | (SPI_MCP3X6X_CMD_ADR_CONFIG0 << SPI_MCP3X6X_CMD_ADR_BITPOS)
                      | SPI_MCP3X6X_CMD_TYPE_INCREMENTAL_READ;
        uint8_t status[SPI_MCP3X6X_ALIGNED_SPI_BUFFER(1)] = {0};
        SPI_MCP3X6X_GOTO_ON_ERROR(chip->spi_read(&cmd, status, 1, NULL, buffer, 9), finish);
        chip->status = status[0];
        chip->config.config0 = buffer[0];
        chip->config.config1 = buffer[1];
        chip->config.config2 = buffer[2];
        chip->config.config3 = buffer[3];
        chip->config.irq = buffer[4];
        chip->config.mux = buffer[5];
        chip->config.scan = (uint32_t)(0x00000000 | (uint32_t)buffer[6] << 16) | (uint32_t)(buffer[7] << 8) | buffer[8];
    }
finish:
    return result;
}

int spi_mcp3x6x_Conversion(spi_mcp3x6x_t* chip)
{
    return spi_mcp3x6x_SendCommandFast(chip, SPI_MCP3X6X_CMD_FAST_CONVERSION);
}

int spi_mcp3x6x_Standby(spi_mcp3x6x_t* chip)
{
    return spi_mcp3x6x_SendCommandFast(chip, SPI_MCP3X6X_CMD_FAST_STANDBY);
}

int spi_mcp3x6x_Shutdown(spi_mcp3x6x_t* chip)
{
    return spi_mcp3x6x_SendCommandFast(chip, SPI_MCP3X6X_CMD_FAST_SHUTDOWN);
}

int spi_mcp3x6x_FullShutdown(spi_mcp3x6x_t* chip)
{
    return spi_mcp3x6x_SendCommandFast(chip, SPI_MCP3X6X_CMD_FAST_FULL_SHUTDOWN);
}

int spi_mcp3x6x_Reset(spi_mcp3x6x_t* chip)
{
    return spi_mcp3x6x_SendCommandFast(chip, SPI_MCP3X6X_CMD_FAST_RESET);
}

int spi_mcp3x6x_SetVref(spi_mcp3x6x_t* chip, uint8_t vref)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    int result;
    if (spi_mcp3x6x_IsInternalVref(chip->variant)) {
        chip->config.config0 = (chip->config.config0 & ~SPI_MCP3X6X_CONFIG0_VREF_MASK)
                               | (SPI_MCP3X6X_CONFIG0_VREF_MASK & (vref << SPI_MCP3X6X_CONFIG0_VREF_BITPOS));
        result = spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG0, &chip->config.config0);
    } else {
        chip->config.config0 &= ~(SPI_MCP3X6X_CONFIG0_VREF_MASK);
        result = SPI_MCP3X6X_ERR_INVALID_ARG;
    }
    return result;
}

int spi_mcp3x6x_SetClock(spi_mcp3x6x_t* chip, uint8_t clk)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.config0 = (chip->config.config0 & ~SPI_MCP3X6X_CONFIG0_CLK_MASK)
                           | (SPI_MCP3X6X_CONFIG0_CLK_MASK & (clk << SPI_MCP3X6X_CONFIG0_CLK_BITPOS));
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG0, &chip->config.config0);
}

int spi_mcp3x6x_SetCurrentSource(spi_mcp3x6x_t* chip, uint8_t current)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.config0 = (chip->config.config0 & ~SPI_MCP3X6X_CONFIG0_CURRENT_MASK)
                           | (SPI_MCP3X6X_CONFIG0_CURRENT_MASK & (current << SPI_MCP3X6X_CONFIG0_CURRENT_BITPOS));
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG0, &chip->config.config0);
}

int spi_mcp3x6x_SetADCMode(spi_mcp3x6x_t* chip, uint8_t mode)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    if (mode == SPI_MCP3X6X_CONFIG0_MODE_SHUTDOWN) {
        chip->config.config0 = 0x00;
    } else {
        chip->config.config0 = (chip->config.config0 & ~SPI_MCP3X6X_CONFIG0_MODE_MASK)
                               | (SPI_MCP3X6X_CONFIG0_MODE_MASK & (mode << SPI_MCP3X6X_CONFIG0_MODE_BITPOS));
    }
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG0, &chip->config.config0);
}

int spi_mcp3x6x_SetPrescaler(spi_mcp3x6x_t* chip, uint8_t prescaler)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.config1 = (chip->config.config1 & ~SPI_MCP3X6X_CONFIG1_PRESCALER_MASK)
                           | (SPI_MCP3X6X_CONFIG1_PRESCALER_MASK & (prescaler << SPI_MCP3X6X_CONFIG1_PRESCALER_BITPOS));
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG1, &chip->config.config1);
}

int spi_mcp3x6x_SetOversampling(spi_mcp3x6x_t* chip, uint8_t osr)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.config1 = (chip->config.config1 & ~SPI_MCP3X6X_CONFIG1_OSR_MASK)
                           | (SPI_MCP3X6X_CONFIG1_OSR_MASK & (osr << SPI_MCP3X6X_CONFIG1_OSR_BITPOS));
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG1, &chip->config.config1);
}

int spi_mcp3x6x_SetBoost(spi_mcp3x6x_t* chip, uint8_t boost)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.config2 = (chip->config.config2 & ~SPI_MCP3X6X_CONFIG2_BOOST_MASK)
                           | (SPI_MCP3X6X_CONFIG2_BOOST_MASK & (boost << SPI_MCP3X6X_CONFIG2_BOOST_BITPOS));
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG2, &chip->config.config2);
}

int spi_mcp3x6x_SetGain(spi_mcp3x6x_t* chip, uint8_t gain)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.config2 = (chip->config.config2 & ~SPI_MCP3X6X_CONFIG2_GAIN_MASK)
                           | (SPI_MCP3X6X_CONFIG2_GAIN_MASK & (gain << SPI_MCP3X6X_CONFIG2_GAIN_BITPOS));
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG2, &chip->config.config2);
}

int spi_mcp3x6x_EnableAutoZeroingMUX(spi_mcp3x6x_t* chip, bool mux)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    if (mux) {
        chip->config.config2 |= SPI_MCP3X6X_CONFIG2_AZ_MUX_MASK;
    } else {
        chip->config.config2 &= ~(SPI_MCP3X6X_CONFIG2_AZ_MUX_MASK);
    }
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG2, &chip->config.config2);
}

int spi_mcp3x6x_EnableAutoZeroingVref(spi_mcp3x6x_t* chip, bool ref)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    int result;
    if (spi_mcp3x6x_IsInternalVref(chip->variant)) {
        if (ref) {
            chip->config.config2 |= SPI_MCP3X6X_CONFIG2_AZ_REF_MASK;
        } else {
            chip->config.config2 &= ~(SPI_MCP3X6X_CONFIG2_AZ_REF_MASK);
        }
        result = spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG2, &chip->config.config2);
    } else {
        chip->config.config2 |= SPI_MCP3X6X_CONFIG2_AZ_REF_MASK;
        result = SPI_MCP3X6X_ERR_INVALID_ARG;
    }
    return result;
}

int spi_mcp3x6x_SetConversionMode(spi_mcp3x6x_t* chip, uint8_t mode)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.config3 = (chip->config.config3 & ~SPI_MCP3X6X_CONFIG3_CONV_MASK)
                           | (SPI_MCP3X6X_CONFIG3_CONV_MASK & (mode << SPI_MCP3X6X_CONFIG3_CONV_BITPOS));
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG3, &chip->config.config3);
}

int spi_mcp3x6x_SetDataFormat(spi_mcp3x6x_t* chip, uint8_t format)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.config3 = (chip->config.config3 & ~SPI_MCP3X6X_CONFIG3_DATA_MASK)
                           | (SPI_MCP3X6X_CONFIG3_DATA_MASK & (format << SPI_MCP3X6X_CONFIG3_DATA_BITPOS));
    int result = spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG3, &chip->config.config3);
    if (result == chip->result_ok) {
        chip->adc_data_length = spi_mcp3x6x_GetADCRawDataSize(chip);
        spi_mcp3x6x_cmd_length[0] = chip->adc_data_length;
    }
    return result;
}

int spi_mcp3x6x_SetCRCFormat(spi_mcp3x6x_t* chip, uint8_t format)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.config3 = (chip->config.config3 & ~SPI_MCP3X6X_CONFIG3_CRC_MASK)
                           | (SPI_MCP3X6X_CONFIG3_CRC_MASK & (format << SPI_MCP3X6X_CONFIG3_CRC_BITPOS));
    int result = spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG3, &chip->config.config3);
    if (result == chip->result_ok) {
        chip->adc_crc_length = spi_mcp3x6x_GetCRCDataSize(chip);
    }
    return result;
}

int spi_mcp3x6x_EnableCRCOnRead(spi_mcp3x6x_t* chip, bool on_read)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    int result = SPI_MCP3X6X_ERR_INVALID_ARG;
    if (chip->fullduplex) {
        if (on_read) {
            chip->config.config3 |= SPI_MCP3X6X_CONFIG3_CRC_ONREAD_MASK;
        } else {
            chip->config.config3 &= ~(SPI_MCP3X6X_CONFIG3_CRC_ONREAD_MASK);
        }
        result = spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG3, &chip->config.config3);
        if (result == chip->result_ok) {
            chip->adc_crc_length = spi_mcp3x6x_GetCRCDataSize(chip);
        }
    }
    return result;
}

int spi_mcp3x6x_EnableCalibration(spi_mcp3x6x_t* chip, bool offset, bool gain)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    if (offset) {
        chip->config.config3 |= SPI_MCP3X6X_CONFIG3_OFFCAL_MASK;
    } else {
        chip->config.config3 &= ~(SPI_MCP3X6X_CONFIG3_OFFCAL_MASK);
    }
    if (gain) {
        chip->config.config3 |= SPI_MCP3X6X_CONFIG3_GAINCAL_MASK;
    } else {
        chip->config.config3 &= ~(SPI_MCP3X6X_CONFIG3_GAINCAL_MASK);
    }
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_CONFIG3, &chip->config.config3);
}

int spi_mcp3x6x_SetIRQMode(spi_mcp3x6x_t* chip, uint8_t mode)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.irq = (chip->config.irq & ~SPI_MCP3X6X_IRQ_MODE_MASK)
                           | (SPI_MCP3X6X_IRQ_MODE_MASK & (mode << SPI_MCP3X6X_IRQ_MODE_BITPOS));
    if (chip->config.irq & (SPI_MCP3X6X_IRQ_MODE_MDAT_INACTIVE_Z << SPI_MCP3X6X_IRQ_MODE_BITPOS)) {
        chip->mdat = true;
    } else {
        chip->mdat = false;
    }
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_IRQ, &chip->config.irq);
}

int spi_mcp3x6x_EnableFastCommand(spi_mcp3x6x_t* chip, bool fast_cmd)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    if (fast_cmd) {
        chip->config.irq |= SPI_MCP3X6X_IRQ_EN_FASTCMD_MASK;
    } else {
        chip->config.irq &= ~(SPI_MCP3X6X_IRQ_EN_FASTCMD_MASK);
    }
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_IRQ, &chip->config.irq);
}

int spi_mcp3x6x_EnableConversionStartInterruptOutput(spi_mcp3x6x_t* chip, bool stp)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    if (stp) {
        chip->config.irq |= SPI_MCP3X6X_IRQ_EN_STP_MASK;
    } else {
        chip->config.irq &= ~(SPI_MCP3X6X_IRQ_EN_STP_MASK);
    }
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_IRQ, &chip->config.irq);
}

int spi_mcp3x6x_SetMux(spi_mcp3x6x_t* chip, uint8_t mux_vin_p, uint8_t mux_vin_n)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        uint8_t ch_count;
        result = spi_mcp3x6x_GetChannelCount(chip, &ch_count);
        if (result == chip->result_ok) {
            if (!spi_mcp3x6x_IsFloatingInput(mux_vin_p, ch_count) && !spi_mcp3x6x_IsFloatingInput(mux_vin_n, ch_count)) {
                chip->config.mux = ((mux_vin_p << SPI_MCP3X6X_MUX_VIN_P_BITPOS) & SPI_MCP3X6X_MUX_VIN_P_MASK)
                                   | (mux_vin_n & SPI_MCP3X6X_MUX_VIN_N_MASK);
                result = spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_MUX, &chip->config.mux);
            } else {
                result = SPI_MCP3X6X_ERR_INVALID_ARG;
            }
        }
    }
    return result;
}

int spi_mcp3x6x_SetScanDelay(spi_mcp3x6x_t* chip, uint8_t delay)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.scan = (chip->config.scan & ~SPI_MCP3X6X_SCAN_DELAY_MASK)
                           | (SPI_MCP3X6X_SCAN_DELAY_MASK & (delay << SPI_MCP3X6X_SCAN_DELAY_BITPOS));
    uint32_t scan = SPI_MCP3X6X_BSWAP_24(chip->config.scan);
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_SCAN, (uint8_t*)&scan);
}

int spi_mcp3x6x_SetScanSelection(spi_mcp3x6x_t* chip, uint16_t selection)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    chip->config.scan = (chip->config.scan & ~SPI_MCP3X6X_SCAN_SEL_MASK)
                           | (SPI_MCP3X6X_SCAN_SEL_MASK & (selection << SPI_MCP3X6X_SCAN_SEL_BITPOS));
    uint32_t scan = SPI_MCP3X6X_BSWAP_24(chip->config.scan);
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_SCAN, (uint8_t*)&scan);
}

int spi_mcp3x6x_SetTimerDelay(spi_mcp3x6x_t* chip, uint32_t delay)
{
    if (chip == NULL) return SPI_MCP3X6X_ERR_NOT_INITIALISED;
    if (delay >= ((uint32_t)1 << 24)) delay = ((uint32_t)1 << 24) - 1;
    delay = SPI_MCP3X6X_BSWAP_24(delay);
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_TIMER, (uint8_t*)&delay);
}

int spi_mcp3x6x_SetOffsetCalibration(spi_mcp3x6x_t* chip, int32_t offset)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        uint8_t res;
        result = spi_mcp3x6x_GetResolution(chip, &res);
        if (result == chip->result_ok) {
            res--;
            if (offset >= ((int32_t)1 << res)) offset = ((int32_t)1 << res) - 1;
            if (offset < -((int32_t)1 << res)) offset = -((int32_t)1 << res);
            if (res <= SPI_MCP3X6X_RESOLUTION_MAX_MCP346X) offset *= 256;
            offset = SPI_MCP3X6X_BSWAP_24(offset);
            result = spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_OFFSETCAL, (uint8_t*)&offset);
        }
    } else {
        result = SPI_MCP3X6X_ERR_NOT_INITIALISED;
    }
    return result;
}

int spi_mcp3x6x_SetGainCalibration(spi_mcp3x6x_t* chip, uint32_t gain)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        uint8_t res;
        result = spi_mcp3x6x_GetResolution(chip, &res);
        if (result == chip->result_ok) {
            if (gain >= ((uint32_t)1 << res)) gain = ((uint32_t)1 << res) - 1;
            if (res == SPI_MCP3X6X_RESOLUTION_MAX_MCP346X) gain *= 256;
            gain = SPI_MCP3X6X_BSWAP_24(gain);
            result = spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_GAINCAL, (uint8_t*)&gain);
        }
    } else {
        result = SPI_MCP3X6X_ERR_NOT_INITIALISED;
    }
    return result;
}

int spi_mcp3x6x_SetLockPassword(spi_mcp3x6x_t* chip, uint8_t pass)
{
    return spi_mcp3x6x_SendCommand(chip, SPI_MCP3X6X_CMD_ADR_LOCK, &pass);
}

int spi_mcp3x6x_ReadADCRawData(spi_mcp3x6x_t* chip, uint8_t* status, uint8_t* data_rd, size_t length)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        if (chip->mdat) {
            result = SPI_MCP3X6X_ERR_INVALID_STATE;
        } else {
            result = chip->spi_read(&chip->adc_data_cmd, status, 1, NULL, data_rd, length);
            chip->status = status[0];
        }
    }
    return result;
}

int spi_mcp3x6x_ReadADCData(spi_mcp3x6x_t* chip, int32_t* data, uint8_t *ch_id)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        if (chip->mdat) {
            result = SPI_MCP3X6X_ERR_INVALID_STATE;
        } else {
            uint8_t buffer[SPI_MCP3X6X_ALIGNED_SPI_BUFFER(chip->adc_data_length + chip->adc_crc_length)];
            memset(buffer, 0, SPI_MCP3X6X_ALIGNED_SPI_BUFFER(chip->adc_data_length + chip->adc_crc_length) * sizeof(*buffer));
            uint8_t status[SPI_MCP3X6X_ALIGNED_SPI_BUFFER(1)] = {0};
            result = spi_mcp3x6x_ReadADCRawData(chip, status, buffer, chip->adc_data_length + chip->adc_crc_length);
            if (result == chip->result_ok) {
                chip->status = status[0];
                if (spi_mcp3x6x_IsCRCOnReadEnabled(chip->config.config3) & (chip->fullduplex)) {
                    result = spi_mcp3x6x_CheckCRC(chip, buffer, chip->adc_data_length);
                }
                if (result == chip->result_ok) {
                    uint32_t adc_data = 0x00000000;
                    size_t data_size = spi_mcp3x6x_GetADCRawDataSize(chip);
                    for (uint8_t i = 0; i < data_size; i++) {
                        adc_data |= (uint32_t)buffer[i] << (8 * (data_size - i - 1));
                    }
                    switch ((chip->config.config3 & SPI_MCP3X6X_CONFIG3_DATA_MASK) >> SPI_MCP3X6X_CONFIG3_DATA_BITPOS) {
                        case SPI_MCP3X6X_CONFIG3_DATA_ADC_CODING:
                            if (adc_data & ((uint32_t)1 << (chip->resolution - 1))) {
                                adc_data |= (0xFFFFFFFF << chip->resolution);
                            } else {
                                adc_data &= ~(0xFFFFFFFF << chip->resolution);
                            }
                            *data = (int32_t)adc_data;
                            break;
                        case SPI_MCP3X6X_CONFIG3_DATA_LEFT:
                            *data = ((int32_t)adc_data) / ((uint32_t)1 << (32 - chip->resolution));
                            break;
                        case SPI_MCP3X6X_CONFIG3_DATA_RIGHT:
                            *data = (int32_t)adc_data;
                            break;
                        case SPI_MCP3X6X_CONFIG3_DATA_RIGHT_CHID:
                            if (ch_id != NULL) {
                                *ch_id = (uint8_t)((adc_data >> 28) & 0x0F);
                            }
                            if (adc_data & 0x08000000) {
                                adc_data |= 0xF0000000;
                            } else {
                                adc_data &= ~(0xF0000000);
                            }
                            *data = (int32_t) adc_data;
                            break;
                    }
                }
            }
        }
    }
    return result;
}

int spi_mcp3x6x_ReadIRQ(spi_mcp3x6x_t* chip, uint8_t* irq)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        uint8_t temp = 0;
        result = spi_mcp3x6x_ReadCommand(chip, SPI_MCP3X6X_CMD_ADR_IRQ, &temp);
        if (result == chip->result_ok) {
            *irq = temp;
        }
    }
    return result;
}

int spi_mcp3x6x_ReadChipID(spi_mcp3x6x_t* chip, uint16_t* id)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        uint16_t temp = 0;
        result = spi_mcp3x6x_ReadCommand(chip, SPI_MCP3X6X_CMD_ADR_RESERVED3, (uint8_t*)&temp);
        if (result == chip->result_ok) {
            *id = SPI_MCP3X6X_BSWAP_16(temp);
        }
    }
    return result;
}

int spi_mcp3x6x_ReadCRCConfig(spi_mcp3x6x_t* chip, uint16_t* crc)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        uint16_t temp = 0;
        result = spi_mcp3x6x_ReadCommand(chip, SPI_MCP3X6X_CMD_ADR_CRCCFG, (uint8_t*)&temp);
        if (result == chip->result_ok) {
            *crc = SPI_MCP3X6X_BSWAP_16(temp);
        }
    }
    return result;
}

int spi_mcp3x6x_ReadStatus(spi_mcp3x6x_t* chip, uint8_t* status)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        if (chip->fullduplex) {
            uint8_t temp[SPI_MCP3X6X_ALIGNED_SPI_BUFFER(1)] = {0};
            result = chip->spi_read(&chip->adc_data_cmd, temp, 1, NULL, NULL, 0);
            if (result == chip->result_ok) {
                chip->status = temp[0];
                *status = temp[0] & SPI_MCP3X6X_STATUS_MASK;
            }
        } else {
            uint8_t temp = 0;
            result = spi_mcp3x6x_ReadCommand(chip, SPI_MCP3X6X_CMD_ADR_IRQ, &temp);
            if (result == chip->result_ok) {
                *status = (temp >> 4) & SPI_MCP3X6X_STATUS_MASK;
            }
        }
    }
    return result;
}

int spi_mcp3x6x_SendCommandFast(spi_mcp3x6x_t* chip, uint8_t cmd)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        cmd = chip->spi_address | (cmd & SPI_MCP3X6X_CMD_FAST_MASK);
        uint8_t status[SPI_MCP3X6X_ALIGNED_SPI_BUFFER(1)] = {0};
        result = chip->spi_write(&cmd, status, 1, NULL, NULL, 0);
        chip->status = status[0];
    }
    return result;
}

int spi_mcp3x6x_SendCommand(spi_mcp3x6x_t* chip, uint8_t cmd, uint8_t* data)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        uint8_t reg = chip->spi_address
                      | ((cmd << SPI_MCP3X6X_CMD_ADR_BITPOS) & SPI_MCP3X6X_CMD_ADR_MASK)
                      | SPI_MCP3X6X_CMD_TYPE_INCREMENTAL_WRITE;
        uint8_t status[SPI_MCP3X6X_ALIGNED_SPI_BUFFER(1)] = {0};
        result = chip->spi_write(&reg, status, 1, data, NULL, spi_mcp3x6x_GetCommandLength(cmd));
        chip->status = status[0];
    }
    return result;
}

int spi_mcp3x6x_ReadCommand(spi_mcp3x6x_t* chip, uint8_t cmd, uint8_t* data)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if (result == chip->result_ok) {
        uint8_t reg = chip->spi_address
                      | ((cmd << SPI_MCP3X6X_CMD_ADR_BITPOS) & SPI_MCP3X6X_CMD_ADR_MASK)
                      | SPI_MCP3X6X_CMD_TYPE_STATIC_READ;
        size_t data_size = spi_mcp3x6x_GetCommandLength(cmd) + chip->adc_crc_length;
        uint8_t buffer[SPI_MCP3X6X_ALIGNED_SPI_BUFFER(data_size)];
        memset(buffer, 0, SPI_MCP3X6X_ALIGNED_SPI_BUFFER(data_size) * sizeof(*buffer));
        uint8_t status[SPI_MCP3X6X_ALIGNED_SPI_BUFFER(1)] = {0};
        result = chip->spi_read(&reg, status, 1, NULL, buffer, data_size);
        chip->status = status[0];
        if (result == chip->result_ok) {
            if (spi_mcp3x6x_IsCRCOnReadEnabled(chip->config.config3) & (chip->fullduplex)) {
                result = spi_mcp3x6x_CheckCRC(chip, buffer, spi_mcp3x6x_GetCommandLength(cmd));
            }
            if (result == chip->result_ok) {
                for (size_t i = 0; i < spi_mcp3x6x_GetCommandLength(cmd); i++) {
                    data[i] = buffer[i];
                }
            }
        }
    }
    return result;
}

int spi_mcp3x6x_CheckCRC(spi_mcp3x6x_t* chip, uint8_t* buffer, size_t data_length)
{
    int result = spi_mcp3x6x_CheckInitialization(chip);
    if ((result == chip->result_ok) & (chip->fullduplex)) {
        uint16_t crc = spi_mcp3x6x_AddCRC(0x0000, chip->status);
        for(size_t i = 0; i < chip->adc_data_length; i++){
            crc = spi_mcp3x6x_AddCRC(crc, buffer[i]);
        }
        if ((((crc >> 8) & 0xFF) == buffer[chip->adc_data_length]) && (crc & 0xFF) == buffer[chip->adc_data_length + 1]) {
            result = chip->result_ok;
        } else {
            result = SPI_MCP3X6X_ERR_INVALID_CRC;
        }
    }
    return result;
}

uint16_t spi_mcp3x6x_AddCRC(uint16_t crc, uint8_t item)
{
    for (uint8_t b = 0; b < 8; b++) {
        uint16_t top = crc & 0x8000;
        if (item & (0x80 >> b)) {
            top ^= 0x8000;
        }
        crc <<= 1;
        if (top) {
            crc ^= 0x8005;
        }
    }
    return crc & 0xFFFF;
}

int spi_mcp3x6x_CheckInitialization(spi_mcp3x6x_t* chip)
{
    int result = SPI_MCP3X6X_ERR_NOT_INITIALISED;
    if (chip != NULL) {
        if (chip->initialised) {
            result = chip->result_ok;
        }
    }
    return result;
}

size_t spi_mcp3x6x_GetADCRawDataSize(spi_mcp3x6x_t *chip)
{
    size_t length = 0;
    if (spi_mcp3x6x_CheckInitialization(chip) == chip->result_ok) {
        if (((chip->config.config3 & SPI_MCP3X6X_CONFIG3_DATA_MASK) >> SPI_MCP3X6X_CONFIG3_DATA_BITPOS)
                == SPI_MCP3X6X_CONFIG3_DATA_ADC_CODING) {
            length = chip->resolution / 8;
        } else {
            length = 4;
        }
    }
    return length;
}

size_t spi_mcp3x6x_GetCRCDataSize(spi_mcp3x6x_t* chip)
{
    size_t length = 0;
    if (spi_mcp3x6x_CheckInitialization(chip) == chip->result_ok) {
        if (chip->config.config3 & SPI_MCP3X6X_CONFIG3_CRC_ONREAD_MASK) {
            if ((chip->config.config3 & SPI_MCP3X6X_CONFIG3_CRC_MASK) == (SPI_MCP3X6X_CONFIG3_CRC_32 << SPI_MCP3X6X_CONFIG3_CRC_BITPOS)) {
                length += 4;
            } else {
                length += 2;
            }
        }
    }
    return length;
}

size_t spi_mcp3x6x_GetCommandLength(uint8_t cmd)
{
    return spi_mcp3x6x_cmd_length[(cmd & (SPI_MCP3X6X_CMD_ADR_MASK >> SPI_MCP3X6X_CMD_ADR_BITPOS))];
}

size_t spi_mcp3x6x_GetConfigLength(spi_mcp3x6x_t* chip, uint8_t start, uint8_t end)
{
    size_t result = 0;
    if (start > 0xF) start = 0xF;
    if (end > 0xF) end = 0xF;
    if (end >= start) {
        for (uint8_t i = start; i < (end + 1); i++) {
            result += spi_mcp3x6x_cmd_length[i];
        }
    }
    return result;
}

bool spi_mcp3x6x_IsConnected(spi_mcp3x6x_t* chip)
{
    bool result = false;
    if (spi_mcp3x6x_CheckInitialization(chip) == chip->result_ok) {
        uint16_t id_read;
        int err = spi_mcp3x6x_ReadChipID(chip, &id_read);
        if (err == chip->result_ok) {
            uint16_t id_expect;
            err = spi_mcp3x6x_GetChipID(chip, &id_expect);
            if (err == chip->result_ok) {
                if (id_read == id_expect) {
                    result = true;
                }
            }
        }
    }
    return result;
}

int spi_mcp3x6x_GetChipID(spi_mcp3x6x_t* chip, uint16_t* id)
{
    int result = SPI_MCP3X6X_ERR_NOT_INITIALISED;
    if (chip != NULL) {
        switch (chip->variant) {
            case SPI_MCP3X6X_VARIANT_MCP3461:
            case SPI_MCP3X6X_VARIANT_MCP3461R:
                *id = SPI_MCP3X6X_DEVICE_ID_MCP3461;
                result = chip->result_ok;
                break;
            case SPI_MCP3X6X_VARIANT_MCP3462:
            case SPI_MCP3X6X_VARIANT_MCP3462R:
                *id = SPI_MCP3X6X_DEVICE_ID_MCP3462;
                result = chip->result_ok;
                break;
            case SPI_MCP3X6X_VARIANT_MCP3464:
            case SPI_MCP3X6X_VARIANT_MCP3464R:
                *id = SPI_MCP3X6X_DEVICE_ID_MCP3464;
                result = chip->result_ok;
                break;
            case SPI_MCP3X6X_VARIANT_MCP3561:
            case SPI_MCP3X6X_VARIANT_MCP3561R:
                *id = SPI_MCP3X6X_DEVICE_ID_MCP3561;
                result = chip->result_ok;
                break;
            case SPI_MCP3X6X_VARIANT_MCP3562:
            case SPI_MCP3X6X_VARIANT_MCP3562R:
                *id = SPI_MCP3X6X_DEVICE_ID_MCP3562;
                result = chip->result_ok;
                break;
            case SPI_MCP3X6X_VARIANT_MCP3564:
            case SPI_MCP3X6X_VARIANT_MCP3564R:
                *id = SPI_MCP3X6X_DEVICE_ID_MCP3564;
                result = chip->result_ok;
                break;
        }
    }
    return result;
}

int spi_mcp3x6x_GetResolution(spi_mcp3x6x_t* chip, uint8_t* resolution)
{
    int result = SPI_MCP3X6X_ERR_NOT_INITIALISED;
    if (chip != NULL) {
        switch (chip->variant) {
            case SPI_MCP3X6X_VARIANT_MCP3461:
            case SPI_MCP3X6X_VARIANT_MCP3461R:
            case SPI_MCP3X6X_VARIANT_MCP3462:
            case SPI_MCP3X6X_VARIANT_MCP3462R:
            case SPI_MCP3X6X_VARIANT_MCP3464:
            case SPI_MCP3X6X_VARIANT_MCP3464R:
                *resolution = SPI_MCP3X6X_RESOLUTION_MAX_MCP346X;
                result = chip->result_ok;
                break;
            case SPI_MCP3X6X_VARIANT_MCP3561:
            case SPI_MCP3X6X_VARIANT_MCP3561R:
            case SPI_MCP3X6X_VARIANT_MCP3562:
            case SPI_MCP3X6X_VARIANT_MCP3562R:
            case SPI_MCP3X6X_VARIANT_MCP3564:
            case SPI_MCP3X6X_VARIANT_MCP3564R:
                *resolution = SPI_MCP3X6X_RESOLUTION_MAX_MCP356X;
                result = chip->result_ok;
                break;
        }
    }
    return result;
}

int spi_mcp3x6x_GetChannelCount(spi_mcp3x6x_t* chip, uint8_t* count)
{
    int result = SPI_MCP3X6X_ERR_NOT_INITIALISED;
    if (chip != NULL) {
        switch (chip->variant) {
            case SPI_MCP3X6X_VARIANT_MCP3461:
            case SPI_MCP3X6X_VARIANT_MCP3461R:
            case SPI_MCP3X6X_VARIANT_MCP3561:
            case SPI_MCP3X6X_VARIANT_MCP3561R:
                *count = SPI_MCP3X6X_CHANNEL_MAX_MCP3X61;
                result = chip->result_ok;
                break;
            case SPI_MCP3X6X_VARIANT_MCP3462:
            case SPI_MCP3X6X_VARIANT_MCP3462R:
            case SPI_MCP3X6X_VARIANT_MCP3562:
            case SPI_MCP3X6X_VARIANT_MCP3562R:
                *count = SPI_MCP3X6X_CHANNEL_MAX_MCP3X62;
                result = chip->result_ok;
                break;
            case SPI_MCP3X6X_VARIANT_MCP3464:
            case SPI_MCP3X6X_VARIANT_MCP3464R:
            case SPI_MCP3X6X_VARIANT_MCP3564:
            case SPI_MCP3X6X_VARIANT_MCP3564R:
                *count = SPI_MCP3X6X_CHANNEL_MAX_MCP3X64;
                result = chip->result_ok;
                break;
        }
    }
    return result;
}

bool spi_mcp3x6x_IsInternalVref(int variant)
{
    bool result;
    switch (variant) {
        case SPI_MCP3X6X_VARIANT_MCP3461R:
        case SPI_MCP3X6X_VARIANT_MCP3462R:
        case SPI_MCP3X6X_VARIANT_MCP3464R:
        case SPI_MCP3X6X_VARIANT_MCP3561R:
        case SPI_MCP3X6X_VARIANT_MCP3562R:
        case SPI_MCP3X6X_VARIANT_MCP3564R:
            result = true;
            break;
        default:
            result = false;
    }
    return result;
}

bool spi_mcp3x6x_IsFloatingInput(uint8_t input, uint8_t ch_count)
{
    bool result = false;
    if (ch_count == SPI_MCP3X6X_CHANNEL_MAX_MCP3X62) {
        switch (input) {
            case SPI_MCP3X6X_MUX_CH7:
            case SPI_MCP3X6X_MUX_CH6:
            case SPI_MCP3X6X_MUX_CH5:
            case SPI_MCP3X6X_MUX_CH4:
                result = true;
                break;
        }
    } else if (ch_count == SPI_MCP3X6X_CHANNEL_MAX_MCP3X61) {
        switch (input) {
            case SPI_MCP3X6X_MUX_CH7:
            case SPI_MCP3X6X_MUX_CH6:
            case SPI_MCP3X6X_MUX_CH5:
            case SPI_MCP3X6X_MUX_CH4:
            case SPI_MCP3X6X_MUX_CH3:
            case SPI_MCP3X6X_MUX_CH2:
                result = true;
                break;
        }
    }
    return result;
}

bool spi_mcp3x6x_IsCRCOnReadEnabled(uint8_t config3)
{
    bool result = false;
    if (config3 & SPI_MCP3X6X_CONFIG3_CRC_ONREAD_MASK) result = true;
    return result;
}
