/*
 * spi_mcp3x6x_def.h
 *
 * Created on: 7 kwi 2023
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2023 Krzysztof Markiewicz
 */

#ifndef _SPI_MCP3X6X_DEF_H_
#define _SPI_MCP3X6X_DEF_H_


enum spi_mcp3x6x_variant {
    SPI_MCP3X6X_VARIANT_MCP3461,                // Two-Channel, 153.6 kSPS, Low Noise, 16-bit Delta-Sigma ADC
    SPI_MCP3X6X_VARIANT_MCP3462,                // Four-Channel, 153.6 kSPS, Low Noise, 16-bit Delta-Sigma ADC
    SPI_MCP3X6X_VARIANT_MCP3464,                // Eight-Channel, 153.6 kSPS, Low Noise, 16-bit Delta-Sigma ADC
    SPI_MCP3X6X_VARIANT_MCP3561,                // Two-Channel, 153.6 kSPS, Low Noise, 24-bit Delta-Sigma ADC
    SPI_MCP3X6X_VARIANT_MCP3562,                // Four-Channel, 153.6 kSPS, Low Noise, 24-bit Delta-Sigma ADC
    SPI_MCP3X6X_VARIANT_MCP3564,                // Eight-Channel, 153.6 kSPS, Low Noise, 24-bit Delta-Sigma ADC
    SPI_MCP3X6X_VARIANT_MCP3461R,               // Two-Channel, 153.6 kSPS, Low Noise, 16-bit Delta-Sigma ADC with Internal Voltage Reference
    SPI_MCP3X6X_VARIANT_MCP3462R,               // Four-Channel, 153.6 kSPS, Low Noise, 16-bit Delta-Sigma ADC with Internal Voltage Reference
    SPI_MCP3X6X_VARIANT_MCP3464R,               // Eight-Channel, 153.6 kSPS, Low Noise, 16-bit Delta-Sigma ADC with Internal Voltage Reference
    SPI_MCP3X6X_VARIANT_MCP3561R,               // Two-Channel, 153.6 kSPS, Low Noise, 24-bit Delta-Sigma ADC with Internal Voltage Reference
    SPI_MCP3X6X_VARIANT_MCP3562R,               // Four-Channel, 153.6 kSPS, Low Noise, 24-bit Delta-Sigma ADC with Internal Voltage Reference
    SPI_MCP3X6X_VARIANT_MCP3564R,               // Eight-Channel, 153.6 kSPS, Low Noise, 24-bit Delta-Sigma ADC with Internal Voltage Reference
};

#define SPI_MCP3X6X_DEVICE_ID_MCP3461           (0x0008)
#define SPI_MCP3X6X_DEVICE_ID_MCP3462           (0x0009)
#define SPI_MCP3X6X_DEVICE_ID_MCP3464           (0x000B)
#define SPI_MCP3X6X_DEVICE_ID_MCP3561           (0x000C)
#define SPI_MCP3X6X_DEVICE_ID_MCP3562           (0x000D)
#define SPI_MCP3X6X_DEVICE_ID_MCP3564           (0x000F)

#define SPI_MCP3X6X_RESOLUTION_MAX_MCP346X      (16)
#define SPI_MCP3X6X_RESOLUTION_MAX_MCP356X      (24)
#define SPI_MCP3X6X_CHANNEL_MAX_MCP3X61         (2)
#define SPI_MCP3X6X_CHANNEL_MAX_MCP3X62         (4)
#define SPI_MCP3X6X_CHANNEL_MAX_MCP3X64         (8)
#define SPI_MCP3X6X_INTERNAL_VREF_MV            (2400)

#define SPI_MCP3X6X_DEVICE_ADDRESS_MASK         (0b11000000)
#define SPI_MCP3X6X_DEVICE_ADDRESS_BITPOS       (6)
#define SPI_MCP3X6X_DEVICE_ADDRESS_DEFAULT      (0b01)

// Fast Command
#define SPI_MCP3X6X_CMD_FAST_MASK               (0b00111111)
#define SPI_MCP3X6X_CMD_FAST_BITPOS             (0)
#define SPI_MCP3X6X_CMD_FAST_CONVERSION         (0b101000)
#define SPI_MCP3X6X_CMD_FAST_STANDBY            (0b101100)
#define SPI_MCP3X6X_CMD_FAST_SHUTDOWN           (0b110000)
#define SPI_MCP3X6X_CMD_FAST_FULL_SHUTDOWN      (0b110100)
#define SPI_MCP3X6X_CMD_FAST_RESET              (0b111000)

// Command Address
#define SPI_MCP3X6X_CMD_ADR_MASK                (0b00111100)
#define SPI_MCP3X6X_CMD_ADR_BITPOS              (2)
#define SPI_MCP3X6X_CMD_ADR_ADCDATA             (0b0000)
#define SPI_MCP3X6X_CMD_ADR_CONFIG0             (0b0001)
#define SPI_MCP3X6X_CMD_ADR_CONFIG1             (0b0010)
#define SPI_MCP3X6X_CMD_ADR_CONFIG2             (0b0011)
#define SPI_MCP3X6X_CMD_ADR_CONFIG3             (0b0100)
#define SPI_MCP3X6X_CMD_ADR_IRQ                 (0b0101)
#define SPI_MCP3X6X_CMD_ADR_MUX                 (0b0110)
#define SPI_MCP3X6X_CMD_ADR_SCAN                (0b0111)
#define SPI_MCP3X6X_CMD_ADR_TIMER               (0b1000)
#define SPI_MCP3X6X_CMD_ADR_OFFSETCAL           (0b1001)
#define SPI_MCP3X6X_CMD_ADR_GAINCAL             (0b1010)
#define SPI_MCP3X6X_CMD_ADR_RESERVED1           (0b1011)
#define SPI_MCP3X6X_CMD_ADR_RESERVED2           (0b1100)
#define SPI_MCP3X6X_CMD_ADR_LOCK                (0b1101)
#define SPI_MCP3X6X_CMD_ADR_RESERVED3           (0b1110)
#define SPI_MCP3X6X_CMD_ADR_CRCCFG              (0b1111)

#define SPI_MCP346X_CMD_ADR_ADCDATA_LENGTH      (2)
#define SPI_MCP356X_CMD_ADR_ADCDATA_LENGTH      (3)
#define SPI_MCP3X6X_CMD_ADR_CONFIG0_LENGTH      (1)
#define SPI_MCP3X6X_CMD_ADR_CONFIG1_LENGTH      (1)
#define SPI_MCP3X6X_CMD_ADR_CONFIG2_LENGTH      (1)
#define SPI_MCP3X6X_CMD_ADR_CONFIG3_LENGTH      (1)
#define SPI_MCP3X6X_CMD_ADR_IRQ_LENGTH          (1)
#define SPI_MCP3X6X_CMD_ADR_MUX_LENGTH          (1)
#define SPI_MCP3X6X_CMD_ADR_SCAN_LENGTH         (3)
#define SPI_MCP3X6X_CMD_ADR_TIMER_LENGTH        (3)
#define SPI_MCP3X6X_CMD_ADR_OFFSETCAL_LENGTH    (3)
#define SPI_MCP3X6X_CMD_ADR_GAINCAL_LENGTH      (3)
#define SPI_MCP3X6X_CMD_ADR_RESERVED1_LENGTH    (3)
#define SPI_MCP3X6X_CMD_ADR_RESERVED2_LENGTH    (1)
#define SPI_MCP3X6X_CMD_ADR_LOCK_LENGTH         (1)
#define SPI_MCP3X6X_CMD_ADR_RESERVED3_LENGTH    (2)
#define SPI_MCP3X6X_CMD_ADR_CRCCFG_LENGTH       (2)

// Command Type
#define SPI_MCP3X6X_CMD_TYPE_MASK               (0b00000011)
#define SPI_MCP3X6X_CMD_TYPE_BITPOS             (0)
#define SPI_MCP3X6X_CMD_TYPE_STATIC_READ        (0b01)
#define SPI_MCP3X6X_CMD_TYPE_INCREMENTAL_WRITE  (0b10)
#define SPI_MCP3X6X_CMD_TYPE_INCREMENTAL_READ   (0b11)

// Status
#define SPI_MCP3X6X_STATUS_DEV_ADR_1_MASK       (0b00100000)
#define SPI_MCP3X6X_STATUS_DEV_ADR_0_MASK       (0b00010000)
#define SPI_MCP3X6X_STATUS_DEV_ADR_N0_MASK      (0b00001000)
#define SPI_MCP3X6X_STATUS_MASK                 (0b00000111)
#define SPI_MCP3X6X_STATUS_DR_MASK              (0b00000100)
#define SPI_MCP3X6X_STATUS_CRCCFG_MASK          (0b00000010)
#define SPI_MCP3X6X_STATUS_POR_MASK             (0b00000001)

#define SPI_MCP3X6X_CONFIG0_VREF_MASK           (0b10000000)    // Datasheet Revision B
#define SPI_MCP3X6X_CONFIG0_VREF_BITPOS         (7)
#define SPI_MCP3X6X_CONFIG0_VREF_INT            (0b1)
#define SPI_MCP3X6X_CONFIG0_VREF_EXT            (0b0)

#define SPI_MCP3X6X_CONFIG0_CLK_MASK            (0b00110000)
#define SPI_MCP3X6X_CONFIG0_CLK_BITPOS          (4)
#define SPI_MCP3X6X_CONFIG0_CLK_INT_OUT         (0b11)
#define SPI_MCP3X6X_CONFIG0_CLK_INT             (0b10)
#define SPI_MCP3X6X_CONFIG0_CLK_EXT             (0b00)

#define SPI_MCP3X6X_CONFIG0_CURRENT_MASK        (0b00001100)
#define SPI_MCP3X6X_CONFIG0_CURRENT_BITPOS      (2)
#define SPI_MCP3X6X_CONFIG0_CURRENT_150UA       (0b11)
#define SPI_MCP3X6X_CONFIG0_CURRENT_037UA       (0b10)
#define SPI_MCP3X6X_CONFIG0_CURRENT_009UA       (0b01)
#define SPI_MCP3X6X_CONFIG0_CURRENT_000UA       (0b00)

#define SPI_MCP3X6X_CONFIG0_MODE_MASK           (0b00000011)
#define SPI_MCP3X6X_CONFIG0_MODE_BITPOS         (0)
#define SPI_MCP3X6X_CONFIG0_MODE_CONVERSION     (0b11)
#define SPI_MCP3X6X_CONFIG0_MODE_STANDBY        (0b10)
//#define SPI_MCP3X6X_CONFIG0_MODE_SHUTDOWN       (0b01)
#define SPI_MCP3X6X_CONFIG0_MODE_SHUTDOWN       (0b00)          // First conversion will start after 256 DMCLK

#define SPI_MCP3X6X_CONFIG1_PRESCALER_MASK      (0b11000000)
#define SPI_MCP3X6X_CONFIG1_PRESCALER_BITPOS    (6)
#define SPI_MCP3X6X_CONFIG1_PRESCALER_DIV8      (0b11)
#define SPI_MCP3X6X_CONFIG1_PRESCALER_DIV4      (0b10)
#define SPI_MCP3X6X_CONFIG1_PRESCALER_DIV2      (0b01)
#define SPI_MCP3X6X_CONFIG1_PRESCALER_DIV1      (0b00)

#define SPI_MCP3X6X_CONFIG1_OSR_MASK            (0b00111100)
#define SPI_MCP3X6X_CONFIG1_OSR_BITPOS          (2)
#define SPI_MCP3X6X_CONFIG1_OSR_98304           (0xF)
#define SPI_MCP3X6X_CONFIG1_OSR_81920           (0xE)
#define SPI_MCP3X6X_CONFIG1_OSR_49152           (0xD)
#define SPI_MCP3X6X_CONFIG1_OSR_40960           (0xC)
#define SPI_MCP3X6X_CONFIG1_OSR_24576           (0xB)
#define SPI_MCP3X6X_CONFIG1_OSR_20480           (0xA)
#define SPI_MCP3X6X_CONFIG1_OSR_16384           (0x9)
#define SPI_MCP3X6X_CONFIG1_OSR_8192            (0x8)
#define SPI_MCP3X6X_CONFIG1_OSR_4096            (0x7)
#define SPI_MCP3X6X_CONFIG1_OSR_2048            (0x6)
#define SPI_MCP3X6X_CONFIG1_OSR_1024            (0x5)
#define SPI_MCP3X6X_CONFIG1_OSR_512             (0x4)
#define SPI_MCP3X6X_CONFIG1_OSR_256             (0x3)
#define SPI_MCP3X6X_CONFIG1_OSR_128             (0x2)
#define SPI_MCP3X6X_CONFIG1_OSR_64              (0x1)
#define SPI_MCP3X6X_CONFIG1_OSR_32              (0x0)

#define SPI_MCP3X6X_CONFIG2_BOOST_MASK          (0b11000000)
#define SPI_MCP3X6X_CONFIG2_BOOST_BITPOS        (6)
#define SPI_MCP3X6X_CONFIG2_BOOST_X2            (0b11)
#define SPI_MCP3X6X_CONFIG2_BOOST_X1            (0b10)
#define SPI_MCP3X6X_CONFIG2_BOOST_X066          (0b01)
#define SPI_MCP3X6X_CONFIG2_BOOST_X05           (0b00)

#define SPI_MCP3X6X_CONFIG2_GAIN_MASK           (0b00111000)
#define SPI_MCP3X6X_CONFIG2_GAIN_BITPOS         (3)
#define SPI_MCP3X6X_CONFIG2_GAIN_X64            (0b111)
#define SPI_MCP3X6X_CONFIG2_GAIN_X32            (0b110)
#define SPI_MCP3X6X_CONFIG2_GAIN_X16            (0b101)
#define SPI_MCP3X6X_CONFIG2_GAIN_X8             (0b100)
#define SPI_MCP3X6X_CONFIG2_GAIN_X4             (0b011)
#define SPI_MCP3X6X_CONFIG2_GAIN_X2             (0b010)
#define SPI_MCP3X6X_CONFIG2_GAIN_X1             (0b001)
#define SPI_MCP3X6X_CONFIG2_GAIN_D3             (0b000)

#define SPI_MCP3X6X_CONFIG2_AZ_MUX_MASK         (0b00000100)
#define SPI_MCP3X6X_CONFIG2_AZ_REF_MASK         (0b00000010)

#define SPI_MCP3X6X_CONFIG3_CONV_MASK           (0b11000000)
#define SPI_MCP3X6X_CONFIG3_CONV_BITPOS         (6)
#define SPI_MCP3X6X_CONFIG3_CONV_CONTINUOUS     (0b11)
#define SPI_MCP3X6X_CONFIG3_CONV_ONE_STANDBY    (0b10)
//#define SPI_MCP3X6X_CONFIG3_CONV_ONE_SHUTDOWN   (0b01)
#define SPI_MCP3X6X_CONFIG3_CONV_ONE_SHUTDOWN   (0b00)

#define SPI_MCP3X6X_CONFIG3_DATA_MASK           (0b00110000)
#define SPI_MCP3X6X_CONFIG3_DATA_BITPOS         (4)
#define SPI_MCP3X6X_CONFIG3_DATA_RIGHT_CHID     (0b11)
#define SPI_MCP3X6X_CONFIG3_DATA_RIGHT          (0b10)
#define SPI_MCP3X6X_CONFIG3_DATA_LEFT           (0b01)
#define SPI_MCP3X6X_CONFIG3_DATA_ADC_CODING     (0b00)

#define SPI_MCP3X6X_CONFIG3_CRC_MASK            (0b00001000)
#define SPI_MCP3X6X_CONFIG3_CRC_BITPOS          (3)
#define SPI_MCP3X6X_CONFIG3_CRC_32              (0b1)
#define SPI_MCP3X6X_CONFIG3_CRC_16              (0b0)

#define SPI_MCP3X6X_CONFIG3_CRC_ONREAD_MASK     (0b00000100)

#define SPI_MCP3X6X_CONFIG3_OFFCAL_MASK         (0b00000010)
#define SPI_MCP3X6X_CONFIG3_GAINCAL_MASK        (0b00000001)

#define SPI_MCP3X6X_IRQ_DATA_READY_MASK         (0b01000000)
#define SPI_MCP3X6X_IRQ_CRC_CFG_ERROR_MASK      (0b00100000)
#define SPI_MCP3X6X_IRQ_POR_STATUS_MASK         (0b00010000)

#define SPI_MCP3X6X_IRQ_MODE_MASK               (0b00001100)
#define SPI_MCP3X6X_IRQ_MODE_BITPOS             (2)
#define SPI_MCP3X6X_IRQ_MODE_MDAT_INACTIVE_LH   (0b11)
#define SPI_MCP3X6X_IRQ_MODE_MDAT_INACTIVE_Z    (0b10)
#define SPI_MCP3X6X_IRQ_MODE_IRQ_INACTIVE_LH    (0b01)
#define SPI_MCP3X6X_IRQ_MODE_IRQ_INACTIVE_Z     (0b00)

#define SPI_MCP3X6X_IRQ_EN_FASTCMD_MASK         (0b00000010)
#define SPI_MCP3X6X_IRQ_EN_STP_MASK             (0b00000001)

#define SPI_MCP3X6X_MUX_VIN_P_MASK              (0b11110000)
#define SPI_MCP3X6X_MUX_VIN_P_BITPOS            (4)
#define SPI_MCP3X6X_MUX_VIN_N_MASK              (0b00001111)
#define SPI_MCP3X6X_MUX_VIN_N_BITPOS            (0)
#define SPI_MCP3X6X_MUX_VCM                     (0xF)
#define SPI_MCP3X6X_MUX_TEMPERATURE_M           (0xE)
#define SPI_MCP3X6X_MUX_TEMPERATURE_P           (0xD)
#define SPI_MCP3X6X_MUX_REFIN_M                 (0xC)
#define SPI_MCP3X6X_MUX_REFIN_P                 (0xB)
#define SPI_MCP3X6X_MUX_AVDD                    (0x9)
#define SPI_MCP3X6X_MUX_AGND                    (0x8)
#define SPI_MCP3X6X_MUX_CH7                     (0x7)
#define SPI_MCP3X6X_MUX_CH6                     (0x6)
#define SPI_MCP3X6X_MUX_CH5                     (0x5)
#define SPI_MCP3X6X_MUX_CH4                     (0x4)
#define SPI_MCP3X6X_MUX_CH3                     (0x3)
#define SPI_MCP3X6X_MUX_CH2                     (0x2)
#define SPI_MCP3X6X_MUX_CH1                     (0x1)
#define SPI_MCP3X6X_MUX_CH0                     (0x0)

#define SPI_MCP3X6X_SCAN_DELAY_MASK             (0xE00000)
#define SPI_MCP3X6X_SCAN_DELAY_BITPOS           (21)
#define SPI_MCP3X6X_SCAN_DELAY_512_DMCLK        (0b111)
#define SPI_MCP3X6X_SCAN_DELAY_256_DMCLK        (0b110)
#define SPI_MCP3X6X_SCAN_DELAY_128_DMCLK        (0b101)
#define SPI_MCP3X6X_SCAN_DELAY_64_DMCLK         (0b100)
#define SPI_MCP3X6X_SCAN_DELAY_32_DMCLK         (0b011)
#define SPI_MCP3X6X_SCAN_DELAY_16_DMCLK         (0b010)
#define SPI_MCP3X6X_SCAN_DELAY_8_DMCLK          (0b001)
#define SPI_MCP3X6X_SCAN_DELAY_0_DMCLK          (0b000)

#define SPI_MCP3X6X_SCAN_SEL_MASK               (0x00FFFF)
#define SPI_MCP3X6X_SCAN_SEL_BITPOS             (0)
#define SPI_MCP3X6X_SCAN_SEL_OFFSET             (0x8000)
#define SPI_MCP3X6X_SCAN_SEL_VCM                (0x4000)
#define SPI_MCP3X6X_SCAN_SEL_AVDD               (0x2000)
#define SPI_MCP3X6X_SCAN_SEL_TEMP               (0x1000)
#define SPI_MCP3X6X_SCAN_SEL_CH6_CH7            (0x0800)
#define SPI_MCP3X6X_SCAN_SEL_CH4_CH5            (0x0400)
#define SPI_MCP3X6X_SCAN_SEL_CH2_CH3            (0x0200)
#define SPI_MCP3X6X_SCAN_SEL_CH0_CH1            (0x0100)
#define SPI_MCP3X6X_SCAN_SEL_CH7                (0x0080)
#define SPI_MCP3X6X_SCAN_SEL_CH6                (0x0040)
#define SPI_MCP3X6X_SCAN_SEL_CH5                (0x0020)
#define SPI_MCP3X6X_SCAN_SEL_CH4                (0x0010)
#define SPI_MCP3X6X_SCAN_SEL_CH3                (0x0008)
#define SPI_MCP3X6X_SCAN_SEL_CH2                (0x0004)
#define SPI_MCP3X6X_SCAN_SEL_CH1                (0x0002)
#define SPI_MCP3X6X_SCAN_SEL_CH0                (0x0001)

#define SPI_MCP3X6X_LOCK_ACCESS_ALLOWED         (0xA5)


#endif /* _SPI_MCP3X6X_DEF_H_ */
