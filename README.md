# MCP3461/2/4(R) and MCP3561/2/4(R) C library

C library for MCP3461/2/4(R) and MCP3561/2/4(R) delta-sigma ADC.

The library was created for interactive reading of measurements. It allows you to configure all registers so it can also be used in other scenarios (e.g. Scan, MDAT). The library provides low-level functions. It is hardware independent, does not use delays or floating point operations. If you need, you need to do the timer values and voltage calculation yourself. 

For communication with the controller, the SPI bus can be used in full-duplex or half-duplex mode (CRC-onRead is not supported in half-duplex mode). Several ADCs can be used in one project, either on one or on different SPI buses. They can also be ICs in various variants. To communicate with the controller via SPI you need to provide two simple functions. Examples of such functions for ESP32 and RP Pico can be found in the examples folder.

If you need to quickly start testing with platforms other than the examples, take a look at HAL for Arduino. I used software emulated SPI. I think it can be easily converted and should work with most platforms. Of course, using native SPI will usually be a better choice.

Currently the library is tested with MCP3564R.

### Initialization
You can do this way:
```
spi_mcp3x6x_t* adc;
adc = spi_mcp3x6x_Create();
spi_mcp3x6x_Init(adc,
                 SPI_MCP3X6X_VARIANT_MCP3564R,
                 SPI_MCP3X6X_DEVICE_ADDRESS_DEFAULT,
                 spi_bus_hal_ReadReg,
                 spi_bus_hal_WriteReg,
                 true,
                 0);
...

...
pi_mcp3x6x_Destroy(adc);
```
or like this:
```
spi_mcp3x6x_t adc;
spi_mcp3x6x_Init(&adc,
                 SPI_MCP3X6X_VARIANT_MCP3564R,
                 SPI_MCP3X6X_DEVICE_ADDRESS_DEFAULT,
                 spi_bus_hal_ReadReg,
                 spi_bus_hal_WriteReg,
                 true,
                 0);

```

At this stage, you need to provide two hardware abstraction layer (HAL) functions.

```
/**
 * Sends a command or sets a register address and reads data from an SPI device.
 * 
 * @param cmd_wr A pointer to a buffer containing a command or register address.
 * @param cmd_rd A pointer to the buffer for the data received in the command phase. For SPI in half-duplex mode, it can be NULL.
 * @param cmd_size The length of the command or register address.
 * @param data_wr A pointer to the buffer for the data to be sent. For SPI in half-duplex mode, it can be NULL.
 * @param data_rd A pointer to a buffer for the received data.
 * @param data_size Expected length of received data.
 * @return Function execution status code.
*/
int spi_bus_hal_ReadReg(uint8_t* cmd_wr, uint8_t* cmd_rd, size_t cmd_size, uint8_t* data_wr, uint8_t* data_rd, size_t data_size);
```
```
/**
 * Sends a command or sets a register address and sends data to the SPI device.
 * 
 * @param cmd_wr A pointer to a buffer containing a command or register address.
 * @param cmd_rd A pointer to the buffer for the data received in the command phase. For SPI in half-duplex mode, it can be NULL.
 * @param cmd_size The length of the command or register address.
 * @param data_wr A pointer to the buffer for the data to be sent.
 * @param data_rd A pointer to a buffer for the received data. For SPI in half-duplex mode, it can be NULL.
 * @param data_size The length of the data to be sent.
 * @return Function execution status code.
*/
int spi_bus_hal_WriteReg(uint8_t* cmd_wr, uint8_t* cmd_rd, size_t cmd_size, uint8_t* data_wr, uint8_t* data_rd, size_t data_size);
```
Both functions should first send the address of the register on which the operation is to be performed, then, depending on the function, send data to or from the register. Throughout the operation, the CS signal should be active.

### Error codes

When initializing the driver, the code of a correctly performed SPI read or write operation is passed as a parameter.
The library functions use it to know if the communication ended correctly. Often this code is set to zero, but when creating your own HAL functions you may decide to use a different value.

The error codes returned by the library default to values from one upwards. If you want to avoid the codes overlapping with others already defined, you can move them into free space by setting
SPI_MCP3X6X_ERR_OFFSET in the header file or before calling it.

### Buffer alignment

32-bit systems may often require a word-length-aligned buffer for DMA operations.
If you're using an 8-bit controller, alignment won't be necessary, and you can save a few bytes of memory by disabling it. Uncomment the appropriate section of the SPI_MCP3X6X_ALIGNED_SPI_BUFFER macro in the header file.

32-bit alignment is enabled by default in the library.
