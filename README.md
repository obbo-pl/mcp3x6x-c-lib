# MCP3461/2/4(R) and MCP3561/2/4(R) C library

C library for MCP3461/2/4(R) and MCP3561/2/4(R) delta-sigma ADC.

The library was created for interactive reading of measurements. It allows you to configure all registers so it can also be used in other scenarios (e.g. Scan, MDAT). The library provides low-level functions. It is hardware independent, does not use delays or floating point operations. If you need, you need to do the timer values and voltage calculation yourself. 

For communication with the controller, the SPI bus can be used in full-duplex or half-duplex mode (CRC-onRead is not supported in half-duplex mode). Several ADCs can be used in one project, either on one or on different SPI buses. They can also be ICs in various variants. To communicate with the controller via SPI you need to provide two simple functions. Examples of such functions for ESP32 and RP Pico can be found in the examples folder.

Currently the library is tested with MCP3564R.
