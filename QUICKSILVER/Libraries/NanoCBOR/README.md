# NanoCBOR

NanoCBOR is a tiny [CBOR](https://tools.ietf.org/html/rfc7049) library aimed at embedded and heavily constrained devices.
It is optimized for 32 bit architectures but should run fine on 8 bit and 16 bit architectures.

The decoder of NanoCBOR should compile to 600-800 bytes on a Cortex-M0+ MCU.

### Dependencies:

Only dependency are two functions to provide endian conversion.
These are not provided by the library and have to be configured in the header file.

### Contributing

Open an issue, PR, the usual.

