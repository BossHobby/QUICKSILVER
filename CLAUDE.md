# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

- Build all targets: `pio run`
- Build specific target: `pio run -e stm32f405` (other targets: stm32f411, stm32f722, stm32f745, stm32f765, stm32g473, stm32h743, at32f435, simulator)
- Clean: `pio run -t clean`
- run native tests: `pio test --environment test_native`
- Verify build across all main targets: `pio run -e stm32f405 -e stm32f411 -e stm32f745 -e stm32f765 -e stm32f722 -e stm32h743 -e stm32g473 -e at32f435 -e at32f435m -e test_native`

## Code Style Guidelines

- Includes: group by standard libraries first, then project modules
- Indentation: 2 spaces
- Braces: opening brace on same line for functions and control structures
- Function naming: snake_case
- Variable naming: snake_case
- Constants/Macros: UPPER_CASE
- Error handling: use failloop.h for critical errors
- Comments: document non-obvious behavior and complex algorithms
- Type safety: use appropriate typedefs (uint8_t, etc.) for hardware registers
- Hardware access: use appropriate driver abstraction layers
- Memory sections: respect DMA and FAST RAM sections where specified
- Use const whenever possible

## Test System

### Overview

The project uses Unity test framework integrated with PlatformIO for unit testing. Tests are written for native platform execution, allowing rapid testing without hardware.

### Test Commands

- Run all tests: `pio test --environment test_native`
- Run with verbose output: `pio test --environment test_native -v`
- Run specific test filter: `pio test --environment test_native --filter="test_filter"`

### Test Structure

Tests are located in `test/test_native/`:

- `test_main.c` - Main test runner that registers and executes all tests
- `test_<module>.c` - Individual test modules for different components
- `mock_helpers.h/.c` - Mock hardware functions for testing

### Test Organization

Each test module follows this pattern:

1. Include Unity: `#include <unity.h>`
2. Include mock helpers: `#include "mock_helpers.h"`
3. Include module under test
4. Define setUp/tearDown if needed (local to module)
5. Write test functions prefixed with `test_`
6. Export test functions via `extern` declarations in `test_main.c`
7. Register tests with `RUN_TEST()` in `test_main.c`

### Unity Assertions

Common Unity assertions used:

- `TEST_ASSERT_EQUAL_FLOAT(expected, actual)` - Float equality
- `TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual)` - Float within tolerance
- `TEST_ASSERT_NOT_NULL(pointer)` - Pointer not null
- `TEST_ASSERT_TRUE(condition)` - Boolean true
- `TEST_ASSERT_FALSE(condition)` - Boolean false
- `TEST_ASSERT_EQUAL_INT(expected, actual)` - Integer equality
- `TEST_ASSERT_EQUAL_MEMORY(expected, actual, length)` - Memory block comparison

### PlatformIO Test Configuration

The `test_native` environment in `platformio.ini`:

```ini
[env:test_native]
extends = common
board = SIMULATOR
platform = native
test_build_src = true
test_filter = test_native
debug_test = test_native
debug_tool = custom
build_src_filter = ${common.build_src_filter} +<driver/mcu/native> +<system/native>
build_flags =
  ${common.build_flags}
  -lm
  -DSIMULATOR
  -Isrc/system/native
```

### Test Coverage

Current test modules include:

- **Filter tests** - DSP filter initialization, low-pass filtering, frequency attenuation
- **PID tests** - Proportional/integral/derivative control, voltage compensation
- **IMU tests** - Gravity vector, gyro integration, accelerometer fusion, attitude calculation
- **Vector tests** - 3D/4D vector operations, rotations, magnitude calculations
- **CRC tests** - Checksum calculations and verification
- **Ring buffer tests** - Circular buffer operations, wraparound, multi-read/write
- **SPI tests** - SPI initialization, DMA transfer simulation, transaction queuing
- **ADC tests** - ADC reading, temperature, voltage, and current measurements
- **Serial tests** - Serial port configuration, data transmission/reception, buffer management

### Writing New Tests

1. Create new test file: `test/test_native/test_mymodule.c`
2. Include Unity and necessary headers
3. Write test functions with descriptive names
4. Add extern declarations to `test_main.c`
5. Register tests with `RUN_TEST()` in `test_main.c`
6. Use mock_helpers to isolate hardware dependencies
7. Run with `pio test --environment test_native`

### Test Best Practices

- Each test should be independent and not rely on test order
- Use setUp/tearDown to reset state between tests
- Mock hardware dependencies using mock_helpers
- Use descriptive test function names that explain what is tested
- Keep tests focused on single functionality
- Use appropriate Unity assertions for the data type being tested
- Add tolerance when comparing floating-point values

### Serial Testing

- **USE_SERIAL macro is now enabled** for simulator builds to exercise more code paths
- Serial mock implementation exists in `src/driver/mcu/native/serial.c`
- Ring buffer implementation used for simulating serial hardware
- Virtual pins are used instead of actual GPIO pins for simulator
- When testing serial functionality:
  - Use SERIAL_PORT1 instead of 0 (which is SERIAL_PORT_INVALID)
  - Initialize serial ports correctly with mock_serial_init()
  - Check for NULL pointers before using ring buffers

## Feature Macros (src/config/feature.h)

The project uses feature macros to control conditional compilation for different hardware targets and build configurations.

### Primary Control Macro: SIMULATOR

- When defined (via `-DSIMULATOR` build flag), builds for native/test environment
- When undefined, builds for actual hardware with all peripherals enabled
- Controlled by PlatformIO environment configuration

### Feature Categories

#### Core Hardware Features (disabled in SIMULATOR builds)

- `USE_ADC` - Analog-to-digital converter support
- `USE_SPI` - SPI bus interface
- `USE_SERIAL` - Hardware UART/USART serial ports
- `USE_GYRO` - Gyroscope sensor support
- `USE_SOFT_SERIAL` - Software-emulated serial port

#### Storage Features (disabled in SIMULATOR builds)

- `USE_SDCARD` - SD card support for blackbox logging
- `USE_DATA_FLASH` - Onboard flash memory for blackbox

#### Motor Control (disabled in SIMULATOR builds)

- `USE_MOTOR_DSHOT` - Digital shot protocol for brushless motors
- `USE_MOTOR_PWM` - PWM control for brushed motors

#### Video/OSD Features (disabled in SIMULATOR builds)

- `USE_VTX` - Video transmitter control
- `USE_DIGITAL_VTX` - Digital VTX protocols
- `USE_MAX7456` - OSD chip support
- `USE_RGB_LED` - Addressable RGB LED support

#### Receiver Features (disabled in SIMULATOR builds)

- `USE_RX_UNIFIED` - Serial receiver protocols (CRSF, SBUS, IBUS, DSM)
- `USE_RX_SPI_FRSKY` - FrSky SPI receivers (not on AT32F4)
- `USE_RX_SPI_FLYSKY` - FlySky SPI receivers (not on AT32F4)
- `USE_RX_SPI_EXPRESS_LRS` - ExpressLRS SPI receivers (not on AT32F4)

#### Always-Enabled Features (available in all builds including SIMULATOR)

- `USE_SERIAL` - Serial port functionality (required for CRSF and other serial protocols)
- `USE_RX_UNIFIED` - Unified serial receiver support (CRSF, SBUS, IBUS, DSM)
- `USE_BLACKBOX` - Flight data logging

### Usage Patterns

```c
// Simple feature check
#ifdef USE_VTX
  // VTX-specific code
#endif

// Feature alternatives
#ifdef USE_MOTOR_DSHOT
  motor_dshot_init();
#else
  #ifdef USE_MOTOR_PWM
    motor_pwm_init();
  #endif
#endif

// Platform-specific exclusions
#ifndef AT32F4
  #define USE_RX_SPI_FRSKY
#endif
```

### Dependencies

- SPI receivers require `USE_SPI`
- VTX features require `USE_SERIAL`
- Blackbox can use either `USE_SDCARD` or `USE_DATA_FLASH`
- Motor control requires either `USE_MOTOR_DSHOT` or `USE_MOTOR_PWM`

### Testing Considerations

When writing tests:

- Mock implementations should respect these feature flags
- Test builds use `-DSIMULATOR` which disables most hardware features
- Include guards should follow the same pattern as production code
- Feature-specific tests should be wrapped in appropriate #ifdef blocks

## Hardware Feature Implementation Guidelines (Simulator/Native)

When enabling hardware features (ADC, SPI, etc.) for the simulator/native platform:

### Header Organization

- Keep native headers minimal to avoid circular dependencies
- Native headers should only contain platform-specific constants and types
- Avoid including system headers from native headers
- Use `#pragma once` for include guards
- Example structure:

  ```c
  // src/driver/mcu/native/adc.h
  #pragma once

  #define VREFINT_CAL (1489)
  #define VREFINT_CAL_VREF (3300)
  ```

### Common Variables

- Use `extern` declarations in native implementation files for shared variables
- Never define variables in native code (they should be in common driver code)
- Example:
  ```c
  // src/driver/mcu/native/adc.c
  extern uint16_t adc_array[ADC_CHAN_MAX];
  extern adc_channel_t adc_pins[ADC_CHAN_MAX];
  ```

### Test Files

- Add `extern` declarations for test functions at the top of test files
- Do not include conditional compilation (#ifdef USE_ADC) in test files
- The test environment always has access to all features
- Example:

  ```c
  // test/test_native/test_adc.c
  extern void adc_set_raw_value(adc_chan_t chan, uint16_t value);

  void test_adc_read_raw() {
    adc_set_raw_value(ADC_CHAN_VBAT, 3000);
    TEST_ASSERT_EQUAL_INT(3000, adc_read_raw(ADC_CHAN_VBAT));
  }
  ```

### Implementation Patterns

1. Create stub implementations that satisfy linker requirements
2. Provide reasonable default values for simulated hardware
3. Implement minimal functionality needed for tests
4. Use static state variables in native implementations
5. Add helper functions for test manipulation (set_raw_value, etc.)

### Common Pitfalls to Avoid

- Don't include system.h in native headers (causes circular dependencies)
- Don't define common variables in native code (use extern)
- Don't wrap test files in feature macros
- Don't create complex dependencies between native implementations
- Don't duplicate setUp/tearDown functions across test files

## Serial Testing Guidelines

When implementing tests for hardware features like serial:

### Ring Buffer Management

- Serial ports require initialized ring buffers for rx_buffer and tx_buffer
- Create static ring buffer data arrays in test files
- Initialize ring_buffer_t structures with proper data, head, tail, and size
- Clear ring buffers between tests to ensure test isolation

### Test Structure

```c
// Create ring buffers for testing
static uint8_t rx_buffer_data[512];
static uint8_t tx_buffer_data[512];
static ring_buffer_t rx_buffer = {
    .buffer = rx_buffer_data,
    .head = 0,
    .tail = 0,
    .size = sizeof(rx_buffer_data),
};

// Initialize port with buffers
serial_port_t port = {
    .rx_buffer = &rx_buffer,
    .tx_buffer = &tx_buffer,
    .tx_done = true,
};
```

### Validation Considerations

- Native/simulator implementations may not pass hardware validation checks
- Target device validation (target_serial_port_valid) may fail in test environment
- Focus tests on functionality that can be exercised in simulator
- Test edge cases and error conditions that don't require hardware
