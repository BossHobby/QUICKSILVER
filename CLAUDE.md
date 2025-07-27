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

## Task Scheduler

### Overview

The task scheduler manages execution of flight controller tasks with different priorities and timing requirements. It uses a runtime equalization system to ensure predictable task execution and prevent loop timing overruns.

### Task Priorities

Tasks are assigned one of four priority levels:
- `TASK_PRIORITY_REALTIME` - Critical control tasks (GYRO, IMU, PID, RX) that must run every loop
- `TASK_PRIORITY_HIGH` - Important tasks that should run frequently
- `TASK_PRIORITY_MEDIUM` - Regular tasks (BLACKBOX, VTXTELEM, etc.)
- `TASK_PRIORITY_LOW` - Background tasks that can be deferred

### Runtime Equalization System

The scheduler tracks task runtime to make scheduling decisions:

```c
typedef struct {
  uint32_t runtime_avg;     // Running average over 32 samples
  uint32_t runtime_worst;   // Worst-case runtime estimate
} task_runtime_t;
```

Key constants:
- `TASK_RUNTIME_REDUCTION`: 0.75x - Reduction when task is skipped
- `TASK_RUNTIME_BUFFER`: 10μs - Buffer before loop deadline

### Scheduling Algorithm

1. **Task Selection**: Tasks are checked in priority order
2. **Runtime Check**: For non-REALTIME tasks, the scheduler checks if `task->runtime_worst > time_left`
3. **Skip Decision**: If insufficient time remains, the task is skipped and its worst-case estimate is reduced
4. **Runtime Update**: After execution, worst-case is maintained at minimum 1.25x the running average

### Task Definition

Tasks are defined in `src/core/tasks.c` using the CREATE_TASK macro:
```c
CREATE_TASK("BLACKBOX", TASK_MASK_ALWAYS, TASK_PRIORITY_MEDIUM, blackbox_update, 0)
```

Parameters:
- Name: Task identifier for debugging
- Mask: When task should run (ALWAYS, ARMED, etc.)
- Priority: Scheduling priority
- Function: Task implementation
- Period: Minimum microseconds between runs (0 = every loop if time permits)

### Writing Predictable Tasks

To ensure predictable runtime:
1. **Avoid variable workloads** - Use fixed-size operations where possible
2. **Implement rate limiting** - Process fixed amounts of data per iteration
3. **Use state machines** - Break large operations into smaller steps
4. **Monitor runtime** - Check task runtime statistics during development
5. **Consider priority** - Only use REALTIME for critical control tasks

### Example: Blackbox Task

The blackbox task demonstrates good practices:
- Uses rate divider based on `profile.blackbox.sample_rate_hz`
- Processes one sample per iteration
- Has consistent workload (compress and write fixed data structure)
- Gracefully handles write failures without blocking

### Common Runtime Issues to Avoid

1. **Unbounded loops** - Never use `while(true)` or loops without fixed bounds in tasks
2. **Variable-size operations** - Limit processing to fixed chunks per iteration
3. **Blocking I/O** - Use non-blocking operations or state machines for I/O
4. **Complex calculations** - Break into smaller steps across multiple iterations
5. **Dynamic memory allocation** - Avoid malloc/free in task loops

### Task Exceptions

Some tasks are designed with different constraints:
- **USB Task** - Blocks scheduler intentionally during configuration (flight is disabled)
- **VTX Task** - Only runs on ground, blocked during flight for safety

These tasks assume the user is not flying while configuring the system.

### Tasks with Good Runtime Behavior

- **OSD Task** - Already optimized for predictable runtime
- **Blackbox Task** - Uses rate limiting and fixed workload per iteration

### Runtime Optimization Guidelines

When implementing new tasks or modifying existing ones:
1. Use state machines for operations that span multiple iterations
2. Implement per-iteration processing limits
3. Add early exit conditions when approaching time budgets
4. Use incremental processing for large operations
5. Cache results to avoid repeated calculations
6. Consider whether the task needs to run during flight

## Scheduler Performance Metrics

### Overview

The scheduler includes comprehensive runtime metrics collection to monitor task performance and optimize scheduling decisions. These metrics are essential for identifying performance bottlenecks and ensuring stable flight performance.

### Available Metrics

The scheduler tracks the following metrics for each task:

#### Core Runtime Metrics
- **current**: Most recent execution time in microseconds
- **avg**: Running average over 32 samples
- **max**: Maximum observed runtime
- **worst**: Worst-case estimate used for scheduling decisions
- **percentile_95**: Smooth 95th percentile estimate using exponential moving average

#### Variability Metrics (Debug builds only)
- **stddev**: Standard deviation of runtime
- **cv_percent**: Coefficient of Variation (stddev/avg × 100%) - measures relative variability
- **skips**: Total number of times task was skipped due to insufficient time
- **max_skips**: Maximum consecutive skips observed
- **overruns**: Number of times task exceeded its worst-case estimate

### Percentile Calculation

The scheduler uses an exponential moving average approach for smooth P95 estimation:

```c
// Track values above average as potential peaks
if (time_taken > task->runtime_avg) {
  if (time_taken > task->runtime_peak_ema) {
    // Fast upward adjustment (1/8 weight)
    task->runtime_peak_ema = ((task->runtime_peak_ema * 7) + time_taken) >> 3;
  } else {
    // Slow downward adjustment (1/32 weight)
    task->runtime_peak_ema = ((task->runtime_peak_ema * 31) + time_taken) >> 5;
  }
}
// Continuous decay to forget old peaks
task->runtime_peak_ema = (task->runtime_peak_ema * 511) >> 9;
```

This provides stable P95 estimates without the noise of traditional percentile calculations.

### Scheduling Algorithm

The scheduler uses a predictive approach based on runtime statistics:

1. **Priority-based selection**: REALTIME tasks always run, others checked in priority order
2. **Runtime prediction**: Uses `runtime_worst` (based on P95 + margin) to estimate if task will fit
3. **Skip decision**: Non-realtime tasks skipped if `runtime_worst > time_remaining`
4. **Skip penalty**: When a task is skipped, its `runtime_worst` is reduced by 25% (`* 3/4`) to increase future execution probability
5. **Adaptive margins**: Uses conservative margins during startup, transitions to P95-based prediction

#### Skip Penalty System

When a task is skipped due to insufficient time, the scheduler applies a reduction penalty:

```c
// Reduce worst-case estimate by 25% (0.75x)
task->runtime_worst = (task->runtime_worst * 3) >> 2;
```

This serves multiple purposes:
- **Prevents starvation**: Skipped tasks become more likely to run in future loops
- **Adapts to changing conditions**: Reduces estimates that may be too conservative
- **Balances throughput**: Ensures non-realtime tasks still get execution time
- **Self-correcting**: Over-pessimistic estimates naturally decay through skipping

### Interpreting Metrics

#### Coefficient of Variation (CV%)
- **< 5%**: Very stable task with consistent runtime
- **5-15%**: Normal variability for most tasks
- **> 20%**: High variability, may need optimization

#### Skip Patterns
- **Occasional skips**: Normal for non-realtime tasks under load
- **High consecutive skips**: May indicate task period too aggressive
- **No skips**: Either REALTIME priority or very light task

#### Overrun Analysis
- **Low overruns**: Good scheduling prediction accuracy
- **High overruns**: May need P95 margin adjustment or task optimization

### Optimization Strategies

Based on metrics analysis:

1. **High CV% tasks**: Consider breaking into smaller steps or reducing workload
2. **Frequent skips**: Reduce task frequency or optimize runtime
3. **High overruns**: Tasks running longer than predicted, need runtime optimization
4. **Consistent high runtime**: Consider priority adjustment or period increase

### Future Enhancements

Potential scheduler improvements being considered:

#### Hardware Timer PLL
Using hardware timers synchronized to gyro EXTI interrupts for improved timing precision:

```c
// PLL-based scheduler concept
typedef struct {
  int32_t phase_error;      // Gyro timing vs expected
  int32_t frequency_error;  // Accumulated correction
  uint32_t timer_period;    // Hardware timer period
} scheduler_pll_t;
```

Benefits:
- Eliminates busy-wait timing overhead
- Locks main loop to actual gyro sampling rate
- Reduces jitter through hardware-driven timing
- Automatically adapts to gyro timing variations

This would require gyro EXTI support and available hardware timers, but could significantly improve timing precision for supported hardware.
