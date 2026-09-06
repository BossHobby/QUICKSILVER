# AGENTS.md

This file provides repository guidance for coding agents working in this repository.

## Build Commands
- Build all targets: `pio run`
- Build specific target: `pio run -e stm32f405` (other targets: stm32f411, stm32f722, stm32f745, stm32f765, stm32g473, stm32h743, at32f435, simulator)
- Clean: `pio run -t clean`

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

## Architecture Overview

QUICKSILVER is FPV drone flight controller firmware supporting STM32 F4/F7/H7/G4 and AT32 F435 MCUs.

### Shared Runtime State (Required Pattern)

Subsystems publish their externally useful runtime values into the global `state` (`control_state_t` in `src/control/control.h`, defined in `src/control/control.cpp`). This is a shared blackboard: despite its name, it is the vehicle's common runtime data interface. The companion `flags` holds shared control/status flags. Consumers read published values without depending on the producer's implementation.

Keep subsystem machinery private. For example, battery filter history stays local while voltage/current readings are published; PID history and integration bookkeeping stay local while P/I/D terms and output are published; GPS publishes decoded position, speed and fix information for consumers such as telemetry.

When adding or changing subsystem behavior:

- Publish runtime values needed by other subsystems or diagnostics through `state` (or `flags` for shared control/status flags). Reuse existing fields when their meaning matches; do not introduce parallel public globals or trivial getters for the same values.
- Keep filter histories, parser buffers, counters used only internally, and algorithm bookkeeping local to the owning subsystem. Do not put private implementation details into `state` merely for convenience.
- Give each published value a clear owning producer. Consumers should treat it as read-only. Document intentional alternate writers, such as simulator input or override paths, and the conditions under which they own the value.
- Document units, coordinate frame or range where relevant, initialization/reset behavior, and validity semantics. Distinguish measurements, requested commands and applied outputs; do not silently repurpose a field between these roles.
- Treat `state` as the latest available values, not a synchronized snapshot. Check producer/consumer order and task cadence in `src/core/tasks.cpp` and `src/core/scheduler.cpp`. Use validity/freshness information when a consumer requires it; do not assume a value was updated this loop. Any ISR or concurrent access needs an explicit synchronization decision.
- Treat serialized state as an external interface. For additions or changes, review `STATE_MEMBERS`, QUIC encoding, and affected Configurator, telemetry and blackbox consumers. Update serialization deliberately and check payload capacity when expanding it; preserve existing field meanings unless the interface change is intentional.
- During review, trace the changed fields' writers and readers and verify ownership, initialization, validity and scheduling assumptions. Preserve this pattern without adding a generic event bus, accessor layer or broad state refactor unless the task requires one.

### Core Modules (`src/core/`)
- `main.c` - Entry point, hardware init sequence, starts scheduler
- `scheduler.c` - Priority-based task scheduler (REALTIME > HIGH > MEDIUM > LOW)
- `tasks.c` - Task definitions with runtime tracking (TASK_GYRO, TASK_PID, TASK_RX, etc.)
- `flash.c` - Configuration persistence to flash memory
- `profile.c` - Runtime configuration/profile management
- `failloop.c` - Critical error handling (halts system with error code)
- `project.h` - Memory section macros (FAST_RAM, DMA_RAM)

### Flight Control (`src/flight/`)
- `control.c` - Main control loop, mode handling (acro/angle), arming logic
- `pid.c` - PID controller with D-term filtering and voltage compensation
- `imu.c` - 6-axis sensor fusion, angle calculation
- `sixaxis.c` - Gyro/accel reading and calibration
- `filter.c` - Digital filters (PT1, dynamic filtering, SDFT)
- `input.c` - Receiver input processing and expo curves

### Hardware Drivers (`src/driver/`)
- `mcu/` - MCU-specific implementations (stm32/, at32/, native/)
- `gyro/` - Gyroscope drivers (BMI270, etc.)
- Motor: `motor.c`, `motor_dshot.c` - DShot protocol implementation
- Communication: `spi.c`, `serial.c`, `usb.c`

### I/O Protocols (`src/io/`)
- `msp.c` - MSP protocol for configurator communication
- `quic.c` - Custom QUIC protocol for telemetry
- `vtx.c` - VTX control abstraction (SmartAudio, TrampHV, MSP)
- `blackbox.c` - Flight data logging

### Receiver Support (`src/rx/`)
- Protocol implementations: CRSF/ELRS, FrSky, FlySky, DSM, SBUS, IBUS
- `rx.c` - Receiver abstraction layer
- `rx_spi.c` - SPI-based receiver interface

### OSD (`src/osd/`)
- `render.c` - OSD rendering engine
- `menu.c` - Configuration menu system

## Memory Sections

Use these macros for performance-critical or DMA-accessible data:
- `FAST_RAM` - Places variables in fast-access RAM for real-time code
- `DMA_RAM` - Places buffers in DMA-accessible memory regions

## Task Scheduler

Tasks are defined in `src/core/tasks.c` with priorities and context masks:
- `TASK_MASK_ON_GROUND` - Runs only when disarmed
- `TASK_MASK_IN_AIR` - Runs only when armed
- `TASK_MASK_ALWAYS` - Runs in all states

REALTIME tasks (gyro, imu, pid, rx) run every loop iteration. Lower priority tasks are scheduled based on available CPU time.

## Target Configuration

Board configurations are in `targets/*.yaml`. Each defines:
- MCU type and pin mappings
- Gyro SPI bus and CS pin
- Motor timer assignments
- Serial port mappings
- LED, buzzer, and peripheral configs

The `targets/_index.json` contains the master board index.
