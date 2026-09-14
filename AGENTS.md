# Repository guidance

QUICKSILVER is flight-controller firmware for STM32 F4/F7/G4/H7 and AT32 F435, with multirotor, rover and wing builds.

## Approach

- Trace callers, initialization order, task priorities and state writers before editing. Establish which conditions are reachable; do not add guards, startup handshakes or tests for hypothetical states the application cannot enter.
- Keep changes focused. Moving USB into a thread does not authorize changing serial reads, simulator polling, protocol parsing or fault handling. Address adjacent issues separately unless they block the requested change.
- Follow existing subsystem structure before inventing abstractions. Avoid trivial wrappers, redundant state, speculative counters and APIs with empty implementations added just to hide a conditional.
- Comments should explain actual ownership, timing and invariants. Do not justify code with a scenario contradicted by startup or arming rules.
- Use `rg` for searches and `apply_patch` for edits, not Python or shell replacement scripts merely to modify source text.

## Style and organization

- Use 2-space indentation, same-line opening braces, `snake_case` functions/variables, `UPPER_CASE` constants/macros, `const` where possible and fixed-width hardware types.
- Simple single-statement guards may omit braces; use braces when they clarify branches. Prefer named intermediate values to awkward line wrapping.
- Follow neighboring include conventions: standard libraries before project modules, with the owning header first where established.
- Order functions from supporting operations toward orchestration. Keep helpers near and before their callers, initialization before the update/service function, and the thread entry loop last. USB order: transport/protocol helpers, `usb_configurator()`, `usb_configurator_thread()`.
- Do not group new public functions or thread entries at the top merely because they are public. Avoid forward declarations introduced only to invert the established order.
- Headers generally contain includes, constants, types, extern variables, then functions. Keep type-dependent macros beside their types and implementation-only constants in the source file.
- In source files, put file-scope state declarations (including synchronization storage and handles) with the other state near the top, after includes/constants/types and before functions. Do not insert them between function definitions beside the function that initializes them.
- Keep internals private. Do not change production linkage with constructs such as `#ifndef PIO_UNIT_TESTING static #endif` to expose them to tests.
- Use existing driver boundaries and `failloop.h` for critical errors. Respect `FAST_RAM`/`DMA_RAM`; check stack, heap and DMA accessibility when adding threads or buffers.

## Commits

- Use `subsystem: short imperative summary`. Check recent history for component terminology rather than generic Conventional Commit types.
- Lowercase subsystem and initial imperative verb; preserve proper names/acronyms and omit the trailing period.
- Prefer a subject-only commit. Add a brief body only for a reason or non-obvious constraint, not routine summaries or test logs.
- Example: `blackbox: move storage into a dedicated FreeRTOS thread`.

## Build and test

Environment names include the vehicle prefix. Current `platformio.ini` and generated `targets/*.ini` are authoritative.

- Default build: `pio run`; clean: `pio run -t clean`.
- MCU build: `pio run -e multi-stm32g473`; board build: `pio run -e multi-befh-betafpvg473_v2`.
- Main MCU coverage: `pio run -e multi-stm32f405 -e multi-stm32f411 -e multi-stm32f745 -e multi-stm32f765 -e multi-stm32f722 -e multi-stm32h743 -e multi-stm32g473 -e multi-at32f435 -e multi-at32f435m`.
- Native tests: `pio test -e multi-test -e rover-test -e wing-test`. Append `--filter test_common` for shared suites or select e.g. `pio test -e wing-test --filter test_wing`. Add `-v` for diagnostics.
- Simulators: `multi-simulator`, `rover-simulator`, `wing-simulator`.
- Test meaningful behavior and reachable transitions, not implementation details. Run checks appropriate to the change; distinguish build/test evidence from hardware validation. Report and investigate intermittent failures rather than silently rerunning until green.

## Startup and scheduling

- `main.cpp` owns Flight startup, timer notifications and loop orchestration. `main()` initializes GPIO, interrupts and time, creates Flight, then starts FreeRTOS. Flight initializes peripherals with the scheduler running, then starts its pacing timer.
- `tasks.cpp` owns static thread definitions/stacks and the IO worker. Subsystem initialization prepares devices and synchronization objects, then calls `thread_start()`. Current threads are created during boot, before arming; `thread_start()` only creates them.
- `main.cpp` owns Flight loop accounting and rate fallback. Native timer behavior stays in `src/driver/mcu/native/timer.cpp`; Flight uses the same timer API and notification wait on all platforms.
- Flight applies thread context masks after control resolves arming. Ground means neither `flags.arm_state` nor `flags.in_air`. Excluded threads remain suspended despite delays or notifications. Flight, Blackbox, OSD and IO run always; USB is ground-only. OSD owns DisplayPort receive/dispatch as well as rendering and transport; do not leave its service work in Flight.
- IO owns RX transport, battery, barometer, utility outputs, GPS and VTX when DisplayPort is absent. With a configured DisplayPort UART, OSD owns VTX service too, so that UART has one sending task; initialize VTX state before starting OSD. Routine IO passes do not take `profile_mutex`. IO yields between passes and waits until a notification or the earliest service deadline; every update gates itself and returns its next deadline in ticks. Serial RX data, circular-DMA IDLE line interrupts and soft-serial bytes wake it via `serial_rx_notify_from_isr`. SPI RX radios keep a fast poll (no EXTI); unified serial autodetect reports its switch timer while cycling. Barometer samples at 10 ms, waking on I2C transfer completion with a short poll only while the device converts; GPS reports a coarse backstop, and VTX remains ground-only. Flight retains RX conditioning, control, Blackbox capture and navigation. RX decoders stage channels privately, IO publishes complete frames to a coalescing mailbox, and Flight owns `state.rx_channels`.
- Flight directly runs sensor acquisition, IMU, RX conditioning, control and Blackbox capture every pass, then calls `nav_update()`. Shared navigation owns its 10 ms cadence and derives altitude/vertical speed from the barometer mailbox on all vehicles, independently of GPS. Multirotor RTH runs at that cadence when GPS is configured. Delayed updates coalesce; there is no cooperative task queue or budget-based admission.
- IO notifications use `eSetBits` to select RX, GPS or VTX from the receiving serial port, and barometer work from I2C completion. Each service deadline sets that service's work bit; check deadlines on every wake so traffic cannot postpone periodic sampling. Serial RX drains queued frames until incomplete input or its finite attempt limit, publishing the latest channels for Flight.
- Keep flight work bounded and non-blocking: no peripheral/configuration mutex waits, unbounded service loops or dynamic allocation. Worker entry loops may run indefinitely but must wait or delay between passes. Ground maintenance may block.

## Continuing the FreeRTOS migration

- Move one subsystem at a time. Identify the bounded Flight-side capture/publication step, worker-owned state, configuration callers and shared peripherals before moving its service loop. Avoid servicing it from both Flight and a worker.
- Read `src/FreeRTOSConfig.h` and `script/freertos.py` before changing kernel assumptions. Flight has priority 2, workers priority 1, with time slicing disabled. A ready worker can starve its peers; use an actual wait/delay between passes, and choose notification coalescing, polling or queued delivery deliberately.
- Preserve the early scheduler start. Creating FreeRTOS synchronization objects before starting it previously left BASEPRI masking DMA interrupts, stalling gyro calibration on G4. Peripheral initialization therefore runs inside Flight after FreeRTOS starts.
- Audit preemption boundaries in shared drivers, not just subsystem variables. In `spi_txn_continue_port()`, claiming a transaction through launching DMA is atomic so Flight cannot preempt the worker and wait for an operation it has not started. Keep critical sections short; never wait for hardware while masking its completion interrupt. ISR kernel calls must use the `FromISR` API and obey the configured interrupt-priority ceiling.
- Audit shared parser/encoding buffers, driver pools and libc allocation when adding another concurrent caller. The USB/Flight configuration mutex does not protect unrelated workers, and `configUSE_NEWLIB_REENTRANT` is disabled. Do not assume `volatile`, separate task stacks or heap allocation provide synchronization.
- Size stacks from compiler `.su` reports (including LTO reports beside the ELF), complete call paths and saved context, then measure on hardware. Flight has 4 KiB in fast RAM, OSD and IO have 4 KiB each, and Blackbox and USB have 2 KiB each. Keep large maintenance buffers on the heap with explicit lifetimes. Native FreeRTOS uses pthread stacks, so passing native tests does not validate Cortex-M stack reservations; account for the separate linker-reserved interrupt stack too.
- USB and other maintenance callers use `flight_reset_runtime()` because long commands can hold the configuration mutex and delay Flight. Flight consumes this reset to discard the affected timing sample and reset its runtime average; removing it can trigger a false 20 ms loop-time fault.
- Use `test/test_common/test_flight.cpp` and `test_blackbox.cpp` as examples for real FreeRTOS tests of preemption, pending work, mask transitions and storage stalls. Hardware follow-up remains USB connect/save/download, arm/disarm transitions, and stack high-water measurements under the heavy paths. Builds and native tests alone are not evidence that those hardware checks passed.

## Configuration, arming and synchronization

- `control_update_arming()` blocks normal arming while USB is active. An arm request during USB activity latches the arm-switch disable; unplugging does not permit arming without lowering the switch. New arming also needs cleared latches, valid prearm, safe throttle and no failsafe. Motor testing is a separate output override.
- Distinguish the arming gate from synchronization. `profile_mutex`, owned and initialized by `core/profile.cpp`, serializes disarmed Flight with complete USB commands, MSP maintenance, binding, OSD edits/ground rendering and VTX setup/application. Receive complete commands before taking it; channel parsing, peripheral polling and airborne OSD telemetry do not take it. Check arming again after a maintenance lock wait. USB can disconnect before a command finishes. Keep checked RX indices and divisors in local values while ground configuration changes; Flight retains coherent configuration ownership for control.
- Apply thread masks before releasing ground configuration ownership; do not suspend a worker holding a mutex another task needs. Prefer scoped ownership to manual take/give blocks.
- Use mailboxes/snapshots when a consumer needs coherent samples, as Blackbox does for Flight captures. Display-only OSD telemetry deliberately reads latest scalar values from different Flight passes; do not add a mailbox, aggregate telemetry type or full-state copy for frame consistency. Keep checked table indices in local values when they must remain stable between validation and use. Do not introduce request/response machinery for synchronous ground-only operations.
- Enforce ground-only maintenance at command entry points. Acquire the storage mutex once per complete operation and once per worker service pass; lower-level device helpers stay lock-free under that ownership. Avoid nested/recursive locking.
- Keep synchronization machinery with its owner. Share a handle only when another owner must acquire it, as Flight does for USB configuration.
- Preserve `usb_configurator(true)` for the fault loop: its explicit fault-mode argument bypasses command/storage mutex waits because the scheduler is suspended. Normal USB uses the default argument; do not infer fault mode from scheduler-state checks throughout normal subsystem code. Tests initialize their mutexes instead of bypassing synchronization. A fault-mode redesign that keeps only USB alive is separate future work.

## Shared runtime state

- Global `state` (`control_state_t` in `src/control/control.h`, defined in `control.cpp`) is the common runtime interface; `flags` holds shared control/status flags. Publish externally useful values here, reusing matching fields instead of parallel globals or trivial getters.
- Keep filter histories, parser buffers, private counters and algorithm bookkeeping local. Add diagnostics only for a concrete consumer/debugging requirement.
- Give each field an owning producer; consumers treat it as read-only. Document alternate writers such as simulator/override paths and when they own it.
- Document units, frames/ranges, initialization/reset and validity where relevant. Keep measurements, requested commands and applied outputs distinct.
- `state` contains latest values, not a synchronized snapshot. Trace writers/readers and cadence; use freshness/validity when needed. ISR and concurrent access require an explicit synchronization decision.
- Serialization is an external interface. Review `STATE_MEMBERS`, QUIC encoding, payload capacity and affected Configurator, telemetry and Blackbox consumers when changing fields. Preserve meanings deliberately; do not add an incidental event bus or broad state refactor.

## Profiles, targets and vehicles

- Advance `PROFILE_VERSION` once after the previous version has been publicly released. Extend the current unreleased version's changelog for further changes; check public release history before bumping.
- Configurator migrations cover publicly released formats only. Do not add migration adapters/default filling for intermediate development artifacts; update current schema and normal defaults directly.
- Do not add on-device profile migrations. Changed persisted layouts are reset or rewritten off-device.
- Vehicle selection is compile-time: `VEHICLE_MULTI`, `VEHICLE_ROVER` or `VEHICLE_WING`. Target YAML `vehicles` lists capabilities, not selection; absent capabilities default to multirotor. `target_init()` rejects incompatible vehicles.
- Target field changes must align across **BossHobby/Targets** schema (`src/schema/target.json`), types (`src/types.ts`) and generated keys; firmware `target_t`/`TARGET_MEMBERS` in `src/core/target.h`; and **BossHobby/Configurator** types/UI.
- Build scripts consume generated Targets YAML/index files and inject CBOR into `.config_flash`; firmware decodes it into `target_t`. Inspect the scripts for current generation/injection details.
- Targets `src/index.ts` generates `output/` YAML and indexes; CI publishes the generated branches. Firmware `script/pre_script.py` fetches these into `targets/`; `TARGETS_BRANCH` overrides branch selection. Use `SKIP_TARGETS_CHECKOUT=1` when intentionally building against an existing local target checkout.
- `targets/_index.json` indexes boards and `targets/_index.ini` supplies board environments. `script/target_inject.py` copies the vehicle/MCU ELF for board injection and writes target CBOR into `.config_flash`; firmware decodes it into `target_t`. `TARGET_HASH` is the YAML's MD5. Check `script/post_script.py` for build/injection wiring.
- Outputs are routed through `profile.mixer`, `profile.outputs` and `target.outputs`; trace `src/control/output.cpp` rather than assuming fixed motor/servo slots. PWM uses `src/driver/servo.cpp` and MCU implementations, with `TIMER_USE_SERVO` allocation. Normalized values [-1, +1] map to 1000–2000 µs pulses; `profile.servo.pwm_rate_hz` must be 50–333 Hz.
- Configurator uses **QUIC**, with CBOR payloads over WebSerial at 921600 baud. Protocol definitions are in Configurator `src/store/serial/quic.ts`; firmware handling is `src/io/quic.cpp`. Do not assume it uses MSP because USB also supports MSP clients.

## Native tests and drivers

- Tests use Unity. Vehicle test environments inherit their simulators and matching controller sources. Shared mocks live under `test/`; suites are `test_common`, `test_pid`, `test_navigation` (multi), `test_rover` and `test_wing`.
- PlatformIO discovers suites by the `test_` directory prefix; environment names use hyphens. Common settings live in `[native-test]`. Tests use Unity's entry point; simulators run the firmware entry point. Reuse `test/mock_helpers.h/.cpp` and `test/mock_outputs.h/.cpp`.
- Add `test_*` functions to the appropriate `.cpp` suite, declare them in its `test_main.cpp`, and register with `RUN_TEST()`. Reset shared state, avoid duplicate suite setup/teardown, and use floating-point tolerances.
- `src/config/feature.h` is authoritative. Native builds enable SPI, serial, ADC, GPS, servos, unified RX and Blackbox; do not assume `SIMULATOR` disables all hardware features or wrap tests in redundant feature guards.
- Keep native headers minimal with `#pragma once`; avoid `system.h` and circular platform dependencies. Define common driver globals once in common code and reference them from native implementations. Native-only bookkeeping stays private.
- Serial tests need a valid port such as `SERIAL_PORT1` (zero is invalid), initialized RX/TX ring buffers and reset indices. Use existing mocks and realistic configuration; account for target validation rather than bypassing it casually.

## Source map

- `src/core/`: startup, scheduling, profile/target persistence and faults.
- `src/control/`: common control, IMU, sixaxis, PID/input and `multi/`, `rover/`, `wing/` controllers.
- `src/driver/`: peripheral APIs and `mcu/{stm32,at32,native}/` implementations; `src/system/`: startup/linker/platform configuration.
- `src/io/`: USB Configurator, QUIC/MSP, Blackbox, GPS, battery and VTX.
- `src/rx/`: receivers; `src/osd/`: rendering/menus; `src/util/`: shared algorithms/helpers.
