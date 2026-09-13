# Repository guidance

QUICKSILVER is flight-controller firmware for STM32 F4/F7/G4/H7 and AT32 F435, with multirotor, rover and wing builds.

## Approach

- Trace callers, initialization order, task priorities and state writers before editing. Establish which conditions are reachable; do not add guards, startup handshakes or tests for hypothetical states the application cannot enter.
- Keep changes focused. Address adjacent issues separately unless they block the requested change.
- Follow existing subsystem structure before inventing abstractions. Avoid trivial wrappers, redundant state, speculative counters and APIs with empty implementations added just to hide a conditional.
- Comments should explain actual ownership, timing and invariants. Do not justify code with a scenario contradicted by startup or arming rules.
- Use `rg` for searches and `apply_patch` for edits, not Python or shell replacement scripts merely to modify source text.

## Style and organization

- Use 2-space indentation, same-line opening braces, `snake_case` functions/variables, `UPPER_CASE` constants/macros, `const` where possible and fixed-width hardware types.
- Simple single-statement guards may omit braces; use braces when they clarify branches. Prefer named intermediate values to awkward line wrapping.
- Follow neighboring include conventions: standard libraries before project modules, with the owning header first where established.
- Order functions from supporting operations toward orchestration. Keep helpers near and before their callers, initialization before the update/service function, and any thread entry loop last.
- Do not group new public functions or thread entries at the top merely because they are public. Avoid forward declarations introduced only to invert the established order.
- Headers generally contain includes, constants, types, extern variables, then functions. Keep type-dependent macros beside their types and implementation-only constants in the source file.
- In source files, put file-scope state declarations (including synchronization storage and handles) with the other state near the top, after includes/constants/types and before functions. Do not insert them between function definitions beside the function that initializes them.
- Keep internals private. Do not change production linkage with constructs such as `#ifndef PIO_UNIT_TESTING static #endif` to expose them to tests.
- Use existing driver boundaries and `failloop.h` for critical errors. Respect `FAST_RAM`/`DMA_RAM`; check stack, heap and DMA accessibility when adding threads or buffers.

## Commits

- Use `subsystem: short imperative summary`. Check recent history for component terminology rather than generic Conventional Commit types.
- Lowercase subsystem and initial imperative verb; preserve proper names/acronyms and omit the trailing period.
- Prefer a subject-only commit. Add a brief body only for a reason or non-obvious constraint, not routine summaries or test logs.

## Build and test

Environment names include the vehicle prefix. Current `platformio.ini` and generated `targets/*.ini` are authoritative.

- Default build: `pio run`; clean: `pio run -t clean`.
- MCU build: `pio run -e multi-stm32g473`; board build: `pio run -e multi-befh-betafpvg473_v2`.
- Main MCU coverage: `pio run -e multi-stm32f405 -e multi-stm32f411 -e multi-stm32f745 -e multi-stm32f765 -e multi-stm32f722 -e multi-stm32h743 -e multi-stm32g473 -e multi-at32f435 -e multi-at32f435m`.
- Native tests: `pio test -e multi-test -e rover-test -e wing-test`. Append `--filter test_common` for shared suites or select e.g. `pio test -e wing-test --filter test_wing`. Add `-v` for diagnostics.
- Simulators: `multi-simulator`, `rover-simulator`, `wing-simulator`.
- Test meaningful behavior and reachable transitions, not implementation details. Run checks appropriate to the change; distinguish build/test evidence from hardware validation. Report and investigate intermittent failures rather than silently rerunning until green.

## Scheduling and timing

- This branch uses the cooperative scheduler in `src/core/scheduler.cpp`; task definitions, priorities, masks and periods live in `src/core/tasks.cpp`. `src/core/main.cpp` initializes hardware before entering the scheduler.
- Preserve the ordered `TASK_FLIGHT` path: `sixaxis_read()` → `imu_calc()` → `control()` → `rx_update()`. It runs every loop at REALTIME priority. Lower priorities run only when their mask, period and remaining budget allow.
- Battery and utility tasks have 1 ms periods, barometer and multirotor navigation 10 ms, OSD 1 ms and GPS 5 ms. Registration omits GPS/navigation without a configured GPS port and barometer without detection. Check actual producer/consumer cadence before changing scheduling.
- Ground means neither `flags.arm_state` nor `flags.in_air`. USB, VTX and gestures are ground-only. USB activity blocks normal arming; an arm request during USB activity latches the arm-switch disable. Motor testing is a separate output override.
- Keep flight-time work bounded and non-blocking: limit bytes/items per call, use incremental state machines, and avoid dynamic allocation. Ground configuration may block; preserve `task_reset_runtime()` around maintenance that must be excluded from timing statistics.
- Reject exhausted budgets before unsigned subtraction. Budget skips are not runtime samples and do not reduce `runtime_worst`. Sustained eligible starvation or overload requests a slower loop rate; ground-only work counts for admission but is excluded from flight-rate decisions.
- Runtime fields use CPU cycles internally; debug serialization converts them to microseconds. `percentile_95` is a smoothed peak estimate, not an exact percentile. Consult source for thresholds and fallback behavior instead of copying formulas into guidance.
- If introducing concurrency, identify each resource's owner and publication boundary, audit shared drivers/buffers as well as state, and keep synchronization with its owner. Arming gates and `volatile` do not provide synchronization. Keep interrupt-masked sections short and never wait while masking a required completion interrupt.
- Declare FreeRTOS threads in the `threads` table in `src/core/tasks.cpp` using `CREATE_THREAD`. Keep thread entry functions private to their owning source files where possible. `tasks.cpp` owns thread creation, kernel startup, and FreeRTOS hooks; `scheduler.cpp` retains cooperative dispatch and loop accounting. The Flight thread runs one cooperative scheduler pass per iteration.

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
