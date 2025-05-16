# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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