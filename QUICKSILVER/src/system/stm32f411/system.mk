ARCH_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 
DEVICE_FLAGS = -DSTM32F4XX -DSTM32F40XX -DUSE_STDPERIPH_DRIVER

SYSTEM_LD_SCRIPT = stm32f4_flash.ld

SYSTEM_INCLUDE = src/system/stm32f411 \
	Libraries/CMSIS/Include \
	Libraries/Device/STM32F4xx/Include \
	Libraries/STM32F4xx_StdPeriph_Driver/inc

SYSTEM_SOURCE = $(wildcard Libraries/STM32F4xx_StdPeriph_Driver/src/*.c) \
	$(wildcard src/system/stm32f411/*.c) \
	src/system/stm32f411/startup_stm32f40xx.s