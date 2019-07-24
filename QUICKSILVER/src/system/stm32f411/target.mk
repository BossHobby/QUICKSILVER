ARCH_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 
DEVICE_FLAGS = -DSTM32F4XX -DSTM32F40XX -DUSE_STDPERIPH_DRIVER

TARGET_LD_SCRIPT = stm32f4_flash.ld

TARGET_INCLUDE = src/system/stm32f405 \
	Libraries/CMSIS/Include \
	Libraries/Device/STM32F4xx/Include \
	Libraries/STM32F4xx_StdPeriph_Driver/inc

TARGET_SOURCE = $(wildcard Libraries/STM32F4xx_StdPeriph_Driver/src/*.c) \
	$(wildcard src/system/stm32f405/*.c) \
	src/system/stm32f405/startup_stm32f40xx.s