ARCH_FLAGS = -mthumb -mcpu=cortex-m0 -mfloat-abi=soft
DEVICE_FLAGS = -DSTM32F031 -DUSE_STDPERIPH_DRIVER

SYSTEM_LD_SCRIPT = stm32f0xx_flash.ld

SYSTEM_INCLUDE = src/system/stm32f0xx \
	Libraries/CMSIS/Include \
	Libraries/Device/STM32F0xx/Include \
	Libraries/STM32F0xx_StdPeriph_Driver/inc

SYSTEM_SOURCE = src/system/stm32f0xx/startup_stm32f0xx.s \
	$(shell sh -c 'find src/system/stm32f0xx -name *.c') \
	$(shell sh -c 'find Libraries/STM32F0xx_StdPeriph_Driver/src -name *.c') 