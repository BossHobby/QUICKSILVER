ARCH_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 
DEVICE_FLAGS = -DSTM32F4XX -DSTM32F40XX -DUSE_STDPERIPH_DRIVER

SYSTEM_LD_SCRIPT = stm32f4_flash.ld

SYSTEM_INCLUDE = src/system/stm32f405 \
	Libraries/CMSIS/Include \
	Libraries/Device/STM32F4xx/Include \
	Libraries/STM32F4xx_StdPeriph_Driver/inc


SYSTEM_SOURCE = src/system/stm32f405/startup_stm32f40xx.s \
	$(shell sh -c 'find src/system/stm32f405 -name *.c') \
	$(shell sh -c 'find Libraries/STM32F4xx_StdPeriph_Driver/src -name *.c') \
	$(shell sh -c 'find src/drivers/usb -iname *.c -or -name *.cpp')