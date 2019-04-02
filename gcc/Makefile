TARGET=bwhoop
EXECUTABLE=bwhoop.elf

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

topdir = ..

DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F031
STARTUP = $(topdir)/Libraries/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc_ride7/startup_stm32f030.s

MCU = cortex-m0
MCFLAGS = -mcpu=$(MCU) -g -ggdb -mthumb -fdata-sections -ffunction-sections -fsingle-precision-constant -ffast-math -nostartfiles --specs=nano.specs --specs=nosys.specs -flto

INCLUDES = -I$(topdir)/Silverware/src/ \
	-I$(topdir)/Libraries/CMSIS/Device/ST/STM32F0xx/Include/ \
	-I$(topdir)/Libraries/CMSIS/Include/ \
	-I$(topdir)/Utilities/ \
	-I$(topdir)/Libraries/STM32F0xx_StdPeriph_Driver/inc/

OPTIMIZE = -Os

CFLAGS = $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(INCLUDES)  -Wl,-T,flash.ld,-Map,output.map,--gc-sections  -std=gnu99

AFLAGS = $(MCFLAGS)

SRC = $(wildcard $(topdir)/Silverware/src/*.c) \
	$(wildcard $(topdir)/Silverware/src/*.h) \
	$(wildcard $(topdir)/Silverware/src/*.cpp) \
	$(topdir)/Libraries/STM32F0xx_StdPeriph_Driver/src/*.c  \
	$(topdir)/Utilities/system_stm32f0xx.c \
	methods.c

OBJDIR = .
OBJ = $(patsubst %.c,$(OBJDIR)/%.o,$(filter %.c,$(SRC)))
OBJ += $(patsubst %.cpp,$(OBJDIR)/%.o,$(filter %.cpp,$(SRC)))
OBJ += Startup.o


all: $(TARGET)

$(TARGET).hex: $(EXECUTABLE)
	$(CP) -O ihex $^ $@
$(TARGET): $(EXECUTABLE)
	$(CP) -O binary $^ $@

$(EXECUTABLE): $(STARTUP) $(SRC)
	$(CC) $(CFLAGS) $^ -lm -o $@
	arm-none-eabi-size $(EXECUTABLE)


clean:
	rm -f Startup.lst $(TARGET) $(TARGET).lst $(OBJ) $(AUTOGEN) \
		$(TARGET).out $(TARGET).hex  $(TARGET).map \
		$(TARGET).dmp $(EXECUTABLE)
