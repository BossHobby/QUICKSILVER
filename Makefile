CXX=arm-none-eabi-g++
CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
SE=arm-none-eabi-size

GCC_VERSION=$(shell arm-none-eabi-gcc -dumpversion)

INCLUDE = src \
	src/main \
	src/main/rx \
	src/main/osd \
	src/main/config \
	src/main/usb \
	src/drivers \
	src/drivers/usb \
	$(TARGET_INCLUDE) \
	Libraries/cbor/src

SOURCE = $(shell sh -c 'find src/main/ -iname *.c -or -name *.cpp') \
	$(shell sh -c 'find src/drivers/ -maxdepth 1 -iname *.c -or -name *.cpp') \
	$(TARGET_SOURCE) \
	$(shell sh -c 'find Libraries/cbor/src -iname *.c -or -name *.cpp')

ifeq ($(MODE),release)
  OPTIMIZE_FLAGS = -s -O3 -Os
else
  MODE = debug
  OPTIMIZE_FLAGS = -g -O1 -Os
endif

# include our target logic
include src/targets/target.mk

BINARY = quicksilver.$(TARGET).$(MODE)
BUILD_DIR = build/$(MODE)

OBJS    = $(addsuffix .o,$(addprefix $(BUILD_DIR)/,$(basename $(SOURCE))))
DEPS    = $(addsuffix .d,$(addprefix $(BUILD_DIR)/,$(basename $(SOURCE))))

COMMON_FLAGS = -Wall -Wdouble-promotion -MMD -MP \
	--specs=nano.specs --specs=nosys.specs \
	-fsingle-precision-constant -fno-exceptions -fno-strict-aliasing \
	-ffunction-sections -fdata-sections -fstack-usage \
	-fno-stack-protector -fomit-frame-pointer \
	-fno-unwind-tables -fno-asynchronous-unwind-tables \
	-fno-math-errno -fmerge-all-constants

CFLAGS   = $(OPTIMIZE_FLAGS) $(ARCH_FLAGS) $(DEVICE_FLAGS) $(TARGET_FLAGS) $(COMMON_FLAGS)  \
	-std=gnu11
CXXFLAGS = $(OPTIMIZE_FLAGS) $(ARCH_FLAGS) $(DEVICE_FLAGS) $(TARGET_FLAGS) $(COMMON_FLAGS)  \
	-std=gnu++17 -fno-rtti -fno-exceptions -fvtable-gc -fno-threadsafe-statics -fno-use-cxa-atexit

LDFLAGS  += -static -lc -lnosys -lm -Wl,-L$(SYSTEM_DIR),-T$(SYSTEM_LD_SCRIPT),-Map,$(BUILD_DIR)/$(BINARY).map,--verbose,--gc-sections,--print-gc-sections \
	$(OPTIMIZE_FLAGS) $(ARCH_FLAGS) $(DEVICE_FLAGS) $(TARGET_FLAGS) $(COMMON_FLAGS)

ASFLAGS  += $(OPTIMIZE_FLAGS) $(ARCH_FLAGS) $(DEVICE_FLAGS) $(TARGET_FLAGS) $(COMMON_FLAGS) -x assembler-with-cpp $(addprefix -I,$(INCLUDE)) -MMD -MP

all: $(TARGET)

FORCE:

$(TARGET): $(BUILD_DIR)/target $(BUILD_DIR)/$(BINARY).bin $(BUILD_DIR)/$(BINARY).hex $(BUILD_DIR)/$(BINARY).list
	@echo "built $(BINARY) with gcc $(GCC_VERSION)"
	
$(BUILD_DIR)/target: FORCE
	@if test -r /doesntreallymatterjustneedadelay; then echo got a delay > /dev/null; fi
	@mkdir -p $(BUILD_DIR)
	@echo "Setting target to $(TARGET)"
	@echo "$(TARGET)" > $(BUILD_DIR)/target.tmp
	@if test -r $(BUILD_DIR)/target; then \
		cmp $(BUILD_DIR)/target.tmp $(BUILD_DIR)/target || mv -f $(BUILD_DIR)/target.tmp $(BUILD_DIR)/target; \
	else \
		mv $(BUILD_DIR)/target.tmp $(BUILD_DIR)/target; \
	fi

$(BUILD_DIR)/$(BINARY).bin: $(BUILD_DIR)/$(BINARY).elf
	$(CP) -O binary $^ $@

$(BUILD_DIR)/$(BINARY).hex: $(BUILD_DIR)/$(BINARY).elf
	$(CP) -O ihex $^ $@

$(BUILD_DIR)/$(BINARY).list: $(BUILD_DIR)/$(BINARY).elf
	$(OD) -h -S $^ > $@

$(BUILD_DIR)/$(BINARY).elf: $(OBJS)
	$(LD) $^ $(LDFLAGS) -o $@ 
	$(SE) -A $@

$(BUILD_DIR)/%.o: %.c $(BUILD_DIR)/target
	@mkdir -p $(@D)
	$(CC) $(addprefix -I,$(INCLUDE)) $(CFLAGS) -c $< -o $@  

$(BUILD_DIR)/%.o: %.cpp $(BUILD_DIR)/target
	@mkdir -p $(@D)
	$(CXX) $(addprefix -I,$(INCLUDE)) $(CXXFLAGS) -c $< -o $@ 

$(BUILD_DIR)/%.o: %.cc $(BUILD_DIR)/target
	@mkdir -p $(@D)
	$(CXX) $(addprefix -I,$(INCLUDE)) $(CXXFLAGS) -c $< -o $@ 

$(BUILD_DIR)/%.o: %.s $(BUILD_DIR)/target
	@mkdir -p $(dir $@)
	$(CC) -c -o $@ $(ASFLAGS) $<

$(BUILD_DIR)/%.o: %.S $(BUILD_DIR)/target
	@mkdir -p $(dir $@)
	$(CC) -c -o $@ $(ASFLAGS) $<

clean:
	rm -rf build

flash: $(BUILD_DIR)/$(BINARY).bin
	(echo -n 'R' > /dev/ttyACM0 && sleep 2) || true
	dfu-util -a 0 -s 0x08000000:leave -D $(BUILD_DIR)/$(BINARY).bin

-include $(DEPS)
