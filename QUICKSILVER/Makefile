CXX=arm-none-eabi-g++
CC=arm-none-eabi-gcc
LD=arm-none-eabi-g++
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
SE=arm-none-eabi-size

# include our target logic
include src/targets/target.mk

INCLUDE = src \
	src/main \
	src/main/rx\
	src/drivers \
	src/drivers/usb \
	$(TARGET_INCLUDE)

SOURCE = $(shell sh -c 'find src/main/ -iname *.c -or -name *.cpp') \
	$(shell sh -c 'find src/drivers/ -maxdepth 1 -iname *.c -or -name *.cpp') \
	$(TARGET_SOURCE) 

BINARY = quicksilver.$(TARGET).$(MODE)
BUILD_DIR = build/$(MODE)

OBJS    = $(addsuffix .o,$(addprefix $(BUILD_DIR)/,$(basename $(SOURCE))))
DEPS    = $(addsuffix .d,$(addprefix $(BUILD_DIR)/,$(basename $(SOURCE))))

COMMON_FLAGS = -Wall -MMD -MP --specs=nano.specs --specs=nosys.specs \
	-fsingle-precision-constant -fno-exceptions -fstack-usage \
	-ffunction-sections -fdata-sections -fno-strict-aliasing \
	-flto -ffast-math

CFLAGS   = $(COMMON_FLAGS) $(OPTIMIZE_FLAGS) $(ARCH_FLAGS) $(DEVICE_FLAGS) \
	-std=c11
CXXFLAGS = $(COMMON_FLAGS) $(OPTIMIZE_FLAGS) $(ARCH_FLAGS) $(DEVICE_FLAGS) \
	-std=c++17 -fno-rtti -fno-exceptions 

LDFLAGS  = $(COMMON_FLAGS) $(ARCH_FLAGS) $(DEVICE_FLAGS) \
	-Wl,--start-group -lc -lm -Wl,--end-group \
	-static -Wl,-L$(SYSTEM_DIR),-T$(SYSTEM_LD_SCRIPT),-Map,output.map,--gc-sections

ASFLAGS  = $(COMMON_FLAGS) $(ARCH_FLAGS) -x assembler-with-cpp $(addprefix -I,$(INCLUDE)) -MMD -MP

ifeq ($(MODE),release)
  OPTIMIZE_FLAGS = -O3 -Os
else
  MODE = debug
  OPTIMIZE_FLAGS = -g -O1 -Os
	LDFLAGS += -u _printf_float
endif

all: $(TARGET)

FORCE:

$(TARGET): $(BUILD_DIR)/target $(BUILD_DIR)/$(BINARY).bin $(BUILD_DIR)/$(BINARY).hex
	
$(BUILD_DIR)/target: FORCE
	@mkdir -p $(BUILD_DIR)
	@echo "Setting target to $(TARGET)"
	@echo "$(TARGET)" > $(BUILD_DIR)/target.tmp
	@if test -r $(BUILD_DIR)/target; then \
		cmp $(BUILD_DIR)/target.tmp $(BUILD_DIR)/target || mv -f $(BUILD_DIR)/target.tmp $(BUILD_DIR)/target; \
	else \
		mv $(BUILD_DIR)/target.tmp $(BUILD_DIR)/target; \
	fi

$(BUILD_DIR)/$(BINARY).bin: $(BUILD_DIR)/$(BINARY).elf
	$(CP) -O binary $(BUILD_DIR)/$(BINARY).elf $(BUILD_DIR)/$(BINARY).bin

$(BUILD_DIR)/$(BINARY).hex: $(BUILD_DIR)/$(BINARY).elf
	$(CP) -O ihex $(BUILD_DIR)/$(BINARY).elf $(BUILD_DIR)/$(BINARY).hex

$(BUILD_DIR)/$(BINARY).elf: $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $^ 

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

#-include $(DEPS)