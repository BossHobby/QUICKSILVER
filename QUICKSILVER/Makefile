CXX=arm-none-eabi-g++
CC=arm-none-eabi-gcc
LD=arm-none-eabi-g++
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
SE=arm-none-eabi-size

BUILD_DIR = build
BINARY = QUICKSILVER

rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))

# TARGET = stm32f4
TARGET_DIR = src/system
include $(TARGET_DIR)/target.mk

INCLUDE = src src/main \
	src/drivers \
	src/drivers/usb \
	$(TARGET_INCLUDE)

SOURCE  = $(call rwildcard,src/,*.c) \
	$(call rwildcard,src/,*.cpp) \
	$(TARGET_SOURCE)

OBJS    = $(addsuffix .o,$(addprefix $(BUILD_DIR)/,$(basename $(SOURCE))))
DEPS    = $(addsuffix .d,$(addprefix $(BUILD_DIR)/,$(basename $(SOURCE))))


all: release

debug: OPTIMIZE_FLAGS = -g -O0
debug: $(BINARY)

release: OPTIMIZE_FLAGS = -O3
release: $(BINARY)

COMMON_FLAGS = -Wall -Wextra -MMD -MP --specs=nano.specs --specs=nosys.specs \
	-fsingle-precision-constant -fno-exceptions -fstack-usage \
	-ffunction-sections -fdata-sections

CFLAGS   = $(COMMON_FLAGS) $(OPTIMIZE_FLAGS) $(ARCH_FLAGS) $(DEVICE_FLAGS)
CXXFLAGS = $(COMMON_FLAGS) $(OPTIMIZE_FLAGS) $(ARCH_FLAGS) $(DEVICE_FLAGS) \
	-fno-rtti -std=c++17

LDFLAGS  = $(COMMON_FLAGS) $(ARCH_FLAGS) $(DEVICE_FLAGS) \
	-Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group \
	-static -Wl,-gc-sections -Wl,-L$(TARGET_DIR) -T$(TARGET_LD_SCRIPT)

ASFLAGS  = $(COMMON_FLAGS) $(ARCH_FLAGS) -x assembler-with-cpp $(addprefix -I,$(INCLUDE)) -MMD -MP

$(BINARY): $(BUILD_DIR)/$(BINARY).bin $(BUILD_DIR)/$(BINARY).hex

$(BUILD_DIR)/$(BINARY).bin: $(BUILD_DIR)/$(BINARY).elf
	$(CP) -O binary $(BUILD_DIR)/$(BINARY).elf $(BUILD_DIR)/$(BINARY).bin

$(BUILD_DIR)/$(BINARY).hex: $(BUILD_DIR)/$(BINARY).elf
	$(CP) -O ihex $(BUILD_DIR)/$(BINARY).elf $(BUILD_DIR)/$(BINARY).hex

$(BUILD_DIR)/$(BINARY).elf: $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $^ 

$(BUILD_DIR)/%.o: %.c
	mkdir -p $(@D)
	$(CC) $(addprefix -I,$(INCLUDE)) $(CFLAGS) -c $< -o $@  

$(BUILD_DIR)/%.o: %.cpp
	mkdir -p $(@D)
	$(CXX) $(addprefix -I,$(INCLUDE)) $(CXXFLAGS) -c $< -o $@ 

$(BUILD_DIR)/%.o: %.cc
	mkdir -p $(@D)
	$(CXX) $(addprefix -I,$(INCLUDE)) $(CXXFLAGS) -c $< -o $@ 

$(BUILD_DIR)/%.o: %.s
	mkdir -p $(dir $@)
	$(CC) -c -o $@ $(ASFLAGS) $<

$(BUILD_DIR)/%.o: %.S
	mkdir -p $(dir $@)
	$(CC) -c -o $@ $(ASFLAGS) $<

clean:
	rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(BINARY).bin
	(echo -n 'R' > /dev/ttyACM0 && sleep 2) || true
	dfu-util -a 0 -s 0x08000000:leave -R -D $(BUILD_DIR)/$(BINARY).bin

-include $(DEPS)