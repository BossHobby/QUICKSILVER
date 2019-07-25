# each directory represents a target
TARGETS := $(sort $(notdir $(wildcard src/targets/*)))
TARGET := $(strip $(foreach dir,$(TARGETS),$(findstring $(dir),$(MAKECMDGOALS))))

ifeq ($(TARGET),)
# default to build AWv2
TARGET = alienwhoop_v2
endif

TARGET_DIR := src/targets/$(TARGET)
include $(TARGET_DIR)/target.mk

# we get SYSTEM from target
SYSTEM_DIR = src/system/$(SYSTEM)
include $(SYSTEM_DIR)/system.mk

TARGET_INCLUDE = $(SYSTEM_INCLUDE) $(TARGET_DIR)
TARGET_SOURCE = $(SYSTEM_SOURCE)