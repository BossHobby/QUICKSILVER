# each directory represents a target
TARGETS := $(sort $(notdir $(shell sh -c 'find src/targets/ -type d')))
TARGET := $(strip $(foreach dir,$(MAKECMDGOALS),$(findstring $(dir),$(TARGETS))))

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
TARGET_FLAGS = -DTARGET=$(TARGET)