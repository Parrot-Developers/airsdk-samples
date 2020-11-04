COMMON_CONFIG_DIR := $(call my-dir)

TARGET_GLOBAL_LDFLAGS := -Wl,-rpath=\$$ORIGIN:\$$ORIGIN/../lib

# Add Common skeleton
TARGET_SKEL_DIRS += $(COMMON_CONFIG_DIR)/../skel
