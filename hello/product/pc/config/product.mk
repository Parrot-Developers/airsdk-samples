HELLO_PC_CONFIG_DIR := $(call my-dir)

TARGET_SDK_DIRS = $(ALCHEMY_WORKSPACE_DIR)/sdk/pc

# Add pc skeleton
TARGET_SKEL_DIRS += $(HELLO_PC_CONFIG_DIR)/../skel

# Include common config modules
include $(HELLO_PC_CONFIG_DIR)/../../common/config/product.mk
