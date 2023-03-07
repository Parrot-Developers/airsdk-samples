HELLO_CLASSIC_CONFIG_DIR := $(call my-dir)

TARGET_SDK_DIRS = $(ALCHEMY_WORKSPACE_DIR)/sdk/classic

# Add classic skeleton
TARGET_SKEL_DIRS += $(HELLO_CLASSIC_CONFIG_DIR)/../skel

# Include common config modules
include $(HELLO_CLASSIC_CONFIG_DIR)/../../common/config/product.mk
