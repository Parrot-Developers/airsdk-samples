COMMON_CONFIG_DIR := $(call my-dir)

# Add Common skeleton
TARGET_SKEL_DIRS += $(COMMON_CONFIG_DIR)/../skel

# Include buildext mission config modules
include $(ALCHEMY_WORKSPACE_DIR)/build/dragon_buildext_mission/product.mk
