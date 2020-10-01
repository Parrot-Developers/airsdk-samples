LOCAL_PATH := $(call my-dir)

airsdk-hello.name        := hello
airsdk-hello.package     := parrot.missions.samples.$(airsdk-hello.name)
airsdk-hello.tree        := $(subst .,/,$(airsdk-hello.package))
airsdk-hello.uid         := com.$(airsdk-hello.package)
airsdk-hello.mission-dir := missions/$(airsdk-hello.uid)
airsdk-hello.payload-dir := $(airsdk-hello.mission-dir)/payload

airsdk-hello.fsup-dir        := $(airsdk-hello.payload-dir)/fsup
airsdk-hello.guidance-dir    := $(airsdk-hello.payload-dir)/guidance
airsdk-hello.pb-python-dir   := $(airsdk-hello.payload-dir)/python/
airsdk-hello.pb-src-dir      := messages/protobuf

# Copy all files relative to SOURCE/ that match *.SUFIX into TARGET
# ($1:SOURCE $2:SUFFIX $3:TARGET)

copy-all-under = $(foreach __f,\
	$(call all-files-under,$1,$2),\
	$(eval LOCAL_COPY_FILES += $(__f):$(patsubst $1/%,$3/%,$(__f))))

#############################################################
# Copy missions files (fsup/guidance python and json)

include $(CLEAR_VARS)

LOCAL_MODULE := airsdk-hello-files
LOCAL_DESCRIPTION := AirSdk Hello mission files
LOCAL_CATEGORY_PATH := airsdk/missions/samples/hello

# Required for integrated build
LOCAL_COPY_FILES += \
	product/common/skel/$(airsdk-hello.mission-dir)/mission.json:$(airsdk-hello.mission-dir)/

$(call copy-all-under,autopilot-plugins/fsup,.py,$(airsdk-hello.fsup-dir))
$(call copy-all-under,autopilot-plugins/guidance/python,.py,$(airsdk-hello.guidance-dir))

LOCAL_LIBRARIES := \
	airsdk-hello-cv-service \
	libguidance-airsdk-hello-pbpy \
	libmission-airsdk-hello-pbpy

include $(BUILD_CUSTOM)

#############################################################
# Messages exchanged between application and hello mission
# (python code generated from protobuf)

include $(CLEAR_VARS)

LOCAL_MODULE := libmission-airsdk-hello-pbpy
LOCAL_DESCRIPTION := Protobuf python code for hello mission
LOCAL_CATEGORY_PATH := airsdk/missions/samples/hello
LOCAL_LIBRARIES := \
	protobuf-python

# Hello mission protobuf library (Python)
hello_mission_proto_path := $(airsdk-hello.pb-src-dir)/$(airsdk-hello.tree)/airsdk
hello_mission_proto_files := \
	$(call all-files-under,$(hello_mission_proto_path),.proto)

$(foreach __f,$(hello_mission_proto_files), \
	$(eval LOCAL_CUSTOM_MACROS += $(subst $(space),,protoc-macro:python, \
		$(TARGET_OUT_STAGING)/$(airsdk-hello.pb-python-dir), \
		$(LOCAL_PATH)/$(__f), \
		$(LOCAL_PATH)/$(airsdk-hello.pb-src-dir))) \
)

include $(BUILD_CUSTOM)

#############################################################
# Messages exchanged between mission and guidance hello mode
# (python code generated from protobuf)

include $(CLEAR_VARS)

LOCAL_MODULE := libguidance-airsdk-hello-pbpy
LOCAL_DESCRIPTION := Protobuf python code for hello guidance
LOCAL_CATEGORY_PATH := airsdk/missions/samples/hello
LOCAL_LIBRARIES := \
	protobuf-python

# Hello guidance protobuf library (Python)
hello_guidance_proto_path := $(airsdk-hello.pb-src-dir)/$(airsdk-hello.tree)/guidance
hello_guidance_proto_files := \
	$(call all-files-under,$(hello_guidance_proto_path),.proto)

$(foreach __f,$(hello_guidance_proto_files), \
	$(eval LOCAL_CUSTOM_MACROS += $(subst $(space),,protoc-macro:python, \
		$(TARGET_OUT_STAGING)/$(airsdk-hello.pb-python-dir), \
		$(LOCAL_PATH)/$(__f), \
		$(LOCAL_PATH)/$(airsdk-hello.pb-src-dir))) \
)

include $(BUILD_CUSTOM)

#############################################################
# Build and copy missions services

include $(CLEAR_VARS)

LOCAL_MODULE := airsdk-hello-cv-service
LOCAL_CATEGORY_PATH := airsdk/missions/samples/hello
LOCAL_DESTDIR := $(airsdk-hello.payload-dir)/services/bin

LOCAL_SRC_FILES := services/native/sample.cpp

LOCAL_LIBRARIES := \
	libpomp \
	libtelemetry \
	libulog \
	libvideo-ipc \
	libvideo-ipc-client-config \
	opencv4

include $(BUILD_EXECUTABLE)
