
LOCAL_PATH := $(call my-dir)

airsdk-hello.name := hello
airsdk-hello.uid := com.parrot.missions.samples.$(airsdk-hello.name)
airsdk-hello.mission-dir := missions/$(airsdk-hello.uid)
airsdk-hello.payload-dir := $(airsdk-hello.mission-dir)/payload

# Copy missions files (fsup/guidance python and json)
include $(CLEAR_VARS)

LOCAL_MODULE := airsdk-hello-files
LOCAL_DESCRIPTION := AirSdk Hello mission files
LOCAL_CATEGORY_PATH := airsdk/mission/samples/hello

LOCAL_COPY_FILES += mission.json:$(airsdk-hello.mission-dir)/

$(foreach __f,$(call all-files-under,fsup,.py), \
	$(eval LOCAL_COPY_FILES += $(__f):$(airsdk-hello.payload-dir)/$(__f)) \
)

$(foreach __f,$(call all-files-under,guidance/python,.py), \
	$(eval LOCAL_COPY_FILES += $(__f):$(airsdk-hello.payload-dir)/$(__f)) \
)

LOCAL_LIBRARIES := \
	airsdk-hello-cv-service \
	libmission-airsdk-hello-pbpy

include $(BUILD_CUSTOM)

# Hello mission protobuf library (Python)
hello_mission_proto_path := messages/protobuf
hello_mission_proto_files := \
	$(call all-files-under,$(hello_mission_proto_path),.proto)

include $(CLEAR_VARS)

LOCAL_MODULE := libmission-airsdk-hello-pbpy
LOCAL_DESCRIPTION := Protobuf python code for hello mission
LOCAL_CATEGORY_PATH := airsdk/missions/samples/hello
LOCAL_LIBRARIES := \
	protobuf-python

$(foreach __f,$(hello_mission_proto_files), \
	$(eval LOCAL_CUSTOM_MACROS += $(subst $(space),,protoc-macro:python, \
		$(TARGET_OUT_STAGING)/$(airsdk-hello.payload-dir)/python/$(airsdk-hello.name), \
		$(LOCAL_PATH)/$(__f), \
		$(LOCAL_PATH)/$(hello_mission_proto_path))) \
)

include $(BUILD_CUSTOM)

# Build and copy missions services
include $(CLEAR_VARS)

LOCAL_MODULE := airsdk-hello-cv-service
LOCAL_CATEGORY_PATH := airsdk/mission/samples/hello
LOCAL_DESTDIR := $(airsdk-hello.payload-dir)/services/bin

LOCAL_SRC_FILES := services/native/sample.cpp

LOCAL_LIBRARIES := \
	libpomp \
	libtelemetry \
	libulog \
	libvideo-ipc \
	libvideo-ipc-client-config \
	opencv

include $(BUILD_EXECUTABLE)
