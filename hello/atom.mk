LOCAL_PATH := $(call my-dir)

airsdk-hello.name        := hello
airsdk-hello.uid         := com.parrot.missions.samples.$(airsdk-hello.name)
airsdk-hello.payload-dir := missions/$(airsdk-hello.uid)/payload

airsdk-hello.fsup-dir        := $(airsdk-hello.payload-dir)/fsup
airsdk-hello.guidance-dir    := $(airsdk-hello.payload-dir)/guidance

# Copy all files relative to SOURCE/ that match *.SUFIX into TARGET
# ($1:SOURCE $2:SUFFIX $3:TARGET)

copy-all-under = $(foreach __f,\
	$(call all-files-under,$1,$2),\
	$(eval LOCAL_COPY_FILES += $(__f):$(patsubst $1/%,$3/%,$(__f))))

#############################################################
# Copy autopilot mission files (fsup/guidance python)
include $(CLEAR_VARS)

LOCAL_MODULE := airsdk-hello-autopilot-plugins
LOCAL_DESCRIPTION := AirSdk autopilot hello mission files
LOCAL_CATEGORY_PATH := airsdk/missions/samples/hello

LOCAL_SRC_FILES := $(call all-files-under,autopilot-plugins/fsup,.py)
LOCAL_SRC_FILES += $(call all-files-under,autopilot-plugins/guidance/python,.py)
LOCAL_CODECHECK_PYTHON := flake8

$(call copy-all-under,autopilot-plugins/fsup,.py,$(airsdk-hello.fsup-dir))
$(call copy-all-under,autopilot-plugins/guidance/python,.py,$(airsdk-hello.guidance-dir))

LOCAL_LIBRARIES := \
	airsdk-hello-cv-service \
	libairsdk-hello-cv-service-pbpy \
	libairsdk-hello-guidance-pbpy \
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
	parrot-protobuf-extensions-py \
	protobuf-python

# Hello mission protobuf library (Python)
hello_mission_proto_path := messages/protobuf
hello_mission_proto_files := \
	$(call all-files-under,$(hello_mission_proto_path),.proto)

$(foreach __f,$(hello_mission_proto_files), \
	$(eval LOCAL_CUSTOM_MACROS += $(subst $(space),,protoc-macro:python, \
		$(TARGET_OUT_STAGING)/usr/lib/python/site-packages, \
		$(LOCAL_PATH)/$(__f), \
		$(LOCAL_PATH)/$(hello_mission_proto_path))) \
)

include $(BUILD_CUSTOM)

#############################################################
# Messages exchanged between mission and guidance hello mode
# (python code generated from protobuf)

include $(CLEAR_VARS)

LOCAL_MODULE := libairsdk-hello-guidance-pbpy
LOCAL_DESCRIPTION := Protobuf python code for hello guidance
LOCAL_CATEGORY_PATH := airsdk/missions/samples/hello
LOCAL_LIBRARIES := \
	protobuf-python

# Hello guidance protobuf library (Python)
hello_guidance_proto_path := autopilot-plugins/guidance/protobuf
hello_guidance_proto_files := \
	$(call all-files-under,$(hello_guidance_proto_path),.proto)

$(foreach __f,$(hello_guidance_proto_files), \
	$(eval LOCAL_CUSTOM_MACROS += $(subst $(space),,protoc-macro:python, \
		$(TARGET_OUT_STAGING)/usr/lib/python/site-packages, \
		$(LOCAL_PATH)/$(__f), \
		$(LOCAL_PATH)/$(hello_guidance_proto_path))) \
)

include $(BUILD_CUSTOM)

#############################################################
# Build and copy mission services

include $(CLEAR_VARS)

LOCAL_MODULE := airsdk-hello-cv-service
LOCAL_CATEGORY_PATH := airsdk/missions/samples/hello
LOCAL_DESTDIR := $(airsdk-hello.payload-dir)/services

LOCAL_SRC_FILES := services/native/processing.cpp services/native/sample.cpp

LOCAL_LIBRARIES := \
	libairsdk-hello-cv-service-msghub \
	libmsghub \
	libpomp \
	libtelemetry \
	libulog \
	libvideo-ipc \
	libvideo-ipc-client-config \
	opencv4 \
	protobuf

include $(BUILD_EXECUTABLE)

#############################################################
# Messages exchanged between mission and native cv service

cv_service_proto_path := services/native/protobuf
cv_service_proto_files := $(call all-files-under,$(cv_service_proto_path),.proto)

include $(CLEAR_VARS)

LOCAL_MODULE := libairsdk-hello-cv-service-pbpy
LOCAL_CATEGORY_PATH := airsdk/missions/samples/hello
LOCAL_LIBRARIES := \
	protobuf-python

$(foreach __f,$(cv_service_proto_files), \
	$(eval LOCAL_CUSTOM_MACROS += $(subst $(space),,protoc-macro:python, \
		$(TARGET_OUT_STAGING)/usr/lib/python/site-packages, \
		$(LOCAL_PATH)/$(__f), \
		$(LOCAL_PATH)/$(cv_service_proto_path))) \
)

include $(BUILD_CUSTOM)

include $(CLEAR_VARS)

LOCAL_MODULE := libairsdk-hello-cv-service-pb
LOCAL_CATEGORY_PATH := airsdk/missions/samples/hello
LOCAL_CXXFLAGS := -std=c++11
LOCAL_LIBRARIES := protobuf
LOCAL_EXPORT_C_INCLUDES := $(call local-get-build-dir)/gen

$(foreach __f,$(cv_service_proto_files), \
	$(eval LOCAL_CUSTOM_MACROS += $(subst $(space),,protoc-macro:cpp,gen, \
		$(LOCAL_PATH)/$(__f), \
		$(LOCAL_PATH)/$(cv_service_proto_path))) \
)

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)

LOCAL_MODULE := libairsdk-hello-cv-service-msghub
LOCAL_CATEGORY_PATH := airsdk/missions/samples/hello
LOCAL_CXXFLAGS := -std=c++11
LOCAL_LIBRARIES := protobuf libairsdk-hello-cv-service-pb libmsghub
LOCAL_EXPORT_C_INCLUDES := $(call local-get-build-dir)/gen

$(foreach __f,$(cv_service_proto_files), \
	$(eval LOCAL_CUSTOM_MACROS += $(subst $(space),,msghub-macro:cpp,gen, \
		$(LOCAL_PATH)/$(__f), \
		$(LOCAL_PATH)/$(cv_service_proto_path))) \
)

include $(BUILD_LIBRARY)
