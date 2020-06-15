
LOCAL_PATH := $(call my-dir)

airsdk-hello.uid := com.parrot.missions.samples.hello
airsdk-hello.mission-dir := missions/$(airsdk-hello.uid)
airsdk-hello.payload-dir := $(airsdk-hello.mission-dir)/payload

# Copy missions files (fsup/guidance python and json)
include $(CLEAR_VARS)

LOCAL_MODULE := airsdk-hello-files
LOCAL_DESCRIPTION := AirSdk Hello mission files
LOCAL_CATEGORY_PATH := airsdk/hello

LOCAL_COPY_FILES += mission.json:$(airsdk-hello.mission-dir)/

$(foreach __f,$(call all-files-under,fsup,.py), \
	$(eval LOCAL_COPY_FILES += $(__f):$(airsdk-hello.payload-dir)/$(__f)) \
)

$(foreach __f,$(call all-files-under,guidance/python,.py), \
	$(eval LOCAL_COPY_FILES += $(__f):$(airsdk-hello.payload-dir)/$(__f)) \
)

include $(BUILD_CUSTOM)

# Build and copy missions services
include $(CLEAR_VARS)

LOCAL_MODULE := ms_sample
LOCAL_DESTDIR := $(airsdk-hello.payload-dir)/services

LOCAL_SRC_FILES := services/native/sample.cpp

LOCAL_LIBRARIES := \
	libpomp \
	libtelemetry \
	libulog \
	libvideo-ipc \
	libvideo-ipc-client-config \
	opencv

include $(BUILD_EXECUTABLE)
