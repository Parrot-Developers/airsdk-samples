
LOCAL_PATH := $(call my-dir)

###############################################################################
# Generated protobuf-c code for the detection metadata (shared wire format with
# the mission's msghub/object_detector/detection.proto). Produces
# detection.pb-c.{h,c}.
###############################################################################
include $(CLEAR_VARS)

LOCAL_MODULE := object-detector-metadata-pb
LOCAL_DESCRIPTION := object_detector detection metadata (protobuf-c)
LOCAL_CATEGORY_PATH := airsdk/samples/object_detector
LOCAL_EXPORT_C_INCLUDES := $(call local-get-build-dir)/generated
LOCAL_C_INCLUDES := $(LOCAL_EXPORT_C_INCLUDES)
LOCAL_LIBRARIES := protobuf-c
# Local copy of the wire format (kept in sync with the mission's
# msghub/object_detector/detection.proto). Self-contained so this folder can be
# copied into pdraw as-is, with nothing to edit.
LOCAL_CUSTOM_MACROS := \
	protoc-c-macro:c,generated,$(LOCAL_PATH)/detection.proto

# BUILD_LIBRARY (not BUILD_CUSTOM): compiles the generated detection.pb-c.c into
# a linkable library, so the viewer resolves the unpack/free symbols.
include $(BUILD_LIBRARY)

###############################################################################
# Viewer application: receives the drone stream, renders it with pdraw's OpenGL
# renderer and draws the detection boxes in the render_overlay callback.
###############################################################################
include $(CLEAR_VARS)

LOCAL_MODULE := object_detector_viewer
LOCAL_DESCRIPTION := object_detector detection viewer (pdraw + GLES2 overlay)
LOCAL_CATEGORY_PATH := airsdk/samples/object_detector
LOCAL_CFLAGS := -D_GNU_SOURCE

LOCAL_SRC_FILES := \
	main.c \
	overlay.c

LOCAL_LIBRARIES := \
	libpdraw \
	libpdraw-backend \
	libpomp \
	libulog \
	libvideo-metadata \
	object-detector-metadata-pb \
	protobuf-c \
	sdl2

# Windowing + GL context are provided by SDL2 (see main.c). overlay.c draws with
# the OpenGL ES2 API (<GLES2/gl2.h>), and main.c requests an SDL GLES3 context
# (backward compatible with that GLES2 API). The GL module name is SDK-dependent:
# on linux-native desktop pdraw_desktop links `opengl` (desktop GL, a superset
# of GLES2); on the drone target use the SDK's GLES2 module. Adjust the line
# below to your SDK if the link fails.
LOCAL_LIBRARIES += opengl

include $(BUILD_EXECUTABLE)
