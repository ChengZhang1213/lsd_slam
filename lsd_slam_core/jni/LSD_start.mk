ROOT := /data/projects/andorid_ndk

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# ------------------- App Entry ------------------
LOCAL_MODULE := LSD_start

LOCAL_ARM_MODE := arm
LOCAL_CFLAGS += -std=c++11 -Wno-deprecated-declarations
LOCAL_CPPFLAGS += -std=c++11 -O3

LSD_PATH := $(LOCAL_PATH)/src
LOCAL_CPP_EXTENSION := .cpp
LOCAL_SRC_FILES := \
    $(LSD_PATH)/LSD_start.cpp

LOCAL_C_INCLUDES += \
    $(LSD_PATH) \

LOCAL_LDLIBS += -landroid -llog
LOCAL_CFLAGS += -g

include $(BUILD_SHARED_LIBRARY)
