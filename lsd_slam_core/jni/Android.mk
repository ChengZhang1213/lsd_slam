LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

ROOT := /data/projects/andorid_ndk

LOCAL_CFLAGS += -std=c++11 -Wno-deprecated-declarations
LOCAL_CPPFLAGS += -std=c++11 -O3
LOCAL_MODULE := LSD
LOCAL_ARM_MODE := arm

LSD_PATH := $(LOCAL_PATH)/src

LOCAL_CPP_EXTENSION := .cpp

LOCAL_SRC_FILES += \
    $(LSD_PATH)/DataStructures/Frame.cpp \
    $(LSD_PATH)/DataStructures/FramePoseStruct.cpp \
    $(LSD_PATH)/DataStructures/FrameMemory.cpp \
    $(LSD_PATH)/SlamSystem.cpp \
    $(LSD_PATH)/GUI.cpp \
    $(LSD_PATH)/LiveSLAMWrapper.cpp \
    $(LSD_PATH)/DepthEstimation/DepthMap.cpp \
    $(LSD_PATH)/DepthEstimation/DepthMapPixelHypothesis.cpp \
    $(LSD_PATH)/util/globalFuncs.cpp \
    $(LSD_PATH)/util/SophusUtil.cpp \
    $(LSD_PATH)/util/settings.cpp \
    $(LSD_PATH)/util/Undistorter.cpp \
    $(LSD_PATH)/util/RawLogReader.cpp \
    $(LSD_PATH)/Tracking/Sim3Tracker.cpp \
    $(LSD_PATH)/Tracking/Relocalizer.cpp \
    $(LSD_PATH)/Tracking/SE3Tracker.cpp \
    $(LSD_PATH)/Tracking/TrackingReference.cpp \
    $(LSD_PATH)/IOWrapper/Timestamp.cpp \
    $(LSD_PATH)/GlobalMapping/FabMap.cpp \
    $(LSD_PATH)/GlobalMapping/KeyFrameGraph.cpp \
    $(LSD_PATH)/GlobalMapping/g2oTypeSim3Sophus.cpp \
    $(LSD_PATH)/GlobalMapping/TrackableKeyFrameSearch.cpp \
    $(LSD_PATH)/util/opengl_helper.cpp \
    $(LSD_PATH)/lsd_jni.cpp \

LOCAL_SRC_FILES += \
    $(LSD_PATH)/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp \
    $(LSD_PATH)/IOWrapper/Pangolin/PangolinOutput3DWrapper.cpp \

LOCAL_C_INCLUDES += \
    $(LSD_PATH) \
    $(LSD_PATH)/DataStructures \
    $(LSD_PATH)/DepthEstimation \
    $(LSD_PATH)/GlobalMapping \
    $(LSD_PATH)/IOWrapper \
    $(LSD_PATH)/Tracking \
    $(LSD_PATH)/util \
    $(LSD_PATH)/thirdparty/Sophus \

LOCAL_C_INCLUDES += \
    $(ROOT)/eigen3 \
    $(ROOT)/OpenCV-android-sdk/sdk/native/jni/include \
    $(ROOT)/Boost-for-Android/build/include/boost-1_53 \
    $(ROOT)/pangolin_android/Pangolin/include \
    $(ROOT)/pangolin_android/build/src/include \
    $(ROOT)/g2o \
    $(ROOT)/g2o/g2o \
    $(ROOT)/g2o/build \
    $(ROOT)/g2o/EXTERNAL/csparse \

LOCAL_LDLIBS += -landroid -lGLESv1_CM -lGLESv2 -lz -llog
LOCAL_CFLAGS += -g

#LOCAL_CFLAGS += -fopenmp
#LOCAL_LDFLAGS += -fopenmp

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_C_INCLUDES) #export includes
LOCAL_EXPORT_LDLIBS := $(LOCAL_LDLIBS) #export linker cmds
LOCAL_EXPORT_CFLAGS := $(LOCAL_CFLAGS) #export c flgs
LOCAL_EXPORT_CPPFLAGS := $(LOCAL_CPPFLAGS) #export cpp flgs
LOCAL_EXPORT_CXXFLAGS := $(LOCAL_CXXFLAGS) #export cpp flgs

LOCAL_STATIC_LIBRARIES += boost_android_thread boost_android_filesystem boost_android_system
LOCAL_STATIC_LIBRARIES += opencv_android_calib3d opencv_android_highgui opencv_android_imgcodecs \
    opencv_android_imgproc opencv_android_core opencv_android_features2d opencv_android_flann \
    libwebp libIlmImf libjasper libjpeg libpng libtiff libtbb libtegra_hal
LOCAL_STATIC_LIBRARIES += glm
LOCAL_SHARED_LIBRARIES += pangolin_android libz
LOCAL_SHARED_LIBRARIES += g2o_android_core g2o_android_stuff g2o_android_solver_csparse \
    g2o_android_csparse_extension g2o_android_types_sim3 g2o_android_types_sba g2o_android_ext_csparse \
    g2o_android_types_slam3d


include $(BUILD_SHARED_LIBRARY)

#---------------------------- Boost ----------------------------
include $(CLEAR_VARS)
LOCAL_MODULE := boost_android_thread
LOCAL_SRC_FILES := $(ROOT)/Boost-for-Android/build/lib/libboost_thread-gcc-mt-1_53.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := boost_android_filesystem
LOCAL_SRC_FILES = $(ROOT)/Boost-for-Android/build/lib/libboost_filesystem-gcc-mt-1_53.a
include $(PREBUILT_STATIC_LIBRARY)
    
include $(CLEAR_VARS)
LOCAL_MODULE := boost_android_system
LOCAL_SRC_FILES := $(ROOT)/Boost-for-Android/build/lib/libboost_system-gcc-mt-1_53.a
include $(PREBUILT_STATIC_LIBRARY)

#---------------------------- OpenCV ----------------------------
include $(CLEAR_VARS)
LOCAL_MODULE := opencv_android_calib3d
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/libs/armeabi-v7a/libopencv_calib3d.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_android_highgui
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/libs/armeabi-v7a/libopencv_highgui.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_android_imgcodecs
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/libs/armeabi-v7a/libopencv_imgcodecs.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_android_imgproc
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/libs/armeabi-v7a/libopencv_imgproc.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_android_core
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/libs/armeabi-v7a/libopencv_core.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_android_features2d
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/libs/armeabi-v7a/libopencv_features2d.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_android_flann
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/libs/armeabi-v7a/libopencv_flann.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libwebp
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/3rdparty/libs/armeabi-v7a/liblibwebp.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libIlmImf
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/3rdparty/libs/armeabi-v7a/libIlmImf.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libjasper
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/3rdparty/libs/armeabi-v7a/liblibjasper.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libjpeg
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/3rdparty/libs/armeabi-v7a/liblibjpeg.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libpng
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/3rdparty/libs/armeabi-v7a/liblibpng.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libtiff
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/3rdparty/libs/armeabi-v7a/liblibtiff.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libtbb
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/3rdparty/libs/armeabi-v7a/libtbb.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libtegra_hal
LOCAL_SRC_FILES := $(ROOT)/OpenCV-android-sdk/sdk/native/3rdparty/libs/armeabi-v7a/libtegra_hal.a
include $(PREBUILT_STATIC_LIBRARY)

#---------------------------- pangolin ----------------------------
include $(CLEAR_VARS)
LOCAL_MODULE := pangolin_android
LOCAL_SRC_FILES := $(ROOT)/pangolin_android/build/libs/armeabi-v7a/libpangolin.so
include $(PREBUILT_SHARED_LIBRARY)

#---------------------------- g2o ----------------------------
include $(CLEAR_VARS)
LOCAL_MODULE := g2o_android_core
LOCAL_SRC_FILES := $(ROOT)/g2o/lib/libg2o_core.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := g2o_android_stuff
LOCAL_SRC_FILES := $(ROOT)/g2o/lib/libg2o_stuff.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := g2o_android_solver_csparse
LOCAL_SRC_FILES := $(ROOT)/g2o/lib/libg2o_solver_csparse.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := g2o_android_csparse_extension
LOCAL_SRC_FILES := $(ROOT)/g2o/lib/libg2o_csparse_extension.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := g2o_android_types_sim3
LOCAL_SRC_FILES := $(ROOT)/g2o/lib/libg2o_types_sim3.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := g2o_android_types_sba
LOCAL_SRC_FILES := $(ROOT)/g2o/lib/libg2o_types_sba.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := g2o_android_ext_csparse
LOCAL_SRC_FILES := $(ROOT)/g2o/lib/libg2o_ext_csparse.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := g2o_android_types_slam3d
LOCAL_SRC_FILES := $(ROOT)/g2o/lib/libg2o_types_slam3d.so
include $(PREBUILT_SHARED_LIBRARY)

$(call import-add-path,$(LOCAL_PATH)/../ndk-modules)
$(call import-module,glm)
