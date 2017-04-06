#ifndef LOGGER_H_
#define LOGGER_H_

#include <android/log.h>
#include "sophus/sim3.hpp"
#include <GLES2/gl2.h>


#define STRINGIFY(x) #x
#define LOG_TAG    __FILE__ ":" STRINGIFY(LSDNative)
#ifdef LOGD
#undef LOGD
#endif
#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)

#ifdef LOGE
#undef LOGE
#endif
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

void printTrans(const Sophus::Sim3f::Transformation& trans);
void printMatrix4f(const Sophus::Matrix4f& m);
void printMatrix4x4(GLfloat* m);

#endif