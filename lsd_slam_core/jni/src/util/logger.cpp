#include "util/logger.h"


void printTrans(const Sophus::Sim3f::Transformation& trans) {
    std::ostringstream out;
    out << trans;
    LOGD("%s:\n%s", __FUNCTION__, out.str().c_str());
}

void printMatrix4f(const Sophus::Matrix4f& m) {
    std::ostringstream out;
    out << m;
    LOGD("%s:\n%s", __FUNCTION__, out.str().c_str());
}

void printMatrix4x4(GLfloat* m) {
    LOGD("%s:\n", __FUNCTION__);
    for (int i=0; i<4; ++i) {
        LOGD("%f %f %f %f\n", m[i*4], m[i*4+1], m[i*4+2], m[i*4+3]);
    }
}