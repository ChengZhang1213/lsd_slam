#include "util/logger.h"
#include <GLES/gl.h>
#include <cmath>

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

void dumpCurrentMatrix() {
    GLfloat matrix[16];
    glGetFloatv (GL_MODELVIEW_MATRIX, matrix);
    printMatrix4x4(matrix);
}

float computeDist(float* a, float* b, int size) {
    float dist = 0.0f;
    for (int i=0; i<size; ++i) {
        dist += std::pow(a[i] - b[i], 2);
    }
    return std::sqrt(dist);
}