#include "opengl_helper.h"
#include "util/logger.h"
#include <GLES/gl.h>


void drawDemoLines() {
//    pangolin::glDrawLine(-0.5f, -0.5f, 0.0f, 0.5f, -0.5f, 0.0f);
}


void drawLines(GLfloat data[], int points, unsigned char r, unsigned char g, unsigned char b, unsigned char a) {
    MyVertex * tmpBuffer = new MyVertex[points];
    for (int i=0; i<points; ++i) {
        tmpBuffer[i].point[0] = data[i * 3];
        tmpBuffer[i].point[1] = data[i * 3 + 1];
        tmpBuffer[i].point[2] = data[i * 3 + 2];
        tmpBuffer[i].color[0] = r;
        tmpBuffer[i].color[1] = g;
        tmpBuffer[i].color[2] = b;
        tmpBuffer[i].color[3] = a;
    }
    drawLines(tmpBuffer, points);
    delete tmpBuffer;
}

void drawLines(MyVertex* vertices, int points) {
    LOGD("drawLines: sizeof(MyVertex)=%u, points=%d\n", sizeof(MyVertex), points);
    
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * points, vertices, GL_STATIC_DRAW);
    
    glVertexPointer(3, GL_FLOAT, sizeof(MyVertex), 0);
    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MyVertex), (const void*) (3*sizeof(float)));

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glDrawArrays(GL_LINES, 0, points);

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glDeleteBuffers(1, &vbo);
}

void drawLines(GLfloat data[], int points) {
    drawLines(data, points, 255, 0, 0, 100);
}
