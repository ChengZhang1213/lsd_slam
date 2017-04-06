#ifndef OPENGL_HELPER_H
#define OPENGL_HELPER_H

#include <pangolin/gl/glplatform.h>

struct MyVertex {
    float point[3];
    unsigned char color[4];
};

void drawLines(GLfloat data[], int points);
void drawLines(GLfloat data[], int points, unsigned char r, unsigned char g, unsigned char b, unsigned char a);
void drawLines(MyVertex* vertices, int points);
void drawDemoLines();



#endif
