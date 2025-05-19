#line 1 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/ellipse_generation.h"
#ifndef ELLIPSE_GENERATION_H
#define ELLIPSE_GENERATION_H
#include<Arduino.h>

void ellipseGeneration(float ellipsePoints[][3], float xStart, float xEnd, float yStart, float yEnd, float strideHeight, float zShift, int numPoints) ;

void linspace(float start, float end, int num, float output[]) ;

#endif