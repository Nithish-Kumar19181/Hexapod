#ifndef ELLIPSE_GENERATION_H
#define ELLIPSE_GENERATION_H
#include<Arduino.h>

void ellipseGeneration(float ellipsePoints[][3] , float xStart , float xEnd , float yStart ,float yEnd , float strideHeight , float zShift, int numPoints);

#endif