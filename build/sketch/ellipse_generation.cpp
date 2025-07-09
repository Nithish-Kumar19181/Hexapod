#line 1 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/ellipse_generation.cpp"
#include "ellipse_generation.h"
#include "inverse_kinematics.h" 
#include <math.h>

#define pi 3.14159265359

void linspace(float start, float end, int num, float output[]);

void easedLinspace(float start, float end, int num, float output[]) {
    for (int i = 0; i < num; ++i) {
        float t = (float)i / (num - 1); // normalized [0, 1]

        // Quintic ease-in-out for even smoother acceleration and deceleration
        float ease = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);

        output[i] = start + (end - start) * ease;
    }
}

void ellipseGeneration(float ellipsePoints[][3], float xStart, float xEnd, float yStart, float yEnd, float strideHeight, float zShift, int numPoints)
{
    float outputX[numPoints];  
    float outputY[numPoints];

    easedLinspace(xStart, xEnd, numPoints, outputX);
    easedLinspace(yStart, yEnd, numPoints, outputY);

    for (int i = 0; i < numPoints; i++) {
        ellipsePoints[i][0] = outputX[i];
        ellipsePoints[i][1] = outputY[i];
        // MODIFICATION: Using numPoints parameter instead of a hardcoded value.
        float phase = ((float)i / (numPoints - 1)) * 2 * pi;
        ellipsePoints[i][2] = zShift + (strideHeight / 2.0) * (1.0 - cos(phase)); // z = lifted
    }
}