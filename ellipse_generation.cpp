#include"ellipse_generation.h"
#include"inverse_kinematics.h" 
#include<math.h>

#define pi 3.14159265359

void linspace(float start, float end, int num, float output[]) ;

void linspace(float start, float end, int num, float output[]) 
{
    float step = (end - start) / (num - 1);
    for (int i = 0; i < num; ++i) {
      output[i] = start + step * i;
    }
}

void ellipseGeneration(float ellipsePoints[][3] , float xStart , float xEnd , float yStart ,float yEnd , float strideHeight , float zShift, int numPoints)
{
    float outputX[numPoints] ; 
    float outputY[numPoints] ;

    linspace(xStart , xEnd , numPoints , outputX) ;
    linspace(yStart , yEnd , numPoints , outputY) ; 

    for(int i=0 ; i<numPoints ; i++)
    {
        ellipsePoints[i][0] = outputX[i] ;
        ellipsePoints[i][1] = outputY[i] ; 
        ellipsePoints[i][2] = (strideHeight/2)*(1-cos(2*pi*(outputY[i] - yStart))/(yEnd - yStart)) ; 
    }
}

