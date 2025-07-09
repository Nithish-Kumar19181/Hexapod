#line 1 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/gait_generation.h"
#ifndef GAIT_GENERATION_H
#define GAIT_GENERATION_H
#include <Arduino.h>

// MODIFICATION: Added a global constant for the number of gait points.
#define NUM_POINTS 10

// MODIFICATION: Updated function signatures to use the NUM_POINTS constant.
bool Stand(float height , float legAngles[6][3]) ;

bool WalkGait(float height , float legAngles[6][NUM_POINTS][3] , float legAnglesLine[6][NUM_POINTS][3], float Angle) ;

bool RotateHexa(float height , float RotateAngle , float legAngles[6][NUM_POINTS][3] , float legAnglesLine[6][NUM_POINTS][3]) ;

bool Tilt(float height, float legAngles[6][3], float Angle) ;


#endif