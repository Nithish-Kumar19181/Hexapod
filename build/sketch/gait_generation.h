#line 1 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/gait_generation.h"
#ifndef GAIT_GENERATION_H
#define GAIT_GENERATION_H
#include <Arduino.h>

bool Stand(float height , float legAngles[6][3]) ;

bool WalkGait(float height , float legAngles[6][5][3] , float legAnglesLine[6][5][3], float Angle) ;

bool RotateHexa(float height , float RotateAngle , float legAngles[6][5][3] , float legAnglesLine[6][5][3]) ;

bool Tilt(float height, float legAngles[6][3], float Angle) ;


#endif