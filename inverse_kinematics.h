#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H
#include <Arduino.h>

bool inverseKinematics(float positionMatrix[3] , float positionShift[2], float jointAngles[3] , float L1 , float L2 , float L3) ;

void Translation(float positionMatrix[] , float tran_x , float tran_y , float tran_z) ;

void RotationZ(float positionMatrix[] , float thetaz) ;

void RotationX(float positionMatrix[] , float thetax) ;

void RotationY(float positionMatrix[] ,float thetay) ;

#endif