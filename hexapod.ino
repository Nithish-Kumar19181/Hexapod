#include<Arduino.h> 
#include"inverse_kinematics.h"
#include"ellipse_generation.h"

void setup() 
{
    Serial.begin(9600);
}

void loop() 
{
    float positionMatrix[] = {0, 0, 0} ;
    float jointAngles[3] ;
    float L1 = 10 ;
    float L2 = 10 ;
    float L3 = 10 ;
    bool result = inverseKinematics(positionMatrix , jointAngles , L1 , L2 , L3) ;
    Serial.println(positionMatrix[1]) ;
}