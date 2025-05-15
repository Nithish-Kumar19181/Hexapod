#line 1 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
#include<Arduino.h> 
#include"inverse_kinematics.h"

#line 4 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void setup();
#line 9 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void loop();
#line 4 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
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
