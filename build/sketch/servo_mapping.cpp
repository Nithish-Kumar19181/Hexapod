#line 1 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/servo_mapping.cpp"
#include<math.h>
#include<Arduino.h>

float getBaseAngleOffset(int baseServoID, float angle) 
{
    switch (baseServoID) 
    {
        case 3:
            return (angle / 2.0)+ 90 - 60;
        case 6:
            return (angle / 2.0)+ 90 - 30;
        case 9:
            return angle > 270 ? (angle / 2.0) - 90 : (angle / 2.0) + 90;
        case 12:
            return (angle / 2.0)+ 90 - 150;
        case 15:
            return (angle / 2.0)+ 90 - 120;
        case 18:
            return (angle / 2.0);
        default:
            return 0; // Or some safe fallback
    }
}
                                                            //{BaseAngle,CoxaAngle,TibiaAngle}  
void mapServoAngles(int baseServoID, float jointAngles[3], float jointAngles_mapped[3]) 
{
    for (int i = 0; i < 3; i++) 
    {
        if (i == 0) {  // Coxa
            float offsetAngle = getBaseAngleOffset(baseServoID, jointAngles[i]);
            jointAngles_mapped[i] = 512.0 * cos(PI * offsetAngle / 180.0) + 512.0;
        } 

        else if (i == 1) {  // Femur
            jointAngles_mapped[i] = 512.0 + 512.0 * sin(jointAngles[i] * PI / 180.0);
        } 
        else if (i == 2) {  // Tibia
            jointAngles_mapped[i] = 512.0 + 512.0 * cos((180.0 - jointAngles[i]) * PI / 180.0) - 100.0;
        }
    }
}