#line 1 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/gait_generation.cpp"
#include<math.h>
#include<gait_generation.h>
#include<inverse_kinematics.h> 

// these legAngles are universal legAngles 

float LINK1 = 5.0;
float LINK2 = 9.0;
float LINK3 = 20.0;

float ShiftParams[6][2] = {{5.25,9.093},
                           {10.5,0},
                           {5.25,-9.093},
                           {-5.25,-9.093},
                           {-10.5,0},
                           {-5.25,9.093}
                        };

bool isValidHeight(float h) 
{
    return h >= 12 && h <= 20;  
}

bool Stand(float height , float legAngles[6][3])
{    
    bool all_legs_successful = true ;
    // these are the stand coordinates for the hexapod 
    float StandParams[6][3] = {{11.75, 20.351 ,height},
                               {23.50,    0   ,height},
                               {11.75, -20.351,height},
                               {-11.75,-20.351,height},
                               {-23.50, 0     ,height},
                               {-11.75, 20.351,height}
                            };
    for (int i = 0; i < 6; i++) {
        // ik for each leg 
        bool success = inverseKinematics(StandParams[i],ShiftParams[i],legAngles[i], LINK1, LINK2, LINK3);
        if (!success) {
            Serial.print("IK failed for leg ");
            Serial.println(i);
            all_legs_successful = false;
        }
    }
    return all_legs_successful;
}