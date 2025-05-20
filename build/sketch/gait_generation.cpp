#line 1 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/gait_generation.cpp"
#include<math.h>
#include<gait_generation.h>
#include<inverse_kinematics.h>
#include<ellipse_generation.h>

// these legAngles are universal legAngles 

float LINK1 = 5.0;
float LINK2 = 9.0;
float LINK3 = 20.0;



// update this to make it dynmic this is only for hexagon dim of 10.5 
float ShiftParams[6][2] = {{-5.25,9.093},
                           {-10.5,0},
                           {-5.25,-9.093},
                           {5.25,-9.093},
                           {10.5,0},
                           {5.25,9.093}
                        };

bool isValidHeight(float h) 
{
    return h >= -20 && h <= -12;  // these are the valid height the bot can make
}

bool Stand(float height , float legAngles[6][3])
{    
    bool all_legs_successful = true ;
    // these are the stand coordinates for the hexapod 
    float StandParams[6][3] = { {-8.75,   15.155, height},
                                {-17.5,    0,   height},
                                {-8.75,  -15.155, height},
                                { 8.75, -15.155, height},
                                {17.5,   0   ,   height},
                                {8.75,  15.155, height} 
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

bool WalkGait(float height , float legAngles[6][5][3] , float legAnglesLine[6][5][3]) 
{
    bool all_legs_successful = true;
    const int stride = 10;
    const int numPoints = 5;
    const float strideHeight = 6.0;

    float ellipsePoints[6][numPoints][3];
    float LinePoints[6][numPoints][3];

    float StandParams[6][3] = { {-8.75,   15.155, height},
                                {-17.5,    0,   height},
                                {-8.75,  -15.155, height},
                                { 8.75, -15.155, height},
                                {17.5,   0   ,   height},
                                {8.75,  15.155, height} 
                            };

    for (int i = 0; i < 6; i++) {
        float xMid = StandParams[i][0];
        float y    = StandParams[i][1];

        float xStart = xMid;
        float xEnd   = xMid;
        float yStart = y - (stride / 2.0) ; 
        float yEnd   = y + (stride / 2.0) ;
        ellipseGeneration(ellipsePoints[i], xStart, xEnd, yStart, yEnd, strideHeight, height, numPoints);
        ellipseGeneration(LinePoints[i], xStart, xEnd, yStart, yEnd, 0, height, numPoints);
    }

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < numPoints; j++) {
            bool success = inverseKinematics(ellipsePoints[i][j], ShiftParams[i], legAngles[i][j], LINK1, LINK2, LINK3);
            inverseKinematics(LinePoints[i][j], ShiftParams[i], legAnglesLine[i][j], LINK1, LINK2, LINK3);
            if (!success) {
                Serial.print("IK failed for leg ");
                Serial.print(i);
                Serial.print(" at step ");
                Serial.println(j);
                all_legs_successful = false;
            }
        }
    }

    return all_legs_successful;

}