#include<math.h>
#include<gait_generation.h>
#include<inverse_kinematics.h>
#include<ellipse_generation.h>
#include <cmath> 
#include <numeric> // For std::accumulate if needed

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
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
    return h >= -24 && h <= -8;  // these are the valid height the bot can make
}

bool Stand(float height , float legAngles[6][3])
{    
    bool all_legs_successful = true ;
    // these are the stand coordinates for the hexapod 
    float StandParams[6][3] = { { -10.0,   17.3205, height },
                                { -20.0,    0.0,    height },
                                { -10.0,  -17.3205, height },
                                {  10.0,  -17.3205, height },
                                {  20.0,    0.0,    height },
                                {  10.0,   17.3205, height } 
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

float toRadians(float degrees) {
    return degrees * (M_PI / 180.0f);
}

bool Tilt(float height, float legAngles[6][3], float Angle) {
    bool all_legs_successful = true;

    // Reconstruct StandParams every time
    float StandParams[6][3] = {
        { -10.0,  17.3205, height },
        { -20.0,   0.0,    height },
        { -10.0, -17.3205, height },
        {  10.0, -17.3205, height },
        {  20.0,   0.0,    height },
        {  10.0,  17.3205, height }
    };

    // Compute center of body
    float center[3] = {0, 0, 0};
    for (int i = 0; i < 6; i++) {
        center[0] += StandParams[i][0];
        center[1] += StandParams[i][1];
        center[2] += StandParams[i][2];
    }
    center[0] /= 6.0;
    center[1] /= 6.0;
    center[2] /= 6.0;

    // Tilt axis in XY plane: perpendicular to tilt direction
    float axis[3] = {
        -sin(Angle),
         cos(Angle),
         0.0
    };
    // Normalize tilt axis (already unit vector in XY)
    float axis_len = sqrt(axis[0]*axis[0] + axis[1]*axis[1]);
    axis[0] /= axis_len;
    axis[1] /= axis_len;

    // Max tilt angle (fixed, e.g., 20°)
    float maxTiltRad = radians(20.0);

    for (int i = 0; i < 6; i++) {
        // Translate leg position to body frame
        float px = StandParams[i][0] - center[0];
        float py = StandParams[i][1] - center[1];
        float pz = StandParams[i][2] - center[2];

        // Compute angle between leg and tilt direction in XY
        float theta_leg = atan2(py, px);
        float angle_diff = theta_leg - Angle;

        // Normalize angle to [-PI, PI]
        while (angle_diff > PI) angle_diff -= TWO_PI;
        while (angle_diff < -PI) angle_diff += TWO_PI;

        // Compute tilt weight [0,1]
        float weight = (cos(angle_diff) + 1.0) * 0.5;
        float theta = maxTiltRad * weight;

        // Rodrigues’ rotation
        float kx = axis[0];
        float ky = axis[1];
        float kz = axis[2];

        float dot = kx * px + ky * py + kz * pz;

        float cross_x = ky * pz - kz * py;
        float cross_y = kz * px - kx * pz;
        float cross_z = kx * py - ky * px;

        float rotated[3];
        rotated[0] = px * cos(theta) + cross_x * sin(theta) + kx * dot * (1 - cos(theta));
        rotated[1] = py * cos(theta) + cross_y * sin(theta) + ky * dot * (1 - cos(theta));
        rotated[2] = pz * cos(theta) + cross_z * sin(theta) + kz * dot * (1 - cos(theta));

        // Translate back
        float finalPos[3] = {
            rotated[0] + center[0],
            rotated[1] + center[1],
            rotated[2] + center[2]
        };

        // Call IK for the leg
        bool success = inverseKinematics(finalPos, ShiftParams[i], legAngles[i], LINK1, LINK2, LINK3);
        if (!success) {
            Serial.print("IK failed for leg ");
            Serial.println(i);
            all_legs_successful = false;
        }
    }

    return all_legs_successful;
}


bool WalkGait(float height , float legAngles[6][5][3] , float legAnglesLine[6][5][3], float Angle) 
{
    bool all_legs_successful = true;
    const int stride = 7;
    const int numPoints = 5;
    const float strideHeight = 7;

    float ellipsePoints[6][numPoints][3];
    float LinePoints[6][numPoints][3];

    float StandParams[6][3] = { { -10.0,   17.3205, height },
                                { -20.0,    0.0,    height },
                                { -10.0,  -17.3205, height },
                                {  10.0,  -17.3205, height },
                                {  20.0,    0.0,    height },
                                {  10.0,   17.3205, height }
                            };

    for (int i = 0; i < 6; i++) 
    {
        float xMid = StandParams[i][0];
        float y    = StandParams[i][1];

        float xStart = xMid;
        float xEnd   = xMid;
        float yStart = y - (stride / 2) ; 
        float yEnd   = y + (stride / 2) ;
        float startArr[] = { xStart, yStart, height };
        float EndArr[]   = { xEnd, yEnd, height };
        
        // Midpoint for rotation
        float midPoint[3] = {
            (startArr[0] + EndArr[0]) / 2.0f,
            (startArr[1] + EndArr[1]) / 2.0f,
            (startArr[2] + EndArr[2]) / 2.0f
        };
        
        // Rotate around the midpoint
        RotationZ(startArr, Angle, midPoint);
        RotationZ(EndArr, Angle, midPoint);

        xStart = startArr[0];
        xEnd   = EndArr[0];
        yStart = startArr[1];
        yEnd   = EndArr[1];

        ellipseGeneration(ellipsePoints[i], xStart, xEnd, yStart, yEnd, strideHeight, height, numPoints);
        ellipseGeneration(LinePoints[i], xStart, xEnd, yStart, yEnd, 0, height, numPoints);
        
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

bool RotateHexa(float height , float RotateAngle , float legAngles[6][5][3] , float legAnglesLine[6][5][3])
{
    bool all_legs_successful = true;
    const int numPoints = 5;
    const float strideHeight = 5.5;
    float ellipsePoints[6][numPoints][3];
    float LinePoints[6][numPoints][3];


    float StandParams[6][3] = { { -10.0,   17.3205, height },
                                { -20.0,    0.0,    height },
                                { -10.0,  -17.3205, height },
                                {  10.0,  -17.3205, height },
                                {  20.0,    0.0,    height },
                                {  10.0,   17.3205, height }
                              };
    
    for (int i = 0; i < 6; i++) 
    {
        float xStart = StandParams[i][0];
        float yStart = StandParams[i][1];
        float pivot[] = {0, 0, 0}; 
        RotationZ(StandParams[i], RotateAngle/2, pivot);

        float xEnd   = StandParams[i][0];
        float yEnd   = StandParams[i][1];

        ellipseGeneration(ellipsePoints[i], xStart, xEnd, yStart, yEnd, strideHeight, height, numPoints);
        ellipseGeneration(LinePoints[i], xStart, xEnd, yStart, yEnd, 0, height, numPoints);

        for (int j = 0; j < numPoints; j++) 
        {
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