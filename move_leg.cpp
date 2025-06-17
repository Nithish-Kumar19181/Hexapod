#include "move_leg.h"
#include "servo_mapping.h"
#include <SCServo.h>
#include <cmath>
#include <Arduino.h>
#include <math.h>
#include "inverse_kinematics.h"
#include "ellipse_generation.h"
#include "gait_generation.h"
#include "servo_mapping.h"

#define S_RXD 18
#define S_TXD 19

SCSCL sc;

int baseIDs[6] = {3, 18, 15, 12, 9, 6};

void moveLegStand(float jointAngles[6][3]) {
    float jointAngles_mapped[3];

    for (int j = 0; j < 6; j++) 
    {
        int baseID = baseIDs[j];
        mapServoAngles(baseID, jointAngles[j], jointAngles_mapped);

        int pos1 = (int)jointAngles_mapped[0]; // Coxa
        int pos2 = (int)jointAngles_mapped[1]; // Femur
        int pos3 = (int)jointAngles_mapped[2]; // Tibia

        sc.RegWritePos(baseID,     pos1, 0, 500);
        sc.RegWritePos(baseID - 1, pos2, 0, 500);
        sc.RegWritePos(baseID - 2, pos3, 0, 500);
        
    }
    sc.RegWriteAction() ;
    delay(100); // Allow time for all servos to move
}

void moveLegWalk(float jointAngles[6][5][3], float jointAnglesLine[6][5][3]) 
{
    float jointAngles_mapped[3];
    float jointAngles_mapped_line[3];

    int groupA[3] = {0, 2, 4};
    int groupB[3] = {1, 3, 5};
    
    for (int step = 0; step < 5; step++) 
    {
        for (int k = 0; k < 3; k++) 
        {
            int i = groupB[k];
            int j = groupA[k];
            int baseID = baseIDs[j];
            int baseID_line = baseIDs[i];

            if(baseID == 15 || baseID_line == 12)
            {
                mapServoAngles(baseID, jointAngles[j][4-step], jointAngles_mapped);
                mapServoAngles(baseID_line, jointAnglesLine[i][step], jointAngles_mapped_line);
            }
            else
            {
                mapServoAngles(baseID, jointAngles[j][step], jointAngles_mapped);
                mapServoAngles(baseID_line, jointAnglesLine[i][4-step], jointAngles_mapped_line);
            }

            sc.RegWritePos(baseID,     (int)jointAngles_mapped[0], 0, 700);
            sc.RegWritePos(baseID - 1, (int)jointAngles_mapped[1], 0, 700);
            sc.RegWritePos(baseID - 2, (int)jointAngles_mapped[2], 0, 700);

            sc.RegWritePos(baseID_line,     (int)jointAngles_mapped_line[0], 0, 700);
            sc.RegWritePos(baseID_line - 1, (int)jointAngles_mapped_line[1], 0, 700);
            sc.RegWritePos(baseID_line - 2, (int)jointAngles_mapped_line[2], 0, 700);

            sc.RegWriteAction() ;
            delay(55);
        }
    }

    for (int step = 0; step < 5; step++) 
    {
        for (int k = 0; k < 3; k++) 
        {
            int i = groupB[k];
            int j = groupA[k];
            int baseID = baseIDs[j];
            int baseID_line = baseIDs[i];

            if(baseID == 15 || baseID_line == 12)
            {
                mapServoAngles(baseID, jointAnglesLine[j][step], jointAngles_mapped_line);
                mapServoAngles(baseID_line, jointAngles[i][4-step], jointAngles_mapped);

            }
            else
            {
                mapServoAngles(baseID, jointAnglesLine[j][4-step], jointAngles_mapped_line);
                mapServoAngles(baseID_line, jointAngles[i][step], jointAngles_mapped);
            }

            sc.RegWritePos(baseID,     (int)jointAngles_mapped_line[0], 0, 700);
            sc.RegWritePos(baseID - 1, (int)jointAngles_mapped_line[1], 0, 700);
            sc.RegWritePos(baseID - 2, (int)jointAngles_mapped_line[2], 0, 700);

            sc.RegWritePos(baseID_line,     (int)jointAngles_mapped[0], 0, 700);
            sc.RegWritePos(baseID_line - 1, (int)jointAngles_mapped[1], 0, 700);
            sc.RegWritePos(baseID_line - 2, (int)jointAngles_mapped[2], 0, 700);

            sc.RegWriteAction() ;
            delay(55);
        }
    }
}