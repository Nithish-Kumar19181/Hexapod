#include <Arduino.h>
#include "inverse_kinematics.h"
#include "ellipse_generation.h"
#include "gait_generation.h"
#include "servo_mapping.h"
#include <SCServo.h>

#define S_RXD 18
#define S_TXD 19

SCSCL sc;

int baseIDs[6] = {3, 18, 15, 12, 9, 6};
float height = -14.0;
const float maxHeight = -20.0;
const float minHeight = -14.0;

float JointAngles[6][3];
float JointAnglesLine[6][3];
float walkAngles[6][5][3];  
float walkAnglesLine[6][5][3];  


enum BotMode { IDLE, STAND, WALK };
BotMode currentMode = IDLE;

void setup() {
    Serial.begin(115200);
    sc.pSerial = &Serial1;
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
}

void loop() {
    handleSerialInput();

    if (currentMode == STAND) {
        if (Stand(height, JointAngles)) {
            moveLegStand(JointAngles);
        } else {
            Serial.println("Stand IK failed for one or more legs.");
        }
    }

    else if (currentMode == WALK) {
        if (WalkGait(height, walkAngles, walkAnglesLine))
        {
            moveLegWalk(walkAngles,walkAnglesLine) ;
        } 
        else {
            Serial.println("WalkGait IK failed.");
        }

        currentMode = STAND;  // Return to STAND after 1 walk cycle
    }
}

void moveLegStand(float jointAngles[6][3]) {
    float jointAngles_mapped[3];

    for (int j = 0; j < 6; j++) 
    {
        int baseID = baseIDs[j];
        mapServoAngles(baseID, jointAngles[j], jointAngles_mapped);

        int pos1 = (int)jointAngles_mapped[0]; // Coxa
        int pos2 = (int)jointAngles_mapped[1]; // Femur
        int pos3 = (int)jointAngles_mapped[2]; // Tibia

        sc.WritePos(baseID,     pos1, 0, 300);
        sc.WritePos(baseID - 1, pos2, 0, 300);
        sc.WritePos(baseID - 2, pos3, 0, 300);
    }
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

            sc.RegWritePos(baseID,     (int)jointAngles_mapped[0], 0, 500);
            sc.RegWritePos(baseID - 1, (int)jointAngles_mapped[1], 0, 500);
            sc.RegWritePos(baseID - 2, (int)jointAngles_mapped[2], 0, 500);

            sc.RegWritePos(baseID_line,     (int)jointAngles_mapped_line[0], 0, 500);
            sc.RegWritePos(baseID_line - 1, (int)jointAngles_mapped_line[1], 0, 500);
            sc.RegWritePos(baseID_line - 2, (int)jointAngles_mapped_line[2], 0, 500);

            sc.RegWriteAction() ;
            delay(50);
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

            sc.RegWritePos(baseID,     (int)jointAngles_mapped_line[0], 0, 500);
            sc.RegWritePos(baseID - 1, (int)jointAngles_mapped_line[1], 0, 500);
            sc.RegWritePos(baseID - 2, (int)jointAngles_mapped_line[2], 0, 500);

            sc.RegWritePos(baseID_line,     (int)jointAngles_mapped[0], 0, 500);
            sc.RegWritePos(baseID_line - 1, (int)jointAngles_mapped[1], 0, 500);
            sc.RegWritePos(baseID_line - 2, (int)jointAngles_mapped[2], 0, 500);

            sc.RegWriteAction() ;
            delay(50);
        }
    }
}


void handleSerialInput() {
    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
            case '1':
                currentMode = STAND;
                Serial.println("Mode: STAND");
                break;

            case 'w':
                if (currentMode == STAND) {
                    currentMode = WALK;
                    Serial.println("Mode: WALK (Tripod gait)");
                } else {
                    Serial.println("Start walking only from STAND mode.");
                }
                break;

            case 'h':
                if (currentMode == STAND && height > maxHeight) {
                    height -= 0.3;
                    Serial.print("Increased height to: ");
                    Serial.println(-height);
                } else if (currentMode != STAND) {
                    Serial.println("Height change only allowed in STAND mode.");
                } else {
                    Serial.println("Max height reached.");
                }
                break;

            case 'd':
                if (currentMode == STAND && height < minHeight) {
                    height += 0.3;
                    Serial.print("Decreased height to: ");
                    Serial.println(-height);
                } else if (currentMode != STAND) {
                    Serial.println("Height change only allowed in STAND mode.");
                } else {
                    Serial.println("Min height reached.");
                }
                break;

            default:
                Serial.print("Unknown command: ");
                Serial.println(cmd);
                break;
        }
    }
}
