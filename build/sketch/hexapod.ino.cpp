#line 1 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
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
float walkAngles[6][5][3];  

enum BotMode { IDLE, STAND, WALK };
BotMode currentMode = IDLE;

#line 25 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void setup();
#line 31 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void loop();
#line 57 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void moveLeg(float jointAngles[6][3]);
#line 75 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void moveLegWalk(float jointAngles[6][5][3]);
#line 102 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void handleSerialInput();
#line 25 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void setup() {
    Serial.begin(115200);
    sc.pSerial = &Serial1;
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
}

void loop() {
    handleSerialInput();

    if (currentMode == STAND) {
        if (Stand(height, JointAngles)) {
            moveLeg(JointAngles);
        } else {
            Serial.println("Stand IK failed for one or more legs.");
        }
    }

    else if (currentMode == WALK) {
        if (WalkGait(height, walkAngles)) 
        {
            moveLegWalk(walkAngles) ;
        } 
        else {
            Serial.println("WalkGait IK failed.");
        }

        currentMode = STAND;  // Return to STAND after 1 walk cycle
    }

    delay(50);
}

void moveLeg(float jointAngles[6][3]) {
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

void moveLegWalk(float jointAngles[6][5][3]) 
{   
    float jointAngles_mapped[3];

    for (int i = 0; i < 5; i++) 
    {   
        {
            for(int j = 0; j < 6; j++)
            {
                if(baseIDs[j] == 3 || baseIDs[j] == 15 || baseIDs[j] == 9)
                {
                    int baseID = baseIDs[j];
                    mapServoAngles(baseID, jointAngles[j][i], jointAngles_mapped);
                    
                    int pos1 = (int)jointAngles_mapped[0]; // Coxa
                    int pos2 = (int)jointAngles_mapped[1]; // Femur
                    int pos3 = (int)jointAngles_mapped[2]; // Tibia
            
                    sc.WritePos(baseID,     pos1, 0, 300);
                    sc.WritePos(baseID - 1, pos2, 0, 300);
                    sc.WritePos(baseID - 2, pos3, 0, 300);
                }
            }
        } delay(500);
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

