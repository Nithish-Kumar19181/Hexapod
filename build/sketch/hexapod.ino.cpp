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

 bool stopWalkFlag = false; 
 float TiltAngle = 0.0f;

int baseIDs[6] = {3, 18, 15, 12, 9, 6};
float height = -14.0;
const float maxHeight = -29.0;
const float minHeight = -5;
float RotateAngle = 0.0f; 
float Angle = 0.0f;

float JointAngles[6][3];
float JointAnglesLine[6][3];
float walkAngles[6][5][3];  
float walkAnglesLine[6][5][3]; 
float rotateAngles[6][5][3] ; 
float rotateAnglesLine[6][5][3] ;


enum BotMode { IDLE, STAND, WALK, ROTATE , TILT };
BotMode currentMode = IDLE;

#line 34 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void setup();
#line 40 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void loop();
#line 116 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void moveLegStand(float jointAngles[6][3]);
#line 137 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void moveLegWalk(float jointAngles[6][5][3], float jointAnglesLine[6][5][3]);
#line 213 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void handleSerialInput();
#line 34 "/home/nithish/hexapod/Hexapod/hexapod_main/hexapod/hexapod/hexapod/hexapod/hexapod.ino"
void setup() {
    Serial.begin(115200);
    sc.pSerial = &Serial1;
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
}

void loop() 
{
    handleSerialInput();

    if (currentMode == STAND) {
        if (Stand(height, JointAngles)) {
            moveLegStand(JointAngles);
        } else {
            Serial.println("Stand IK failed for one or more legs.");
        }
    }

    else if (currentMode == WALK) {
        if (stopWalkFlag) {
            currentMode = STAND; 
            stopWalkFlag = false; 
            Serial.println("Walk interrupted. Mode: STAND");
        } else {
            if (WalkGait(height, walkAngles, walkAnglesLine, Angle)) {
                moveLegWalk(walkAngles, walkAnglesLine);

                currentMode = STAND; 
            } else {
                Serial.println("WalkGait IK failed.");
                currentMode = STAND; 
            }
        }
    }

    else if (currentMode == ROTATE) 
    {
        if (stopWalkFlag) { 
            currentMode = STAND;
            stopWalkFlag = false;
            Serial.println("Rotate interrupted. Mode: STAND");
        } 
        else 
        {
            if (RotateHexa(height, RotateAngle, rotateAngles, rotateAnglesLine)) {
                Serial.println(RotateAngle);
                moveLegWalk(rotateAngles, rotateAnglesLine);
                currentMode = STAND;
            } 
            else 
            {
                Serial.println("Rotate IK failed.");
                currentMode = STAND; 
            }
        }
    }

    else if (currentMode == TILT)
    {
        Serial.println("Tilt mode active");
        delay(0);
        if (stopWalkFlag) { 
            currentMode = STAND;
            stopWalkFlag = false;
            Serial.println("Rotate interrupted. Mode: STAND");
        } 
        else 
        {
            if(Tilt(height, JointAngles, TiltAngle))
            {
                moveLegStand(JointAngles);
                currentMode = STAND;
            }
            else 
            {
                Serial.println("Rotate IK failed.");
                currentMode = STAND; // Revert to stand on failure
            }
        }
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

void handleSerialInput() {
    if (Serial.available()) {
        char cmd_char = Serial.peek(); // Peek to check the first character

        if (cmd_char == 'w' || cmd_char == 'r' || cmd_char == 't') 
        {
            char cmd = Serial.read(); 
            if (Serial.available()) {
               
                while (Serial.available() && !isDigit(Serial.peek()) && Serial.peek() != '-' && Serial.peek() != '.') {
                    Serial.read(); 
                }
                float received_value = Serial.parseFloat(); // This will read until a non-float char or timeout

                if (cmd == 'w') {
                    if (currentMode == STAND || currentMode == WALK) {
                        currentMode = WALK;
                        Angle = received_value;
                        stopWalkFlag = false; // Ensure walk continues
                        Serial.print("Mode: WALK (Tripod gait), Angle: ");
                        Serial.println(Angle);
                    } else {
                        Serial.println("Start walking only from STAND or WALK mode.");
                    }
                } 
                else if (cmd == 'r')
                { 
                    if (currentMode == STAND || currentMode == ROTATE) {
                        currentMode = ROTATE;
                        RotateAngle = received_value;
                        stopWalkFlag = false; // Ensure rotate continues
                        Serial.print("Mode: ROTATE (Tripod gait), Angle: ");
                        Serial.println(RotateAngle);
                    } else {
                        Serial.println("Start rotating only from STAND or ROTATE mode.");
                    }
                }

                else if (cmd == 't') 
                {
                    if (currentMode == STAND || currentMode == TILT) {
                        currentMode = TILT ;
                        TiltAngle = received_value;
                        stopWalkFlag = false; // Ensure rotate continues
                        Serial.println(TiltAngle);
                    } else {
                        Serial.println("Height change only allowed in STAND mode.");
                    }
                } 
                else 
                {
                    Serial.print("Unknown command: ");
                    Serial.println(cmd);
                }
            } 

            else
            {
                Serial.print("Command '");
                Serial.print(cmd);
                Serial.println("' received without argument. Ignoring.");
            }
        } 
        else 
        {
            // Handle single character commands (s, h, d, o, L, A etc.)
            char cmd = Serial.read(); 

            switch (cmd) {
                case 's':
                    currentMode = STAND;
                    stopWalkFlag = true;
                    Serial.println("Mode: STAND");
                    break;

                case 'o':
                    stopWalkFlag = true;
                    Serial.println("Command: HALT/PAUSE (o)");
                    break;
                
                case 'h':
                    if (currentMode == STAND && height > maxHeight) { 
                        height -= 0.3; 
                        Serial.print("Increased height to: ");
                        Serial.println(height); 
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
                        Serial.println(height); 
                    } else if (currentMode != STAND) {
                        Serial.println("Height change only allowed in STAND mode.");
                    } else {
                        Serial.println("Min height reached.");
                    }
                    break;

                case 'L': 
                    char next_char = ' ';
                    if (Serial.available()) {
                        next_char = Serial.read(); // Read the next character
                    }
                    if (next_char == 'A') {
                        Serial.println("Received: LA_combo");
                        // Specific actions for LB + A combo
                    } else {
                        Serial.print("Unknown command: ");
                        Serial.println(cmd);
                        if (next_char != ' ') { 
                            Serial.println(next_char);
                        }
                    }
                    break;
            }
        }
        while (Serial.available()) {
            Serial.read();
        }
    }
}
