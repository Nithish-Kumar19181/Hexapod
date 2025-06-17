#include <Arduino.h>
#include "inverse_kinematics.h"
#include "ellipse_generation.h"
#include "gait_generation.h"
#include "servo_mapping.h"
#include "move_leg.h"
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
                currentMode = STAND; 
            }
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
                        next_char = Serial.read(); 
                    }
                    if (next_char == 'A') {
                        Serial.println("Received: LA_combo");
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