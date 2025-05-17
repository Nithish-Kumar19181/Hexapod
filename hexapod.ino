#include<Arduino.h> 
#include"inverse_kinematics.h"
#include"ellipse_generation.h"
#include"gait_generation.h"
#include"servo_mapping.h"

#include<SCServo.h> 

#define S_RXD 18
#define S_TXD 19

SCSCL sc;

float height = -12.0; // Initial height (negative as per your convention)
const float maxHeight = -20.0;
const float minHeight = -12.0;
float JointAngles[6][3] ;
float JointAngles_mapped[6][3] ;

void setup() 
{
    Serial.begin(115200);
    sc.pSerial = &Serial1;  
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
}

void loop() 
{
    handleSerialInput();
    bool success ;
  
    success = Stand(height, JointAngles);
    Serial.println(success) ;

    if (success) {
        moveLeg(3, JointAngles[0]);  // baseServoID = 3
        Serial.println("Leg 1 moved successfully");
        moveLeg(18, JointAngles[1]);  // baseServoID = 18
        Serial.println("Leg 2 moved successfully");
        moveLeg(15, JointAngles[2]);  // baseServoID = 15
        Serial.println("Leg 3 moved successfully");
        moveLeg(12, JointAngles[3]);  // baseServoID = 12
        Serial.println("Leg 4 moved successfully");
        moveLeg(9, JointAngles[4]);  // baseServoID = 9
        Serial.println("Leg 5 moved successfully");
        moveLeg(6, JointAngles[5]);  // baseServoID = 6
        Serial.println("Leg 6 moved successfully");

    }

    else
    {
        Serial.println("The Stand Positions are unreachable for one or more legs.");
    }
  
    //delay(3000);
  }
// clubbing the 3 joints of a leg by getting the baseID 
void moveLeg(int baseServoID, float jointAngles[3]) {

    float jointAngles_mapped[3];
    mapServoAngles(baseServoID, jointAngles, jointAngles_mapped);
    Serial.print(height);
    int pos1 = (int)jointAngles_mapped[0]; // Coxa
    int pos2 = (int)jointAngles_mapped[1]; // Femur
    int pos3 = (int)jointAngles_mapped[2]; // Tibia
  
    sc.WritePos(baseServoID,     pos1, 0, 300); // Coxa
    sc.WritePos(baseServoID - 1, pos2, 0, 300); // Femur
    sc.WritePos(baseServoID - 2, pos3, 0, 300); // Tibia

    Serial.print("pos1: ");
    Serial.println(pos1);
    Serial.print("pos2: ");
    Serial.println(pos2);
    Serial.print("pos3: ");
    Serial.println(pos3);
  }

  void handleSerialInput() {
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 'h') {
            if (height > maxHeight) {
                height -= 0.3;
                Serial.print("Increased height to: ");
                Serial.println(-height);
            } else {
                Serial.println("Max height reached.");
            }
        } else if (cmd == 'd') {
            if (height < minHeight) {
                height += 0.3;
                Serial.print("Decreased height to: ");
                Serial.println(-height);
            } else {
                Serial.println("Min height reached.");
            }
        }
    }
}
