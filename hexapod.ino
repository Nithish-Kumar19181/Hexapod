#include<Arduino.h> 
#include"inverse_kinematics.h"
#include"ellipse_generation.h"
#include"gait_generation.h"
#include<SCServo.h> 

#define S_RXD 18
#define S_TXD 19

SCSCL sc;

void setup() 
{
    Serial.begin(115200);
    sc.pSerial = &Serial1;  
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
}
float JointAngles[6][3] ;

void loop() 
{
    float height = -15.0;
    bool success ;
  
    success = Stand(height, JointAngles);
    Serial.println(success) ;

    if (success) {
      moveLeg(7, JointAngles[0]);  // baseServoID = 1
    }

    else
    {
        Serial.println("The Stand Positions are unreachable for one or more legs.");
    }
  
    delay(3000);
  }

void moveLeg(int baseServoID, float jointAngles[3]) {

    float theta1 = jointAngles[0] ;
    float theta2 = jointAngles[1] ;
    float theta3 = jointAngles[2] ;
  
    int pos1 = mapAngleToPulse(theta1);
    int pos2 = mapAngleToPulse(theta2);
    int pos3 = mapAngleToPulse(theta3);
  
    sc.WritePos(9, pos1, 0, 300); // Coxa
    sc.WritePos(8, pos2, 0, 300); // Femur
    sc.WritePos(7, pos3, 0, 300); // Tibia

    Serial.print("pos1: ");
    Serial.println(pos1);
    Serial.print("pos2: ");
    Serial.println(pos2);
    Serial.print("pos3: ");
    Serial.println(pos3);
  }

  int mapAngleToPulse(float angleDeg) {
    angleDeg = constrain(angleDeg, 0, 360); 
    return (int)(angleDeg / 360.0 * 1023.0);
  }