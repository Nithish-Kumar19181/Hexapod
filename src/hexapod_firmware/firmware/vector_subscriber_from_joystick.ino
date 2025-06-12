#include <Arduino.h>

int pin = 2 ; 
void setup() {
  Serial.begin(115200); // Initialize Serial for communication
  Serial.println("ESP32 ready to receive Vector3 data");
  pinMode(pin, OUTPUT) ; 
}

void loop() {
  // Check if data is available from the serial port
  if (Serial.available() > 0) {
    // Read incoming data
    String data = Serial.readStringUntil('\n'); // Read data until newline
    Serial.print("Received Vector3: ");
    Serial.println(data);

    // Parse the received data
    float x, y, z;
    sscanf(data.c_str(), "%f,%f,%f", &x, &y, &z);

    // Print the parsed values
    Serial.print("X: ");
    Serial.println(x);
    Serial.print("Y: ");
    Serial.println(y);
    Serial.print("Z: ");
    Serial.println(z);

    if (y >= 90 && y <= 0) {
      analogWrite(pin, HIGH) ; 
    }
    else {
      analogWrite(pin, LOW) ;
    }
    

    // Add additional logic to process the data
  }
//      analogWrite(pin, map(360, 0,360,0,3.3)) ; 

}
