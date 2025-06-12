#include <Arduino.h>

int pin = 2;

void setup() {
  Serial.begin(115200); // Initialize Serial for communication
  Serial.println("ESP32 ready to receive Vector6 data");
  pinMode(pin, OUTPUT);
}

void loop() {
  // Check if data is available from the serial port
  if (Serial.available() > 0) {
    // Read incoming data
    String data = Serial.readStringUntil('\n'); // Read data until newline
    Serial.print("Received Vector6: ");
    Serial.println(data);

    // Parse the received data (Vector of length 6)
    float values[6] = {0};
    int parsed = sscanf(data.c_str(), "%f,%f,%f,%f,%f,%f",
                        &values[0], &values[1], &values[2],
                        &values[3], &values[4], &values[5]);

    // Check if parsing was successful
    if (parsed == 6) {
      // Print the parsed values
      for (int i = 0; i < 6; i++) {
        Serial.print("Value[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.println(values[i]);
      }

      // Example logic: Use Y value (values[1]) to control pin
      if (values[1] >= 90) {
        digitalWrite(pin, HIGH);
      } else {
        digitalWrite(pin, LOW);
      }

      // Additional logic: Map and write analog value from the first element
      int pwmValue = map((int)values[0], 0, 360, 0, 255); // Map 0-360 to 0-255
      analogWrite(pin, pwmValue); // Write PWM signal to pin
      Serial.print("Mapped PWM Value: ");
      Serial.println(pwmValue);
    } else {
      Serial.println("Error: Failed to parse Vector6 data");
    }
  }
}
