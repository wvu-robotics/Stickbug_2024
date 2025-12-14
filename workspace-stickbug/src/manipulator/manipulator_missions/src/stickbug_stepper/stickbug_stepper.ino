#include <Arduino.h>

const int stepPin = 2; // Define the stepper motor step pin
const int dirPin = 3;  // Define the stepper motor direction pin
long stepper_delay = 200; // Define the delay between steps

void setup() {
  Serial.begin(9600);      // Start serial communication at 9600 baud rate
  pinMode(stepPin, OUTPUT); // Set the step pin as output
  pinMode(dirPin, OUTPUT);  // Set the direction pin as output
}

void loop() {
  if (Serial.available() > 0) {
    // Read the serial input as a signed integer
    //long num_steps = Serial.parseInt();

    String command = Serial.readStringUntil('\n');
    int commaIndex = command.indexOf(',');
    
    // Parse delta_steps and stepper_delay from the command
    long num_steps = command.substring(0, commaIndex).toInt();
    stepper_delay = command.substring(commaIndex + 1).toInt();

    // Set direction based on the sign of num_steps
    digitalWrite(dirPin, num_steps >= 0 ? HIGH : LOW);

    // Move the stepper motor for the absolute value of num_steps
    for (int i = 0; i < abs(num_steps); i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepper_delay); 
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepper_delay); // Modify this delay to control the speed of the stepper
    }

    // Write the number of steps to the serial port
    Serial.println(num_steps);
  }
}
