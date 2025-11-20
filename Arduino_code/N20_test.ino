#include <Encoder.h>

// Motor control pins
const int PWMA = 5;    // PWM speed control
const int AIN1 = 7;    // Direction 1
const int AIN2 = 8;    // Direction 2

// Encoder
Encoder myEncoder(2, 3);

void setup() {
  Serial.begin(9600);
  
  // Set motor pins as outputs
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
}

void loop() {
  // Read encoder position
  long encoderPos = myEncoder.read();
  Serial.print("Position: ");
  Serial.println(encoderPos);
  
  // Example: Move forward at half speed for 2 seconds
  motorControl(255, 1);  // Speed, Direction (1=forward)
  delay(2000);
  
  // Read encoder position
  encoderPos = myEncoder.read();
  Serial.print("Position: ");
  Serial.println(encoderPos);

  // Stop for 1 second
  motorControl(0, 0);
  delay(1000);
  
  // Move backward at half speed for 2 seconds
  motorControl(255, -1); // Speed, Direction (-1=backward)
  delay(2000);

  // Read encoder position
  encoderPos = myEncoder.read();
  Serial.print("Position: ");
  Serial.println(encoderPos);
}

void motorControl(int speed, int direction) {
  // Limit speed to 0-255
  speed = constrain(speed, 0, 255);
  
  if (direction == 1) { // Forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  }
  else if (direction == -1) { // Backward
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, speed);
  }
  else { // Stop/Brake
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}