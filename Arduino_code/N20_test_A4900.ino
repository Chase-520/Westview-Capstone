// Single Motor Test - Use Motor A only
const int MOTOR_DIR = 13;  // AIN1 (Direction pin 1)
const int MOTOR_PWM = 12;   // PWMA (Speed control)

void setup() {
  // Set motor pins as outputs
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("A4990 Single Motor Test");
  Serial.println("Starting in 2 seconds...");
  delay(2000);
}

void loop() {
  // Test 1: Forward
  Serial.println("Forward - Medium speed");
  setMotor(150, true);  // Forward, speed 150/255
  delay(2000);
  
  // Test 2: Faster forward
  Serial.println("Forward - Full speed");
  setMotor(255, true);  // Forward, full speed
  delay(2000);
  
  // Test 3: Stop briefly
  Serial.println("Stop");
  setMotor(0, true);    // Stop
  delay(500);
  
  // Test 4: Backward
  Serial.println("Backward - Medium speed");
  setMotor(150, false); // Backward, speed 150/255
  delay(2000);
  
  // Test 5: Stop longer
  Serial.println("Stop for 3 seconds");
  setMotor(0, true);    // Stop
  delay(500);
}

// Single function to control the motor
void setMotor(int speed, bool forwardDirection) {
  // Set direction
  if (forwardDirection) {
    digitalWrite(MOTOR_DIR, HIGH);
  } else{
    digitalWrite(MOTOR_DIR, LOW);
  }
  
  analogWrite(MOTOR_PWM, speed);
}