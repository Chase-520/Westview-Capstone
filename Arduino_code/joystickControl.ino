const int lr_joystick = 4;
const int ud_joystick = 25;

const int pwm_1 = 0;
const int dir_1 = 2;

void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  // setup pin mode
  pinMode(pwm_1, OUTPUT);
  pinMode(dir_1, OUTPUT);

  //set the resolution to 12 bits (0-4095)
  analogReadResolution(12);
}

// Single function to control the motor
void setMotor(int speed, bool forwardDirection, int pwm_pin, int dir_pin) {
  // Set direction
  if (forwardDirection) {
    digitalWrite(dir_pin, HIGH);
  } else{
    digitalWrite(dir_pin, LOW);
  }
  
  analogWrite(pwm_pin, speed);

  //debug print
  Serial.print("speed is: "); Serial.print(speed);

}

void loop() {
  // read joystick input
  int LRjoystick = analogRead(lr_joystick);
  int UDjoystick = analogRead(ud_joystick);

  // map to pwm 
  int pwm_out = map(UDjoystick, 0, 4095, -255, 255);

  // control motor
  if(pwm_out > 0){
    // when positive
    setMotor(pwm_out, true, pwm_1, dir_1);
  } else{
    // when negative
    setMotor(abs(pwm_out), false, pwm_1, dir_1);
  }

  // debug print
  Serial.print("pwm out is: "); Serial.println(pwm_out);

  delay(100);  // delay in between reads for clear read from serial
}


