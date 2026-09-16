// #include <Arduino.h>
// #include <Servo.h>

// Servo myServo;       
// const int servoPin = 9; 

// void setup() {
//   Serial.begin(115200);
//   myServo.attach(servoPin); 
//   Serial.println("Servo initialized");
//   //myServo.writeMicroseconds(1500); 
//   myServo.write(0);
//   Serial.println("Servo set to center position");        
// }

// void loop() {
//   // here there will be something 
//   Serial.println("Looping...");
// }
#include<Arduino.h>
#include <PWMServo.h>

PWMServo ds32Servo; 
const int servoPin = 9; // Use any Teensy 4.1 PWM Pin

void setup() {
  Serial.begin(115200);
  ds32Servo.attach(servoPin, 500, 2500);
             // give Serial time to connect
  Serial.println("Before move");
  ds32Servo.write(90);
  Serial.println("After move — should print ONCE if no reset loop");
}


void loop() {
  // Use standard angles 0 to 180 (or 0 to 270 if you bought the 270° model)
  
}

