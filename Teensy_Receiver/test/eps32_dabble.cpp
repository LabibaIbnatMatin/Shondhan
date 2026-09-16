
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h> 

#define ESP_TX 16  
#define ESP_RX 17  
#define SERIAL_BAUD 9600
#define TILT_DEADZONE 10  

void setup() {
  Serial.begin(115200);      
  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, 16, 17); 

  Dabble.begin("Rover_ESP32");

}

// Static variables for smooth arm ramping
static int currentArmX = 0;
static int currentArmY = 0;
const int ARM_MAX_SPEED = 5;      // Reduced max speed for safety (was 7)
const int ARM_ACCEL_STEP = 1;     // Gradual acceleration per loop

void loop() {
  Dabble.processInput(); 
  
  
  float joyX = GamePad.getXaxisData();  // -7 to +7
  float joyY = GamePad.getYaxisData();  // -7 to +7
  
  int wheelX = (int)joyX;
  int wheelY = (int)joyY;
  
 
  /
  int targetArmX = 0;  
  int targetArmY = 0;
  

  if(GamePad.isTrianglePressed()) targetArmY = ARM_MAX_SPEED;
  else if(GamePad.isCrossPressed()) targetArmY = -ARM_MAX_SPEED;


  if(GamePad.isCirclePressed()) targetArmX = ARM_MAX_SPEED;
  else if(GamePad.isSquarePressed()) targetArmX = -ARM_MAX_SPEED;
  
  
  if(currentArmY < targetArmY) currentArmY = min(currentArmY + ARM_ACCEL_STEP, targetArmY);
  else if(currentArmY > targetArmY) currentArmY = max(currentArmY - ARM_ACCEL_STEP, targetArmY);

  if(currentArmX < targetArmX) currentArmX = min(currentArmX + ARM_ACCEL_STEP, targetArmX);
  else if(currentArmX > targetArmX) currentArmX = max(currentArmX - ARM_ACCEL_STEP, targetArmX);
  
  int armX = currentArmX;
  int armY = currentArmY;

  
int lifter_mode=1;
 if(GamePad.isUpPressed() && GamePad.isTRianglePressed()) lifter_mode=2;
 else if(GamePad.isDownPressed() && GamePad.isCrossPressed()) lifter_mode = 3;
 else lifter_mode =1;

  int ghoraVal = 0;
  int gripperVal = 0;
  int baseVal = 0;
  int lifterMode = 1;
  

  if (GamePad.isStartPressed()) gripperVal = 7;
  else if (GamePad.isSelectPressed()) gripperVal = -7;
  
  if (GamePad.isLeftPressed ()) baseVal = -7;
  else if (GamePad.isRightPressed()) baseVal = 7;
  
  if (GamePad.isUpPressed()) ghoraVal = 7;
  else if (GamePad.isDownPressed()) ghoraVal = -7;
  

  String data = String(wheelX) + "," + String(wheelY) + "," +
                String(armX) + "," + String(armY) + "," +
                String(ghoraVal) + "," + String(gripperVal) + "," +
                String(baseVal) + "," + String(lifterMode);
  
  if(Dabble.isConnected()) Serial2.println(data);
  else
  {
    
    data = "[0,0,0,0,0,0,0,1]";
    Serial2.println(data);
  }
  
  
  
  
  delay(50); 
}