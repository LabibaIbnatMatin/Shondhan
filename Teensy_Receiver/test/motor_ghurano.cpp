#include <FlexCAN_T4.h>


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

const uint32_t MOTOR_ID = 1;
const uint32_t SEND_ID = 0x140 + MOTOR_ID; 

void sendCommand(uint8_t* data) {
  CAN_message_t msg;
  msg.id = SEND_ID;
  msg.len = 8; 
  for (int i = 0; i < 8; i++) msg.buf[i] = data[i];
  bool success = can1.write(msg);
  if (!success) {
    Serial.println("Failed to send command.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

 
  can1.begin();
  can1.setBaudRate(1000000); 
  Serial.println("CAN Initialized at 1Mbps.");
  
  uint8_t systemReset[9] = {0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendCommand(systemReset);
  Serial.println("System Reset Sent. Waiting for reboot...");
  delay(2000); 

  
  uint8_t brakeRelease[8] = {0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendCommand(brakeRelease);
  Serial.println("Brake Released.");
  delay(100); 

  uint8_t rotateCmd[8] = {
    0xA4,       // Command Byte [9]
    0x00,       // NULL [9]
    0x64, 0x00, // Speed Limit: 100 dps (Low Byte, High Byte) [9]
    0x28, 0x23, 0x00, 0x00 // Target Angle: 9000 (Low to High Byte) [9]
  };

  sendCommand(rotateCmd);
  Serial.println("Rotation command sent: 90 Degrees.");

  
 
}

void loop() {
  


  
  CAN_message_t incoming;
  if (can1.read(incoming)) {
    if (incoming.id == (0x240 + MOTOR_ID)) {
      
      int16_t currentAngle = (int16_t)(incoming.buf[2] | (incoming.buf[3] << 8));
      Serial.print("Current Angle: ");
      Serial.println(currentAngle);
    }
  }
}