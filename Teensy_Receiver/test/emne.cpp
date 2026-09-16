#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <Arduino.h>
#include "ODriveCAN.h"
#include <FastLED.h>
#include <array>
#include <string>
#include <sstream>
#include <vector>
#include <cstring>
#include <cstdio>
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
#include <AccelStepper.h>
#include <MultiStepper.h>

#define CAN_BAUDRATE 1000000
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_intf;

// Create 4 ODrive objects with different node IDs
ODriveCAN odrv0(wrap_can_intf(can_intf), 0);

// User data for each ODrive
struct ODriveUserData {
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    bool available = false;
};

ODriveUserData odrv0_data;


#define LED_PIN 13

void onHeartbeat0(Heartbeat_msg_t &msg, void *user_data) {
    ODriveUserData *data = static_cast<ODriveUserData *>(user_data);
    data->last_heartbeat = msg;
    data->received_heartbeat = true;
}








void onCanMessage(const CAN_message_t &msg) {
    onReceive(msg, odrv0);
    
}



void setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200); 
     

    can_intf.begin();
    can_intf.setBaudRate(CAN_BAUDRATE);
    can_intf.setMaxMB(16);
    can_intf.enableFIFO();
    can_intf.enableFIFOInterrupt();
    can_intf.onReceive(onCanMessage);

    // Register heartbeat callbacks for each ODrive
    odrv0.onStatus(onHeartbeat0, &odrv0_data);
    

    Serial.println("Teensy started. Waiting for ODrive heartbeats...");
    
    // Try to detect each ODrive (5 attempts max)
    for (int attempt = 0; attempt < 5; attempt++) {
        unsigned long start_time = millis();
        while (millis() - start_time < 3000) {
            pumpEvents(can_intf);
            delay(2);
        }
        
        Serial.print("Attempt ");
        Serial.print(attempt + 1);
        Serial.println(":");
        Serial.print("  ODrive 0: ");
        Serial.println(odrv0_data.received_heartbeat ? "ONLINE" : "offline");
       
        
        // If all connected, break
        if (odrv0_data.received_heartbeat ) {
            break;
        }
        
        // Blink LED while waiting
        for (int i = 0; i < 5; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    }
    
    // Mark available ODrives and configure them
    if (odrv0_data.received_heartbeat) {
        odrv0_data.available = true;
        odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        odrv0.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        Serial.println("ODrive 0 enabled!");
    }
    
    
}

void loop() {
    pumpEvents(can_intf);  
    odrv0.setVelocity(20);
   
}