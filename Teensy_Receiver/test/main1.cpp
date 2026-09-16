#include <Arduino.h>



void setup() {
    Serial.begin(115200);    
    Serial4.begin(9600);    
   digitalWrite(LED_BUILTIN, HIGH);
   delay(5000);
    digitalWrite(LED_BUILTIN, LOW);
    
}

void loop() {
    Serial.println("Waiting for data from ESP32...");
    String data = Serial4.readStringUntil('\n');
    Serial.println("Received from ESP32: " + data);
    // if (Serial4.available()) {
    //     Serial.println("Waiting for data from ESP32...");
    //     String data = Serial4.readStringUntil('\n'); 
    //     Serial.println("Received from ESP32: " + data);
    //     if(data=="STOP") {
    //         Serial.println("Action: STOP");
          
    //     } else if(data=="TRIANGLE") {
    //         Serial.println("Action: TRIANGLE pressed");
    //          for(int i=0;i<2;i++)
    //          {
    //             digitalWrite(LED_BUILTIN, HIGH);
    //             delay(1000);
    //             digitalWrite(LED_BUILTIN, LOW);
    //             delay(1000);

    //          }
    //     } else if(data=="CIRCLE") {
    //         Serial.println("Action: CIRCLE pressed");
    //        for(int i=0;i<3;i++)
    //          {
    //             digitalWrite(LED_BUILTIN, HIGH);
    //             delay(1000);
    //             digitalWrite(LED_BUILTIN, LOW);
    //             delay(1000);
    //          }
    //     } else if(data=="SQUARE") {
    //         Serial.println("Action: SQUARE pressed");
    //        for(int i=0;i<4;i++)
    //          {
    //             digitalWrite(LED_BUILTIN, HIGH);
    //             delay(1000);
    //             digitalWrite(LED_BUILTIN, LOW);
    //             delay(1000);
    //          }
    //     } else if(data=="CROSS") {
    //         Serial.println("Action: CROSS pressed");
    //         for(int i=0;i<5;i++)
    //          {
    //             digitalWrite(LED_BUILTIN, HIGH);
    //             delay(1000);
    //             digitalWrite(LED_BUILTIN, LOW);
    //             delay(1000);
    //          }
    //     } else {
    //         Serial.println("Unknown command");
    //     }

    // }
}