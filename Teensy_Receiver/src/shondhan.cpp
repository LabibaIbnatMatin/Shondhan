#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <ArduinoJson.h> 
#include <SabertoothSimplified.h>
#include <PWMServo.h>


SabertoothSimplified ST(Serial4);


uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 68, 177); 
unsigned int localPort = 5005;  
PWMServo servo9;
PWMServo servo10;
PWMServo servo11;
PWMServo servo12;

EthernetUDP Udp;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; 

void setup() {
    Serial4.begin(9600);   
    Serial.begin(115200);
 
    // Attach the servos to physical pins
    servo9.attach(9);
    servo10.attach(10);
    servo11.attach(11);
    servo12.attach(12);

    servo9.write(70);
    servo10.write(90);
    servo11.write(90);
    servo12.write(90);

 
    
      

    
    
    Ethernet.begin(mac, ip);
    
    
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found. Cannot run without network hardware.");
        while (true) { delay(1); }
    }

    
    Udp.begin(localPort);
    Serial.print("Listening for ROS 2 UDP packets on IP: ");
    Serial.print(Ethernet.localIP());
    Serial.print(" Port: ");
    Serial.println(localPort);

    
    ST.motor(1, 0);
    ST.motor(2, 0);
}

void processPacket(char* jsonString) {
    
    
    StaticJsonDocument<64> doc;
    
    DeserializationError error = deserializeJson(doc, jsonString);
    if (error) {
        Serial.print("JSON Parsing failed: ");
        Serial.println(error.c_str());
        return;
    }

    
    int linear_pwm  = doc[0]; 
    int angular_pwm = doc[1];

    
    if (linear_pwm == 0 && angular_pwm == 0) return; 

    
    int throttle = map(linear_pwm, 1000, 2000, -50, 50);
    int steering = map(angular_pwm, 1000, 2000, -50, 50);

    // 2. Differential Drive Mixing Logic
    int motorLeft  = throttle + steering;
    int motorRight = throttle - steering;

    
    motorLeft  = constrain(motorLeft, -127, 127);
    motorRight = constrain(motorRight, -127, 127);

    
    ST.motor(1, motorLeft);  // Left Side Motors
    ST.motor(2, motorRight); // Right Side Motors

    // Debugging print to standard serial monitor
    Serial.print("RX PWM -> Lin: "); Serial.print(linear_pwm);
    Serial.print(" | Ang: "); Serial.print(angular_pwm);
    Serial.print(" -> Motors -> L: "); Serial.print(motorLeft);
    Serial.print(" | R: "); Serial.println(motorRight);
}

void loop() {
    // Check if a network packet has arrived
    int packetSize = Udp.parsePacket();
    if (packetSize) {
        // Read the packet contents into the buffer
        int len = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
        if (len > 0) {
            packetBuffer[len] = 0; // Null-terminate string array safely
        }
        
        // Process the JSON string
        processPacket(packetBuffer);
    }
}
