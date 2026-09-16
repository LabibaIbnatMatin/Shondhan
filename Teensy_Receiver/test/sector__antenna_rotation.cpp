#include <NativeEthernet.h>
#include<NativeEthernetUdp.h>
#include<Arduino.h>
#include "teensystep4.h"

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress myIP(192,168,1,177);
const IPAddress allowedIP(192,168,1,11);
const unsigned int port = 8888;

EthernetUDP Udp;

const  uint8_t STEP_PIN = 1;
const uint8_t DIR_PIN = 3;

using namespace TS4;

Stepper stepperB(1, 3);


void applyCommand(const char* cmd)
{
     static int last_dir = 0;
    int dir = 0;

    if(strncmp(cmd, "LEFT", 4)==0)
    {
        dir = -1;
    }
    else if(strncmp(cmd, "RIGHT", 5)==0)
    {
        dir = 1;
    }
    else if(strncmp(cmd, "STOP", 4)==0)
    {
        dir = 0;
    }

    if (dir != 0)
    {
        if (stepperB.isMoving)
        {
            stepperB.emergencyStop();
            // last_dir = 0;
        }
        if (dir != last_dir)
        {
            stepperB.rotateAsync(16000 * dir);
            last_dir = dir;
        }
    }
    else
    {
        if (stepperB.isMoving)
        {
            stepperB.emergencyStop();
            last_dir = 0;
        }
    }

}

void setup() 
{
    Serial.begin(115200);
    TS4::begin();
    Ethernet.begin(mac, myIP);
    delay(100);
    Udp.begin(port);
    Serial.println("Local IP Address: ");
    Serial.println(Ethernet.localIP());


    stepperB.setMaxSpeed(16000);
    stepperB.setAcceleration(500);
    stepperB.setPosition(0);

}

void loop()
{
    int packetSize = Udp.parsePacket();
    Serial.println(packetSize);
    // delay(1000);
    if (packetSize > 0)
    {
        IPAddress src = Udp.remoteIP();
        uint16_t srcPort = Udp.remotePort();

        if (src != allowedIP)
        {
            // discard payload
            while (Udp.available()) { uint8_t tmp; Udp.read(&tmp, 1); }
            Serial.print("Ignored packet from ");
            Serial.println(src);
        }
        else
        {
            char buf[128] = {0};
            int len = Udp.read((uint8_t*)buf, sizeof(buf) - 1);
            if (len > 0)
            {
                buf[len] = '\0';
                applyCommand(buf);
                Serial.print("CMD from ");
                Serial.print(src);
                Serial.print(':');
                Serial.println(buf);
            }
        }
    }
    
    }



