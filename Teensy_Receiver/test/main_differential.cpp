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
#include <SabertoothSimplified.h>

SabertoothSimplified ST(Serial2);

void setup()
{
    Serial2.begin(9600);  // Sabertooth
    Serial.begin(115200); // Data input
    ST.motor(1, 0);
    ST.motor(2, 0);
}

void processdata(String s)
{
   
    // Split the string
    int commaIndex = s.indexOf(',');
    if (commaIndex == -1)
        return;
    String tmp = s.substring(1, commaIndex);
    int x = tmp.toInt();
    int y = s.substring(commaIndex + 1, s.length() - 1).toInt();

    // Map joystick values 1000–2000 to motor -127 to 127
    int motor1, motor2;

    // Motor 1 (x axis)
    if (x > 1500)
        motor1 = map(x, 1500, 2000, 0, 126);
    else
        motor1 = map(x, 1000, 1500, -126, 0);

    // Motor 2 (y axis)
    if (y > 1500)
        motor2 = map(y, 1500, 2000, 0, 126);
    else
        motor2 = map(y, 1000, 1500, -126, 0);

    // Send to Sabertooth
    ST.motor(1, motor1);
    ST.motor(2, motor2);
    Serial.print("X=");
    Serial.print(x);
    Serial.print("  Y=");
    Serial.print(y);
    Serial.print("  M1=");
    Serial.print(motor1);
    Serial.print("  M2=");
    Serial.println(motor2);
}

void loop()
{
    String s = "";
    while (Serial.available())
    {
        s = Serial.readStringUntil('\n');
        if (s.startsWith("[") && s.endsWith("]"))
        {
            processdata(s);
        }
    }
}
/*
[1000,2000]
[2000,1000]
[1600,1600]


*/