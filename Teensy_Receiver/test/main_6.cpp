#include <Arduino.h>
#include <SabertoothSimplified.h>
#include <string>

SabertoothSimplified ST(Serial4);

void setup()
{
    Serial4.begin(9600);  

    Serial.begin(115200); 
    
    
    ST.motor(1, 0);
    ///ST.motor(2, 0);
    delay(1000);        
}

void loop()
{
    
    int staticSpeedM1 = 60;   
    int staticSpeedM2 = -60;  

    
    ST.motor(1, staticSpeedM1);
    ST.motor(2, staticSpeedM2);

    
    Serial.print("Running Static Speed - M1: ");
    Serial.print(staticSpeedM1);
    Serial.print(" | M2: ");
    Serial.println(staticSpeedM2);

    delay(100); 
}
