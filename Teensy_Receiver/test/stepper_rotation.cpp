#include <NativeEthernet.h>
#include<NativeEthernetUdp.h>
#include<Arduino.h>
#include "teensystep4.h"

const uint8_t STEP_PIN_1 = 9;
const uint8_t DIR_PIN_1  = 8;

const uint8_t STEP_PIN_2 = 7;
const uint8_t DIR_PIN_2  = 6;

const uint8_t STEP_PIN_3 = 5;
const uint8_t DIR_PIN_3  = 4;

const uint8_t STEP_PIN_4= 3;
const uint8_t DIR_PIN_4 = 2;


void rotateSequence();
using namespace TS4;


Stepper stepper1(STEP_PIN_1, DIR_PIN_1);
Stepper stepper2(STEP_PIN_2, DIR_PIN_2);
Stepper stepper3(STEP_PIN_3, DIR_PIN_3);
Stepper stepper4(STEP_PIN_4, DIR_PIN_4);

// void applyCommand(const char* cmd)
// {
//      static int last_dir = 0;
//     int dir = 0;

//     if(strncmp(cmd, "LEFT", 4)==0)
//     {
//         dir = -1;
//     }
//     else if(strncmp(cmd, "RIGHT", 5)==0)
//     {
//         dir = 1;
//     }
//     else if(strncmp(cmd, "STOP", 4)==0)
//     {
//         dir = 0;
//     }

//     if (dir != 0)
//     {
//         if (stepperB.isMoving)
//         {
//             stepperB.emergencyStop();
//             // last_dir = 0;
//         }
//         if (dir != last_dir)
//         {
//             stepperB.rotateAsync(16000 * dir);
//             last_dir = dir;
//         }
//     }
//     else
//     {
//         if (stepperB.isMoving)
//         {
//             stepperB.emergencyStop();
//             last_dir = 0;
//         }
//     }

// }

void setup() 
{
    Serial.begin(115200);
    TS4::begin();


    // Configure each stepper
   
    //  stepperB.setMaxSpeed(900);
    // stepperB.setAcceleration(200);
    // // stepperB.setStepPulseLength(t); // Not supported in TeensyStep4 currently

    // stepperG.setMaxSpeed(16000);
    // stepperG.setAcceleration(500);
    // // stepperG.setStepPulseLength(15);
   
    stepper1.setMaxSpeed(900);
    stepper1.setAcceleration(200);
    stepper1.setPosition(0);

    stepper2.setMaxSpeed(900);
    stepper2.setAcceleration(200);
    stepper2.setPosition(0);

    stepper3.setMaxSpeed(900);
    stepper3.setAcceleration(200);
    stepper3.setPosition(0);

    stepper4.setMaxSpeed(900);
    stepper4.setAcceleration(200);
    stepper4.setPosition(0);
    delay(1);

}

void loop()
{
    
    rotateSequence(); 
    delay(2000);
}

void rotateSequence()
{
    const int32_t steps = 16000; 

    
    if (!stepper1.isMoving) stepper1.rotateAsync(steps);
    while (stepper1.isMoving) { delay(100); }

    
    if (!stepper2.isMoving) stepper2.rotateAsync(steps);
    while (stepper2.isMoving) { delay(100); }

    if (!stepper3.isMoving) stepper3.rotateAsync(steps);
    while (stepper3.isMoving) { delay(100); }

    if (!stepper4.isMoving) stepper4.rotateAsync(steps);
    while (stepper4.isMoving) { delay(100); }
}



