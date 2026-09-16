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
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_intf;

#define NUM_LEDS 60
#define DATA_PIN 33


CRGB leds[NUM_LEDS];


//            For Autonomous Testing 

const int ledPin = LED_BUILTIN;

#define CAN_BAUDRATE 500000
#define ODRV0_NODE_ID0 0
#define ODRV0_NODE_ID1 1
#define ODRV0_NODE_ID2 2
#define ODRV0_NODE_ID3 3
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID0);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV0_NODE_ID1);
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV0_NODE_ID2);
ODriveCAN odrv3(wrap_can_intf(can_intf), ODRV0_NODE_ID3);
ODriveCAN *odrives[] = {&odrv0, &odrv1, &odrv2, &odrv3};


void showgreen();
void showred();
void showblue();
void fillStrip(uint32_t color);

struct ODriveUserData
{
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback = false;
    bool available = false;
};

ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;
ODriveUserData odrv2_user_data;
ODriveUserData odrv3_user_data;
enum
{
    POSITION_MODE = 0,
    VELOCITY_MODE = 1,
} control_mode0,
    control_mode1, control_mode2, control_mode3;

void onHeartbeat(Heartbeat_msg_t &msg, void *user_data)
{
    ODriveUserData *odrv_user_data = static_cast<ODriveUserData *>(user_data);
    odrv_user_data->last_heartbeat = msg;
    odrv_user_data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t &msg, void *user_data)
{
    ODriveUserData *odrv_user_data = static_cast<ODriveUserData *>(user_data);
    odrv_user_data->last_feedback = msg;
    odrv_user_data->received_feedback = true;
}

void onCanMessage(const CanMsg &msg)
{
    for (auto odrive : odrives)
    {
        onReceive(msg, *odrive);
    }
}

bool setupCan()
{
    can_intf.begin();
    can_intf.setBaudRate(CAN_BAUDRATE);
    can_intf.setMaxMB(16);
    can_intf.enableFIFO();
    can_intf.enableFIFOInterrupt();
    can_intf.onReceive(onCanMessage);
    return true;
}
void fillStrip(uint32_t color)
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = color;
    }
    FastLED.show();
}
void showRed()
{
    // disco();
    fillStrip(CRGB::Red);
}

void showGreen()
{
    fillStrip(CRGB::Green);
}

void showBlue()
{
    fillStrip(CRGB::Blue);
}
void setup()
{

   
    Serial.begin(115200); // Data input
    // setupCan();
    // for(int i=0;i<5;i++)
    // {
    //     digitalWrite(ledPin, HIGH);
    //     delay(200);
    //     digitalWrite(ledPin, LOW);
    //     delay(200);
    // }

    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    FastLED.show();

    odrv0.onFeedback(onFeedback, &odrv0_user_data);
    odrv0.onStatus(onHeartbeat, &odrv0_user_data);
    odrv1.onFeedback(onFeedback, &odrv1_user_data);
    odrv1.onStatus(onHeartbeat, &odrv1_user_data);
    odrv2.onFeedback(onFeedback, &odrv2_user_data);
    odrv2.onStatus(onHeartbeat, &odrv2_user_data);
    odrv3.onFeedback(onFeedback, &odrv3_user_data);
    odrv3.onStatus(onHeartbeat, &odrv3_user_data);
        pinMode(ledPin, OUTPUT);
        digitalWrite(ledPin, HIGH);
    if (!setupCan())
    {
        while (true)
            ;
    }

    unsigned long start_time = millis();
    const unsigned long max_wait_time = 3000;

    while (millis() - start_time < max_wait_time)
    {
        pumpEvents(can_intf);
        delay(2);

        if (odrv0_user_data.received_heartbeat || odrv1_user_data.received_heartbeat ||
            odrv2_user_data.received_heartbeat || odrv3_user_data.received_heartbeat
    )
        {
            break;
        }
    }

    odrv0_user_data.available = odrv0_user_data.received_heartbeat;
    odrv1_user_data.available = odrv1_user_data.received_heartbeat;
    odrv2_user_data.available = odrv2_user_data.received_heartbeat;
    odrv3_user_data.available = odrv3_user_data.received_heartbeat;
   

    start_time = millis();
    const unsigned long init_timeout = 2000;

    while (millis() - start_time < init_timeout)
    {
        if (odrv0_user_data.available && odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
        {
            odrv0.clearErrors();
            delay(1);
            odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        }

        if (odrv1_user_data.available && odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
        {
            odrv1.clearErrors();
            delay(1);
            odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        }

        if (odrv2_user_data.available && odrv2_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
        {
            odrv2.clearErrors();
            delay(1);
            odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        }

        if (odrv3_user_data.available && odrv3_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
        {
            odrv3.clearErrors();
            delay(1);
            odrv3.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        }

       
        for (int i = 0; i < 10; ++i)
        {
            pumpEvents(can_intf);
            delay(2);
        }

        bool all_ready = true;
        if (odrv0_user_data.available && odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
            all_ready = false;
        if (odrv1_user_data.available && odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
            all_ready = false;
        if (odrv2_user_data.available && odrv2_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
            all_ready = false;
        if (odrv3_user_data.available && odrv3_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
            all_ready = false;
        

        if (all_ready)
            break;
    }

    if (odrv0_user_data.available)
    {
        odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        control_mode0 = VELOCITY_MODE;
    }

    if (odrv1_user_data.available)
    {
        odrv1.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        control_mode1 = VELOCITY_MODE;
    }

    if (odrv2_user_data.available)
    {
        odrv2.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        control_mode2 = VELOCITY_MODE;
    }

    if (odrv3_user_data.available)
    {
        odrv3.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        control_mode3 = VELOCITY_MODE;
    }
  
    // showGreen();

     digitalWrite(ledPin, LOW);
    

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

    // float left_speed_norm = y + x;
    // float right_speed_norm = y - x;

    
    float left_motor = constrain(x, 1000, 2000);
    float right_motor = constrain(y, 1000, 2000);

    // patch
    // if (left_motor > 1500)
    // {
    //     left_motor = 1500 - abs(1500 - left_motor);
    // }
    // else
    // {
    //     left_motor = 1500 + abs(1500 - left_motor);
    // }

    // if (right_motor > 1500)
    // {
    //     right_motor = 1500 - abs(1500 - right_motor);
    // }
    // else
    // {
    //     right_motor = 1500 + abs(1500 - right_motor);
    // }

    int pwmMotor0 = map(left_motor, 1000, 2000, -53, 53);

    
    
    int pwmMotor1 = map(right_motor, 1000, 2000, -53, 53);

    if (odrv1_user_data.available)
    {
        odrv1.setVelocity(-1*pwmMotor1);
    }


    if (odrv3_user_data.available)
    {
        odrv3.setVelocity(pwmMotor0);
    }

    if (odrv2_user_data.available)
    {
        odrv2.setVelocity(pwmMotor0);
    }

    if (odrv0_user_data.available)
    {
        odrv0.setVelocity(-1*pwmMotor1);
    }

    
    Serial.print("X=");
    Serial.print(x);
    Serial.print("  Y=");
    Serial.print(y);
    Serial.print("  M1=");
    Serial.print(pwmMotor0);
    Serial.print("  M2=");
    Serial.println(pwmMotor1);
}
void process_led_data(String s)
{
 
 if(s[1]=='g')
 {
  showGreen();
  Serial.println("Green");
 }
 else if(s[1]=='r')
 {
  showRed();
  Serial.println("Red");
 }
 else 
 {
  showBlue();
  Serial.println("Blue");
 }
 }



void loop()
{
    // copy form main kaj holo jodi initiall na hoi abr bar bar setup korbe
     static unsigned long last_odrive_check = 0;
    if (millis() - last_odrive_check > 5000)
    {
        last_odrive_check = millis();

        pumpEvents(can_intf);

        if (!odrv0_user_data.available && odrv0_user_data.received_heartbeat)
        {
            odrv0_user_data.available = true;
            odrv0.clearErrors();
            delay(5);
            odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
            control_mode0 = VELOCITY_MODE;
        }
        if (!odrv1_user_data.available && odrv1_user_data.received_heartbeat)
        {
            odrv1_user_data.available = true;
            odrv1.clearErrors();
            delay(5);
            odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv1.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
            control_mode1 = VELOCITY_MODE;
        }
        if (!odrv2_user_data.available && odrv2_user_data.received_heartbeat)
        {
            odrv2_user_data.available = true;
            odrv2.clearErrors();
            delay(5);
            odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv2.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
            control_mode2 = VELOCITY_MODE;
        }
        if (!odrv3_user_data.available && odrv3_user_data.received_heartbeat)
        {
            odrv3_user_data.available = true;
            odrv3.clearErrors();
            delay(5);
            odrv3.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv3.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
            control_mode3 = VELOCITY_MODE;
        }
    }
    String s = "";
    while (Serial.available())
    {
        s = Serial.readStringUntil('\n');
        if (s.startsWith("[") && s.endsWith("]"))
        {
            processdata(s);
            
        }
        else if(s.startsWith("#") && s.endsWith("#"))
        {
            digitalWrite(LED_BUILTIN, HIGH);
            process_led_data(s);
        }

    }
}
/*
[1000,2000]
[2000,1000]
[1600,1600]


*/