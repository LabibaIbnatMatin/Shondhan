#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <Arduino.h>
#include "ODriveCAN.h"
#include <FastLED.h>
#include <SabertoothSimplified.h>
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

long b = 0;
long base_angle = 0;
long base_step;
long thrs_base_f = 100000;
long thrs_base_b = -100000;
const int t = 100;
long m = 0;

#define NUM_LEDS 60
#define DATA_PIN 33

CRGB leds[NUM_LEDS];

AccelStepper stepperB(1, 10, 11);

AccelStepper stepperG(AccelStepper::DRIVER, 2, 3);

SabertoothSimplified ST(Serial5);

#define CAN_BAUDRATE 1000000
#define ODRV0_NODE_ID0 0
#define ODRV0_NODE_ID1 1
#define ODRV0_NODE_ID2 2
#define ODRV0_NODE_ID3 3
#define ODRV0_NODE_ID4 4
#define ODRV0_NODE_ID5 5
#define ODRV0_NODE_ID6 6
#define ODRV0_NODE_ID7 7

const int ledPin = LED_BUILTIN;
byte mac[] = {0x04, 0xE9, 0xE5, 0x13, 0xA9, 0x8B};
IPAddress ip(192, 168, 1, 177);
IPAddress multicastGroup(239, 0, 0, 18);
unsigned int localPort = 5010;

char packetBuffer[1024];
EthernetUDP Udp;

unsigned long last_joystick_time = 0;
int mode = 1;
bool wifi = false, failsafe, failsafe_bck;
int panpos1 = 45, tiltpos1 = 75, panpos = 0, tiltpos = 0;
const int relayPin = 6;
int chVal[30] = {};
long positions[2] = {0, 0};
int limit_pin = 0;
bool red_once_onJoystick = false, greenOnce_joystick = false;

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_intf;

ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID0);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV0_NODE_ID1);
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV0_NODE_ID2);
ODriveCAN odrv3(wrap_can_intf(can_intf), ODRV0_NODE_ID3);
ODriveCAN odrv4(wrap_can_intf(can_intf), ODRV0_NODE_ID4);
ODriveCAN odrv5(wrap_can_intf(can_intf), ODRV0_NODE_ID5);

ODriveCAN *odrives[] = {&odrv0, &odrv1, &odrv2, &odrv3, &odrv4, &odrv5};

const uint8_t NUM_PWM_CHANNELS = 6;
const uint8_t pwmChanPin[NUM_PWM_CHANNELS] = {4, 5, 9, 12, 15, 18};
volatile uint32_t pwmRisingTime[NUM_PWM_CHANNELS];
volatile uint16_t pwmPulseWidth[NUM_PWM_CHANNELS];
bool pwm_active = false;
bool blue_once_pwm = false;

// PWM fluctuation tracking variables
unsigned long last_pwm_check_time = 0;
const unsigned long PWM_CHECK_INTERVAL = 2500; // 3 seconds
uint16_t pwm_baseline[NUM_PWM_CHANNELS];
bool pwm_fluctuation_detected[NUM_PWM_CHANNELS];
bool pwm_baseline_set = false;

bool wifi_available = false;
const unsigned long WIFI_TIMEOUT = 1000;

bool ethernet_hardware_available = false;
bool ethernet_link_available = false;
unsigned long last_ethernet_check = 0;
const unsigned long ETHERNET_CHECK_INTERVAL = 5000;

void fillStrip(uint32_t color);
void showRed();
void showGreen();
void showBlue();
void showYellow();
void showCyan();
void showMagenta();
void showWhite();
void turnOff();
void bangladeshFlag();
void disco();
bool setupCan();
void processJoystickData(const char *data, size_t length);
void processPositionData(const char *data, size_t length);
void processPWMInput();

void isr_pwm_ch0();
void isr_pwm_ch1();
void isr_pwm_ch2();
void isr_pwm_ch3();
void isr_pwm_ch4();
void isr_pwm_ch5();

void (*const pwm_isr_funcs[NUM_PWM_CHANNELS])() = {
    isr_pwm_ch0, isr_pwm_ch1, isr_pwm_ch2, isr_pwm_ch3, isr_pwm_ch4, isr_pwm_ch5};

void handlePwmInterrupt(uint8_t ch)
{
    uint32_t t = micros();
    bool level = digitalReadFast(pwmChanPin[ch]);
    if (level)
    {
        pwmRisingTime[ch] = t;
    }
    else
    {
        uint16_t width = uint16_t(t - pwmRisingTime[ch]);
        if (width >= 500 && width <= 2500)
        {
            pwmPulseWidth[ch] = width;
        }
    }
}

void isr_pwm_ch0() { handlePwmInterrupt(0); }
void isr_pwm_ch1() { handlePwmInterrupt(1); }
void isr_pwm_ch2() { handlePwmInterrupt(2); }
void isr_pwm_ch3() { handlePwmInterrupt(3); }
void isr_pwm_ch4() { handlePwmInterrupt(4); }
void isr_pwm_ch5() { handlePwmInterrupt(5); }

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
ODriveUserData odrv4_user_data;
ODriveUserData odrv5_user_data;

enum
{
    POSITION_MODE = 0,
    VELOCITY_MODE = 1,
} control_mode0,
    control_mode1, control_mode2, control_mode3, control_mode4, control_mode5;

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

void stepper_setup()
{
    stepperB.setMaxSpeed(900);
    stepperB.setAcceleration(200);
    stepperB.setSpeed(600);

    stepperB.setMinPulseWidth(t);

    stepperG.setAcceleration(2000);
    stepperG.setMinPulseWidth(15);
    delay(0.01);
}

void wheel(int x, int y)
{
    float x_norm = (x - 1500) / 500.0;
    float y_norm = (y - 1500) / 500.0;

    float left_speed_norm = y_norm + x_norm;
    float right_speed_norm = y_norm - x_norm;

    float left_motor = 1500 + 500 * left_speed_norm;
    float right_motor = 1500 + 500 * right_speed_norm;

    left_motor = constrain(left_motor, 1000, 2000);
    right_motor = constrain(right_motor, 1000, 2000);

    // patch
    if (left_motor > 1500)
    {
        left_motor = 1500 - abs(1500 - left_motor);
    }
    else
    {
        left_motor = 1500 + abs(1500 - left_motor);
    }

    if (right_motor > 1500)
    {
        right_motor = 1500 - abs(1500 - right_motor);
    }
    else
    {
        right_motor = 1500 + abs(1500 - right_motor);
    }

    int pwmMotor0 = map(left_motor, 1000, 2000, -53, 53);

    if (odrv1_user_data.available)
    {
        odrv1.setVelocity(pwmMotor0);
    }

    int pwmMotor1 = map(right_motor, 1000, 2000, -53, 53);
    if (odrv3_user_data.available)
    {
        odrv3.setVelocity(pwmMotor0);
    }

    if (odrv2_user_data.available)
    {
        odrv2.setVelocity(-1 * pwmMotor1);
    }

    if (odrv0_user_data.available)
    {
        odrv0.setVelocity(-1 * pwmMotor1);
    }
}

void armWheel(int x, int y)
{
    const int MAX_SPEED = 127;
    static int current_speed1 = 0;
    static int current_speed2 = 0;
    if (x > 1550)
        current_speed1 = map(x, 1500, 2000, 0, MAX_SPEED);
    else if (x < 1450)
        current_speed1 = -map(x, 1000, 1500, MAX_SPEED, 0);
    else
        current_speed1 = 0;

    if (y > 1550)
        current_speed2 = map(y, 1500, 2000, 0, MAX_SPEED);
    else if (y < 1450)
        current_speed2 = -map(y, 1000, 1500, MAX_SPEED, 0);
    else
        current_speed2 = 0;

    ST.motor(1, current_speed2);
    ST.motor(2, current_speed1);
}

void gripper(int x)
{
    int dir = 1;
    if (x < 1500)
        dir = -1;
    else
        dir = 1;
    if (abs(1500 - x) > 5)
    {
        stepperG.setMaxSpeed(16000);
        stepperG.moveTo(dir * 1000000);
    }
    else
    {
        stepperG.setMaxSpeed(0);
        stepperG.moveTo(dir * 0);
        stepperG.stop();
    }
}
double angle2 = 0;
void ghora(int x)
{
    // int vel = map(x, 1000, 2000, 13, -13);
    // if (odrv5_user_data.available)
    //     odrv5.setVelocity(vel);
    if (x > 1500)
    {
        if (angle2 < 65)
        {
            angle2 += 0.8;
            if (odrv5_user_data.available)
                odrv5.setPosition(angle2 * ((7.0) / 90.0));
        }
    }
    else if (x < 1500)
    {
        if (angle2 > -65)
        {
            angle2 -= 0.8;
            if (odrv5_user_data.available)
                odrv5.setPosition(angle2 * ((7.0) / 90.0));
        }
    }
    else
    {
    }
}

double angle = 0;

void HANDLE_LIFTER()
{
    if (mode == 2)
    {
        if (angle + 0.015 > 20)
            angle = 20;
        else
            angle += 0.015;
        odrv4.setPosition(angle * ((7.0) / 90.0));
    }
    else if (mode == 3)
    {
        if (angle - 0.015 < -150)
            angle = -150;
        else
            angle -= 0.015;
        odrv4.setPosition(angle * ((7.0) / 90.0));
    }

    delay(1);
}

void processJoystickData(const char *data, size_t length)
{
    wifi_available = true;
    pwm_active = false;

    if (control_mode0 == POSITION_MODE)
    {
        odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        control_mode0 = VELOCITY_MODE;
    }
    if (control_mode1 == POSITION_MODE)
    {
        odrv1.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        control_mode1 = VELOCITY_MODE;
    }
    if (control_mode2 == POSITION_MODE)
    {
        odrv2.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        control_mode2 = VELOCITY_MODE;
    }
    if (control_mode3 == POSITION_MODE)
    {
        odrv3.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        control_mode3 = VELOCITY_MODE;
    }

    pumpEvents(can_intf);
    last_joystick_time = millis();

    std::vector<int> arr(20, 1500);
    Serial.println(data);
    if (data[0] != '[')
    {
        if (!red_once_onJoystick)
        {
            odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
            odrv1.setState(ODriveAxisState::AXIS_STATE_IDLE);
            odrv2.setState(ODriveAxisState::AXIS_STATE_IDLE);
            odrv3.setState(ODriveAxisState::AXIS_STATE_IDLE);
            // showRed();
            red_once_onJoystick = true;
            greenOnce_joystick = false;
        }
        return;
    }

    std::string input(data, length);
    std::istringstream iss(input.substr(1, input.length() - 2));
    std::string token;
    while (std::getline(iss, token, ','))
    {
        char index_char = token[0];
        int index = isdigit(index_char) ? (index_char - '0') : (10 + (index_char - 'A'));
        int value = std::stoi(token.substr(1));
        if (index >= 0 && index < arr.size())
        {
            arr[index] = value;
            Serial.print(arr[index]);
            Serial.print(' ');
        }
    }
    Serial.println();

    if (arr[0] >= 1700)
    {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
        digitalWrite(LED_BUILTIN, LOW);
    }

    wheel(arr[0], arr[1]);
    armWheel(arr[2], arr[3]);

    limit_pin = digitalRead(14);

    if (arr.size() >= 8)
    {
        chVal[9] = arr[7];
    }

    ghora(arr[4]);

    if (arr[5] == 2000)
    {
        mode = 2;
    }
    else if (arr[5] == 1000)
    {
        mode = 3;
    }
    else if (arr[5] == 1500)
    {
        mode = 1;
    }

    gripper(arr[6]);

    if (!greenOnce_joystick)
    {
        greenOnce_joystick = true;
        // showGreen();
        red_once_onJoystick = false;
        odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrv3.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    }
}

void base_arm()
{
    if ((chVal[9] < 1450 || chVal[9] > 1550))
    {
        int diff = abs(chVal[9] - 1500);
        int mp = 400;

        if (chVal[9] > 1550 && stepperB.currentPosition() <= thrs_base_f && chVal[9] <= 2000)
        {
            b -= mp;
        }
        else if (chVal[9] < 1450 && stepperB.currentPosition() >= thrs_base_b && chVal[9] >= 1000)
            b += mp;

        stepperB.moveTo(b);
        stepperB.setSpeed(900);

        stepperB.runSpeedToPosition();
    }

    else
    {
        b = 0;
    }
    if (stepperB.currentPosition() >= thrs_base_f)
        positions[0] = thrs_base_f;
    if (stepperB.currentPosition() <= thrs_base_b)
        positions[0] = thrs_base_b;

    delay(0.01);
}

bool disarm = false;

void processPositionData(const char *data, size_t length)
{
    pumpEvents(can_intf);
    last_joystick_time = millis();

    std::string input(data, length);
    std::istringstream iss(input);
    std::string id, position;
    iss >> id >> position;

    if (id == "0")
    {
        if (control_mode0 == VELOCITY_MODE)
        {
            odrv0.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ);
            control_mode0 = POSITION_MODE;
        }
        digitalWrite(LED_BUILTIN, LOW);
        odrv0.setPosition(std::stof(position));
        delay(100);
    }
    else if (id == "1")
    {
        if (control_mode1 == VELOCITY_MODE)
        {
            odrv1.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ);
            control_mode1 = POSITION_MODE;
        }
        digitalWrite(LED_BUILTIN, LOW);
        odrv1.setPosition(std::stof(position));
        delay(100);
    }
    else if (id == "2")
    {
        if (control_mode2 == VELOCITY_MODE)
        {
            odrv2.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ);
            control_mode2 = POSITION_MODE;
        }
        digitalWrite(LED_BUILTIN, LOW);
        odrv2.setPosition(std::stof(position));
        delay(100);
    }
    else if (id == "3")
    {
        if (control_mode3 == VELOCITY_MODE)
        {
            odrv3.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ);
            control_mode3 = POSITION_MODE;
        }
        digitalWrite(LED_BUILTIN, LOW);
        odrv3.setPosition(std::stof(position));
        delay(100);
    }
    else
    {
    }
}

void posControl4(int relativeAngle, int angle)
{
    if (angle > 720)
    {
        return;
    }
}

void testing()
{
    // odrv5.setPosition(0);
    // delay(100);
    // odrv5.setPosition(-10 * (7.0 / 90.0));
    // delay(100);
    // odrv5.setPosition(-20 * (7.0 / 90.0));
    // delay(100);
    // odrv5.setPosition(-30 * (7.0 / 90.0));
    // delay(100);
    // odrv5.setPosition(-40 * (7.0 / 90.0));
    // delay(100);
    // odrv5.setPosition(-65 * (7.0 / 90.0));
    // delay(100);
}

void

setup()
{
    Serial.begin(115200);
    Serial5.begin(9600);

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);

    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    FastLED.show();

    // showRed();

    for (uint8_t ch = 0; ch < NUM_PWM_CHANNELS; ch++)
    {
        pinMode(pwmChanPin[ch], INPUT_PULLUP);
        pwmPulseWidth[ch] = 1500;
        attachInterrupt(digitalPinToInterrupt(pwmChanPin[ch]), pwm_isr_funcs[ch], CHANGE);
    }

    Ethernet.begin(mac, ip);

    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
        ethernet_hardware_available = false;
    }
    else
    {
        ethernet_hardware_available = true;
        if (Ethernet.linkStatus() == LinkOFF)
        {
            ethernet_link_available = false;
        }
        else
        {
            ethernet_link_available = true;
            Udp.begin(localPort);
        }
    }

    odrv0.onFeedback(onFeedback, &odrv0_user_data);
    odrv0.onStatus(onHeartbeat, &odrv0_user_data);
    odrv1.onFeedback(onFeedback, &odrv1_user_data);
    odrv1.onStatus(onHeartbeat, &odrv1_user_data);
    odrv2.onFeedback(onFeedback, &odrv2_user_data);
    odrv2.onStatus(onHeartbeat, &odrv2_user_data);
    odrv3.onFeedback(onFeedback, &odrv3_user_data);
    odrv3.onStatus(onHeartbeat, &odrv3_user_data);
    odrv4.onFeedback(onFeedback, &odrv4_user_data);
    odrv4.onStatus(onHeartbeat, &odrv4_user_data);
    odrv5.onFeedback(onFeedback, &odrv5_user_data);
    odrv5.onStatus(onHeartbeat, &odrv5_user_data);

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
            odrv2_user_data.received_heartbeat || odrv3_user_data.received_heartbeat ||
            odrv4_user_data.received_heartbeat || odrv5_user_data.received_heartbeat)
        {
            break;
        }
    }

    odrv0_user_data.available = odrv0_user_data.received_heartbeat;
    odrv1_user_data.available = odrv1_user_data.received_heartbeat;
    odrv2_user_data.available = odrv2_user_data.received_heartbeat;
    odrv3_user_data.available = odrv3_user_data.received_heartbeat;
    odrv4_user_data.available = odrv4_user_data.received_heartbeat;
    odrv5_user_data.available = odrv5_user_data.received_heartbeat;

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

        if (odrv4_user_data.available && odrv4_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
        {
            odrv4.clearErrors();
            delay(1);
            odrv4.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        }

        if (odrv5_user_data.available && odrv5_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
        {
            odrv5.clearErrors();
            delay(1);
            odrv5.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
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
        if (odrv4_user_data.available && odrv4_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
            all_ready = false;
        if (odrv5_user_data.available && odrv5_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
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
    if (odrv4_user_data.available)
    {
        odrv4.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ);

        odrv4.setPosition(0.0 * (7.0 / 90.0));
        odrv4.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
        control_mode4 = POSITION_MODE;
    }
    if (odrv5_user_data.available)
    {
        odrv5.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ);

        odrv5.setPosition(0.0 * (7.0 / 90.0));
        odrv5.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
        control_mode4 = POSITION_MODE;
    }
    // showGreen();
    stepper_setup();

    digitalWrite(ledPin, LOW);
}
void emergencyStop();

void processPWMInput()
{
    return;
    last_joystick_time = millis();
    pwm_active = true;

    if (!blue_once_pwm)
    {
        // showBlue();
        blue_once_pwm = true;
        red_once_onJoystick = false;
        greenOnce_joystick = false;
        if (odrv0_user_data.available && control_mode0 != VELOCITY_MODE)
        {
            odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
            control_mode0 = VELOCITY_MODE;
        }
        if (odrv1_user_data.available && control_mode1 != VELOCITY_MODE)
        {
            odrv1.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
            control_mode1 = VELOCITY_MODE;
        }
        if (odrv2_user_data.available && control_mode2 != VELOCITY_MODE)
        {
            odrv2.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
            control_mode2 = VELOCITY_MODE;
        }
        if (odrv3_user_data.available && control_mode3 != VELOCITY_MODE)
        {
            odrv3.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
            control_mode3 = VELOCITY_MODE;
        }
        if (odrv0_user_data.available && odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
            odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        if (odrv1_user_data.available && odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
            odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        if (odrv2_user_data.available && odrv2_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
            odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        if (odrv3_user_data.available && odrv3_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
            odrv3.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    }

    // New simplified PWM failsafe logic
    static bool failsafe_pwm = false;
    static unsigned long last_change_check = 0;
    static unsigned long last_recovery_check = 0;
    static uint16_t previous_values[NUM_PWM_CHANNELS];
    static bool first_run = true;

    unsigned long current_time = millis();

    // Initialize on first run
    if (first_run)
    {
        for (int i = 0; i < NUM_PWM_CHANNELS; i++)
        {
            previous_values[i] = pwmPulseWidth[i];
        }
        last_change_check = current_time;
        last_recovery_check = current_time;
        first_run = false;
    }

    // Check for value changes every 1.5 seconds
    if (current_time - last_change_check >= 1500)
    {
        bool any_change = false;
        bool all_centered = true;

        // Check if values changed and if all are centered
        for (int i = 0; i < 4; i++) // Check first 4 channels
        {
            if (abs((int)pwmPulseWidth[i] - (int)previous_values[i]) >= 2)
            {
                any_change = true;
            }
            if (abs((int)pwmPulseWidth[i] - 1500) > 10)
            {
                all_centered = false;
            }
            previous_values[i] = pwmPulseWidth[i];
        }

        // Special case: if all values are 1500, don't consider it a failure
        if (all_centered)
        {
            any_change = true;
        }

        // If no change detected, enter failsafe
        if (!any_change && !failsafe_pwm)
        {
            failsafe_pwm = true;
            Serial.println("PWM Failsafe: No value changes detected");
            emergencyStop();
        }

        last_change_check = current_time;
    }

    // Recovery check every 2 seconds
    if (failsafe_pwm && current_time - last_recovery_check >= 2000)
    {
        bool all_centered = true;

        // Check if all critical channels are centered
        for (int i = 0; i < 4; i++)
        {
            if (abs((int)pwmPulseWidth[i] - 1500) > 10)
            {
                all_centered = false;
                break;
            }
        }

        if (all_centered)
        {
            failsafe_pwm = false;
            Serial.println("PWM Recovery: Values centered, resuming operation");
        }

        last_recovery_check = current_time;
    }

    // Process PWM values (use neutral values during failsafe)
    int pwm_ch_val[NUM_PWM_CHANNELS];
    if (failsafe_pwm)
    {
        // Use neutral values during failsafe
        for (int i = 0; i < NUM_PWM_CHANNELS; i++)
        {
            pwm_ch_val[i] = 1500;
        }
    }
    else
    {
        // Use actual PWM values when not in failsafe
        for (int i = 0; i < NUM_PWM_CHANNELS; i++)
        {
            pwm_ch_val[i] = constrain(map(pwmPulseWidth[i], 1000, 2000, 1000, 2000), 1000, 2000);
        }
    }

    // Apply motor commands
    wheel(pwm_ch_val[0], pwm_ch_val[1]);
    armWheel(pwm_ch_val[2], pwm_ch_val[3]);

    // Handle mode and other controls
    if (pwm_ch_val[5] > 1700)
    {
        mode = 2;
    }
    else if (pwm_ch_val[5] < 1300)
    {
        mode = 3;
    }
    else
    {
        mode = 1;
    }

    // Visual feedback for failsafe state
    if (failsafe_pwm)
    {
        if (!red_once_onJoystick)
        {
            // showRed();
            red_once_onJoystick = true;
            blue_once_pwm = false;
        }
    }
    else
    {
        if (!blue_once_pwm && pwm_active)
        {
            // showBlue();
            blue_once_pwm = true;
            red_once_onJoystick = false;
        }
    }

    digitalWrite(LED_BUILTIN, LOW);
}

bool security_triggered = false;
unsigned long last_control_signal_time = 0;
const unsigned long CONTROL_TIMEOUT = 1000; // 1 second timeout for both control systems
void checkSecurity()
{
    // Check if both WiFi and PWM controls are failing
    bool wifi_failed = !wifi_available;

    // New PWM failsafe logic - check for fluctuation over 3 seconds
    bool pwm_failed = false;
    unsigned long current_time = millis();

    if (!pwm_baseline_set)
    {
        for (int i = 0; i < NUM_PWM_CHANNELS; ++i)
        {
            pwm_baseline[i] = pwmPulseWidth[i];
            pwm_fluctuation_detected[i] = false;
        }
        last_pwm_check_time = current_time;
        pwm_baseline_set = true;
        return;
    }

    for (int i = 0; i < NUM_PWM_CHANNELS; ++i)
    {
        if (abs((int)pwmPulseWidth[i] - (int)pwm_baseline[i]) >= 2)
        {
            pwm_fluctuation_detected[i] = true;
        }
    }

    if (current_time - last_pwm_check_time >= PWM_CHECK_INTERVAL)
    {

        bool flag = false;

        for (int i = 0; i < NUM_PWM_CHANNELS; ++i)
        {
            // if (pwm_fluctuation_detected[i])
            // {
            //     flag = true;
            // }
            pwm_baseline[i] = pwmPulseWidth[i];
            pwm_fluctuation_detected[i] = false;
        }
        // if (!flag)
        // {
        //     emergencyStop();
        // }
        last_pwm_check_time = current_time;
    }

    if (!wifi_failed || !pwm_failed)
    {
        last_control_signal_time = millis();
        if (security_triggered)
        {
            security_triggered = false;
            // showYellow();
            delay(300);
            if (wifi_available)
            {
                // showGreen();
            }
            else if (pwm_active)
            {
                // showBlue();
            }
        }
    }

    // Check if we've exceeded the timeout period with no valid controls
    if (millis() - last_control_signal_time > CONTROL_TIMEOUT)
    {
        if (!security_triggered)
        {
            security_triggered = true;
            emergencyStop();
            // Visual indication - alternating red/white pattern
            for (int i = 0; i < 3; i++)
            {
                // showRed();
                // delay(150);
                // showWhite();
                // delay(150);
            }
            // showRed();
        }
    }
}

bool colorGreen = false;

void processColorData(const char *data, size_t length)
{
    Serial.print("Color Data: ");
    Serial.println(data);

    wifi_available = true;
    pwm_active = false;
    if (data[0] == '0')
    {
        Serial.println("Red Color Command Received");
        showRed();
        colorGreen = false;
    }
    else if (data[0] == '1')
    {
        showGreen();
        colorGreen = false;
    }
    else if (data[0] == '2')
    {
        // showGreen();
        colorGreen = true;
    }
}

void emergencyStop()
{
    // Stop all motors
    wheel(1500, 1500);
    armWheel(1500, 1500);
    ghora(1500);
    gripper(1500);

    // // Disable ODrives
    // if (odrv0_user_data.available)
    //     odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
    // if (odrv1_user_data.available)
    //     odrv1.setState(ODriveAxisState::AXIS_STATE_IDLE);
    // if (odrv2_user_data.available)
    //     odrv2.setState(ODriveAxisState::AXIS_STATE_IDLE);
    // if (odrv3_user_data.available)
    //     odrv3.setState(ODriveAxisState::AXIS_STATE_IDLE);
    // if (odrv4_user_data.available)
    //     odrv4.setState(ODriveAxisState::AXIS_STATE_IDLE);
    // if (odrv5_user_data.available)
    //     odrv5.setState(ODriveAxisState::AXIS_STATE_IDLE);

    // Stop steppers
    stepperB.stop();
    stepperG.stop();
}

void loop()
{
        //disco();
        // showRed();
        // delay(100);
        // showBlue();
        // delay(100);
        // showCyan();
        // delay(100);
        // FastLED.clear();
        // FastLED.show();
        // delay(100);

    if (colorGreen)
    {
        showBlue();
        delay(100);
        FastLED.clear();
        FastLED.show();
        delay(100);
    }

    // Check security status first

    // Only proceed with normal operation if security system isn't triggered
    if (!security_triggered)
    {
        stepperG.run();
        pumpEvents(can_intf);
        base_arm();
        HANDLE_LIFTER();
    }

    unsigned long current_time = millis();
    if (current_time - last_ethernet_check > ETHERNET_CHECK_INTERVAL)
    {
        last_ethernet_check = current_time;

        if (!ethernet_hardware_available)
        {
            if (Ethernet.hardwareStatus() != EthernetNoHardware)
            {
                ethernet_hardware_available = true;
            }
        }

        if (ethernet_hardware_available)
        {
            if (Ethernet.linkStatus() == LinkON && !ethernet_link_available)
            {
                ethernet_link_available = true;
                Udp.begin(localPort);
            }
            else if (Ethernet.linkStatus() == LinkOFF && ethernet_link_available)
            {
                ethernet_link_available = false;
            }
        }
    }

    if (ethernet_link_available)
    {
        int packetSize = Udp.parsePacket();
        if (packetSize)
        {
            memset(packetBuffer, 0, 1024);
            Udp.read(packetBuffer, 1024);
            Serial.print("Received packet: ");
            Serial.println(packetBuffer);
            if (strncmp(packetBuffer, "JOYSTICK:", 9) == 0)
            {
                processJoystickData(packetBuffer + 9, packetSize - 9);
            }
            else if (strncmp(packetBuffer, "M:", 2) == 0)
            {

                processColorData(packetBuffer + 2, packetSize - 2);
            }
        }
    }

    if (millis() - last_joystick_time > WIFI_TIMEOUT)
    {
        wifi_available = false;
        if (greenOnce_joystick || red_once_onJoystick)
        {
            greenOnce_joystick = false;
            red_once_onJoystick = false;
        }
    }

    if (!wifi_available)
    {
        processPWMInput();
    }
    else
    {
        if (pwm_active)
        {
            pwm_active = false;
            blue_once_pwm = false;
            wheel(1500, 1500);
            armWheel(1500, 1500);
            ghora(1500);
            gripper(1500);
            mode = 1;
            chVal[9] = 1500;
        }
    }

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
        if (!odrv4_user_data.available && odrv4_user_data.received_heartbeat)
        {
            odrv4_user_data.available = true;
            odrv4.clearErrors();
            delay(5);
            odrv4.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv4.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ);
            odrv4.setPosition(0.0 * (7.0 / 90.0));
            odrv4.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);

            control_mode3 = POSITION_MODE;
        }
        if (!odrv5_user_data.available && odrv5_user_data.received_heartbeat)
        {
            odrv5_user_data.available = true;
            odrv5.clearErrors();
            delay(5);
            odrv5.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);

            odrv5.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
            odrv5.setPosition(0.0 * (7.0 / 90.0));
            control_mode5 = POSITION_MODE;
        }
    }
}

void fillStrip(uint32_t color)
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = color;
    }
    FastLED.show();
}

void disco();

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

void showYellow()
{
    fillStrip(CRGB::Yellow);
}

void showCyan()
{
    fillStrip(CRGB::Cyan);
}

void showMagenta()
{
    fillStrip(CRGB::Magenta);
}

void showWhite()
{
    fillStrip(CRGB::White);
}

void turnOff()
{
    FastLED.clear();
    FastLED.show();
}

void bangladeshFlag()
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CRGB::DarkGreen;
    }

    int circleCenter = NUM_LEDS / 3;
    int circleRadius = NUM_LEDS / 6;

    for (int i = circleCenter - circleRadius; i <= circleCenter + circleRadius; i++)
    {
        if (i >= 0 && i < NUM_LEDS)
        {
            leds[i] = CRGB::Red;
        }
    }
    FastLED.show();
}

void disco()
{
    static uint16_t pos = 0;

    fadeToBlackBy(leds, NUM_LEDS, 51);

    uint16_t index = pos % NUM_LEDS;

    uint8_t colorIndex = pos & 0xFF;
    uint8_t r, g, b_val;
    if (colorIndex < 85)
    {
        r = colorIndex * 3;
        g = 255 - colorIndex * 3;
        b_val = 0;
    }
    else if (colorIndex < 170)
    {
        colorIndex -= 85;
        r = 255 - colorIndex * 3;
        g = 0;
        b_val = colorIndex * 3;
    }
    else
    {
        colorIndex -= 170;
        r = 0;
        g = colorIndex * 3;
        b_val = 255 - colorIndex * 3;
    }

    leds[index] = CRGB(r, g, b_val);

    FastLED.show();
    pos++;
}