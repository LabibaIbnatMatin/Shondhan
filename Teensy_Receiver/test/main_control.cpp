
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <QNEthernet.h>
#include <cctype>
#include <cstdlib>
#include <vector>
#include <Wire.h>
#include <AS5600.h>
#include <teensystep4.h>
#undef CAN_ERROR_BUS_OFF
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"
#include <FastLED.h>
// #include "Servo.h"

// #define SERVO_PIN 14
// #define SERVO_PIN2 38

// Servo servo1;
// Servo servo2;

#define NUM_LEDS 60
#define DATA_PIN 15

CRGB leds[NUM_LEDS];

using namespace TS4;

using namespace qindesign::network;

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_intf;

constexpr uint8_t MOTOR_ID = 8;

constexpr float MIN_ANGLE_DEG = -70.0f;
constexpr float MAX_ANGLE_DEG = 70.0f;

constexpr float INPUT_MIN = 1000.0f;
constexpr float INPUT_MAX = 2000.0f;
constexpr float SPEED_MIN_DPS = -5.0f;
constexpr float SPEED_MAX_DPS = 5.0f;
bool isGreen = false;

constexpr size_t UDP_BUFFER_SIZE = 1024;
constexpr size_t MAX_ARM_VALUES = 20;
uint32_t lastWheelTime = 0;
uint32_t lastArmTime = 0;
bool wheelTimeoutTriggered = false;
bool armTimeoutTriggered = false;
IPAddress ip(192, 168, 2, 155);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 2, 1);
constexpr uint16_t LOCAL_PORT = 5010;

EthernetUDP Udp;
char packetBuffer[UDP_BUFFER_SIZE];

std::vector<double> armValues;

// ===== COMPATIBILITY FOR OLD processArm =====
double targetBaseAngle = 0.0;
double currentMotor8Angle = 0.0;
double motorScaleFactor = -1.0;
Stepper stepperB(22, 23);
Stepper stepperG(20, 21);
constexpr int32_t STEPPER_G_SPEED = 2180;
double odrv4PositionOffset = 0.0;
double lastOdrv4PositionCommand = -9999.99;
// ============================================

#define ODRV0_NODE_ID0 0
#define ODRV0_NODE_ID1 1
#define ODRV0_NODE_ID2 2
#define ODRV0_NODE_ID3 3
#define ODRV0_NODE_ID4 6
#define ODRV0_NODE_ID5 5
#define ODRV0_NODE_ID7 7

ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID0);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV0_NODE_ID1);
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV0_NODE_ID2);
ODriveCAN odrv3(wrap_can_intf(can_intf), ODRV0_NODE_ID3);
ODriveCAN odrv4(wrap_can_intf(can_intf), ODRV0_NODE_ID4);
ODriveCAN odrv5(wrap_can_intf(can_intf), ODRV0_NODE_ID5);
ODriveCAN odrv7(wrap_can_intf(can_intf), ODRV0_NODE_ID7);
ODriveCAN *odrives[] = {&odrv0, &odrv1, &odrv2, &odrv3, &odrv4, &odrv5, &odrv7};
struct EncoderPositionReply
{
    int id;
    double temp, iq, speed, angle;
    void print()
    {
        Serial.print(id);
        Serial.print(" ");
        Serial.print(temp);
        Serial.print(" ");
        Serial.print(iq);
        Serial.print(" ");
        Serial.print(speed);
        Serial.print(" ");
        Serial.print(angle);
        Serial.print(" ");
        Serial.println();
    }
};
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
ODriveUserData odrv7_user_data;

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

float requested_velocity_dps = 0.0f;
float last_commanded_velocity_dps = 0.0f;
bool have_commanded_velocity = false;
bool motor_is_homing = true;
float current_angle_deg = 0.0f;
int16_t current_speed_dps = 0;
int8_t current_temp_c = 0;

unsigned long last_status_request_ms = 0;

// --------------------------------------------------
// helpers
// --------------------------------------------------
static inline int16_t le16_to_i16(uint8_t lo, uint8_t hi)
{
    return (int16_t)((uint16_t)hi << 8 | lo);
}

void sendFrame(const CAN_message_t &msg)
{
    can_intf.write(msg);
}

void setVelocity(float speed_dps);
void applyMotorVelocity();

// --------------------------------------------------
// Velocity control command
// Command: 0xA2
//
// speed unit:
// 0.01 dps
//
// Example:
// 100 = 1 dps
// 1500 = 15 dps
// --------------------------------------------------
void setVelocity(float speed_dps)
{
    CAN_message_t msg;

    msg.id = 0x140 + MOTOR_ID;
    msg.flags.extended = 0;
    msg.flags.remote = 0;
    msg.len = 8;

    int32_t speed_raw = (int32_t)(speed_dps * 100.0f);

    msg.buf[0] = 0xA2; // velocity mode
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;

    msg.buf[4] = speed_raw & 0xFF;
    msg.buf[5] = (speed_raw >> 8) & 0xFF;
    msg.buf[6] = (speed_raw >> 16) & 0xFF;
    msg.buf[7] = (speed_raw >> 24) & 0xFF;

    can_intf.write(msg);
}

void setAcceleration(uint8_t motor_id, uint32_t accel)
{
    CAN_message_t msg;

    msg.id = 0x140 + motor_id;
    msg.flags.extended = 0;
    msg.flags.remote = 0;
    msg.len = 8;

    msg.buf[0] = 0x43;
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;

    msg.buf[4] = accel & 0xFF;
    msg.buf[5] = (accel >> 8) & 0xFF;
    msg.buf[6] = (accel >> 16) & 0xFF;
    msg.buf[7] = (accel >> 24) & 0xFF;

    can_intf.write(msg);
}

void sendPositionCommand(uint8_t motor_id, float angle_deg, uint16_t speed_dps)

{
    CAN_message_t msg;
    msg.id = 0x140 + motor_id;
    msg.flags.extended = 0;
    msg.flags.remote = 0;
    msg.len = 8;

    int32_t angle_lsb = (int32_t)(angle_deg * 100.0f);
    uint16_t speed_lsb = speed_dps;

    msg.buf[0] = 0xA4;
    msg.buf[1] = 0x00;
    msg.buf[2] = speed_lsb & 0xFF;
    msg.buf[3] = (speed_lsb >> 8) & 0xFF;
    msg.buf[4] = angle_lsb & 0xFF;
    msg.buf[5] = (angle_lsb >> 8) & 0xFF;
    msg.buf[6] = (angle_lsb >> 16) & 0xFF;
    msg.buf[7] = (angle_lsb >> 24) & 0xFF;
    can_intf.write(msg);
}

void stopMotor()
{
    setVelocity(0);
}

float mapControlInputToVelocity(double input)
{
    if (input < INPUT_MIN)
        input = INPUT_MIN;
    if (input > INPUT_MAX)
        input = INPUT_MAX;

    const double ratio = (input - 1000.0) / (2000.0 - 1000.0);
    return (float)(-10.0 + ratio * (10.0 - (-10.0)));
}

float applyLimitGuard(float speed_dps)
{
    if (current_angle_deg >= MAX_ANGLE_DEG && speed_dps < 0.0f)
    {
        return 0.0f;
    }

    if (current_angle_deg <= MIN_ANGLE_DEG && speed_dps > 0.0f)
    {
        return 0.0f;
    }

    return speed_dps;
}

void applyMotorVelocity()
{
    if (motor_is_homing)
    {
        if (abs(requested_velocity_dps) < 0.1f)
            return; // Still homing, no stick input
        motor_is_homing = false;
    }
    // const float limited_velocity_dps = applyLimitGuard(requested_velocity_dps);
    const float limited_velocity_dps = requested_velocity_dps;
    // if (have_commanded_velocity && limited_velocity_dps == last_commanded_velocity_dps)
    // {
    //     return;
    // }
    setVelocity(limited_velocity_dps);
    last_commanded_velocity_dps = limited_velocity_dps;
    have_commanded_velocity = true;
}

void wheel(int x, int y)
{
    int pwmMotor0 = map(x, 1000, 2000, -95, 95);
    if (odrv2_user_data.available)
    {
        odrv2.setVelocity(pwmMotor0);
    }

    int pwmMotor1 = map(y, 1000, 2000, -95, 95);
    if (odrv3_user_data.available)
    {
        odrv3.setVelocity(pwmMotor0); // kept as original script
    }

    if (odrv1_user_data.available)
    {
        odrv1.setVelocity(-1 * pwmMotor1);
    }
    if (odrv0_user_data.available)
    {
        odrv0.setVelocity(-1 * pwmMotor1);
    }
}

bool handleWheelPacket(const String &packet)
{
    String trimmed = packet;
    trimmed.trim();

    if (!trimmed.startsWith("[") || !trimmed.endsWith("]") || trimmed.startsWith("A:["))
    {
        return false;
    }

    String payload = trimmed.substring(1, trimmed.length() - 1);
    std::vector<int> newValues;
    unsigned int startIndex = 0;

    while (startIndex < payload.length())
    {
        int commaIndex = payload.indexOf(',', startIndex);
        String token;

        if (commaIndex == -1)
        {
            token = payload.substring(startIndex);
            startIndex = payload.length();
        }
        else
        {
            token = payload.substring(startIndex, commaIndex);
            startIndex = commaIndex + 1;
        }

        token.trim();
        if (token.length() > 0)
        {
            newValues.push_back(token.toInt());
        }
    }

    if (newValues.size() >= 2)
    {
        lastWheelTime = millis();
        wheelTimeoutTriggered = false;
        wheel(newValues[0], newValues[1]);
        return true;
    }

    return false;
}

void controlStepperG(double angle_deg)
{
    static int lastDirection = 0;
    int direction = 0;

    if (angle_deg < 5.0)
    {
        direction = -1;
    }
    else if (angle_deg > 60.0)
    {
        direction = 1;
    }

    if (direction == lastDirection)
    {
        return;
    }

    if (direction == 0)
    {
        if (stepperG.isMoving)
        {
            stepperG.stopAsync();
        }
    }
    else
    {
        stepperG.rotateAsync(STEPPER_G_SPEED * direction);
    }

    lastDirection = direction;
}

void setOdrv4Position(double position)
{
    const double commandPosition = position + odrv4PositionOffset;
    if (lastOdrv4PositionCommand != commandPosition)
    {
        odrv4.setPosition(commandPosition * (7.0 / 90.0));
        lastOdrv4PositionCommand = commandPosition;
    }
}

void updateMotor(double raw_value)
{
    // The current main.cpp logic does this implicitly with applyMotorVelocity and processArmValues.
    // If we want to use the OLD CAN message format as a fallback, we can send it directly.
    float target_angle = raw_value * motorScaleFactor;
    // ... we don't fully implement the old sendPositionCommand for Motor 8 because it conflicts with main.cpp's new velocity command.
}

bool parseArmPacket(const String &packet, std::vector<double> &values)

{
    String trimmed = packet;
    trimmed.trim();

    if (!trimmed.startsWith("A:[") || !trimmed.endsWith("]"))
    {
        return false;
    }

    values.clear();
    String payload = trimmed.substring(3, trimmed.length() - 1);
    unsigned int startIndex = 0;

    while (startIndex < payload.length())
    {
        if (values.size() >= MAX_ARM_VALUES)
        {
            return false;
        }

        const int commaIndex = payload.indexOf(',', startIndex);
        String token = (commaIndex == -1)
                           ? payload.substring(startIndex)
                           : payload.substring(startIndex, commaIndex);
        token.trim();

        if (token.length() == 0)
        {
            return false;
        }

        char *endPtr = nullptr;
        const double value = std::strtod(token.c_str(), &endPtr);
        while (endPtr != nullptr && *endPtr != '\0' && std::isspace((unsigned char)*endPtr))
        {
            ++endPtr;
        }

        if (endPtr == token.c_str() || *endPtr != '\0')
        {
            return false;
        }

        values.push_back(value);

        if (commaIndex == -1)
        {
            break;
        }

        startIndex = commaIndex + 1;
    }

    return !values.empty();
}
double wristAngle = 0;
double wristAngle2 = 0;
bool servoStat = false;
static int32_t lastRelayHighTime = -1;
void processArmValues(const std::vector<double> &values)
{

    if (values.size() >= 1)
    {
        if (abs(values[0] - 1500) < 100)
        {
            if (stepperB.isMoving)
                stepperB.emergencyStop();
        }
        else if (values[0] > 1500)
        {
            // 1150 vs 500
            // stepperB.rotateAsync(500);
            if (values[0] <= 1700)
            {
                stepperB.rotateAsync(300);
            }
            else if (values[0] <= 1900)
            {
                stepperB.rotateAsync(500);
            }
            else
            {
                stepperB.rotateAsync(1150);
            }
        }
        else if (values[0] < 1500)
        {
            if (values[0] >= 1300)
            {
                stepperB.rotateAsync(-300);
            }
            else if (values[0] >= 1100)
            {
                stepperB.rotateAsync(-500);
            }
            else
            {
                stepperB.rotateAsync(-1150);
            }
        }
    }

    if (values.size() >= 2)
    {
        // patch
        double diff = abs(1500 - values[1]);
        int dps = 1500;
        if (values[1] > 1500)
            dps = 1500 - diff;
        else if (values[1] < 1500)
        {
            dps = 1500 + diff;
        }

        requested_velocity_dps = mapControlInputToVelocity(dps);
        applyMotorVelocity();
    }

    if (values.size() >= 3)
    {
        // map 1000 to 2000 to 10 to -10

        int pwmValue = (int)values[2];
        int mapped = map(pwmValue, 1000, 2000, -20, 20);
        odrv5.setVelocity(mapped);
    }
    if (values.size() >= 4)
    {
        // controlStepperG(values[3]);
        if (values[3] > 1500)
        {
            wristAngle += 0.5;
            if (wristAngle >= 120)
            {
                wristAngle -= 0.5;
            }
            odrv4.setPosition(wristAngle * (7.0 / 90.0));
        }
        else if (values[3] < 1500)
        {
            wristAngle -= 0.5;
            if (wristAngle <= -120)
            {
                wristAngle += 0.5;
            }
            odrv4.setPosition(wristAngle * (7.0 / 90.0));
        }
        else
        {
            odrv4.setPosition(wristAngle * (7.0 / 90.0));
        }
    }
    if (values.size() >= 5)
    {
        // controlStepperG(values[3]);
        if (values[4] > 1500)
        {

            wristAngle2 += 0.5;
            if (wristAngle2 >= 70)
            {
                wristAngle2 -= 0.5;
            }
            odrv7.setPosition(wristAngle2 * (7.0 / 90.0));
        }
        else if (values[4] < 1500)
        {
            wristAngle2 -= 0.5;
            if (wristAngle2 <= -70)
            {
                wristAngle2 += 0.5;
            }
            odrv7.setPosition(wristAngle2 * (7.0 / 90.0));
        }
        else
        {
            odrv7.setPosition(wristAngle2 * (7.0 / 90.0));
        }
    }
    if (values.size() >= 6)
    {
        if (values[5] == 1500)
        {
            if (stepperG.isMoving)
                stepperG.stopAsync();
        }
        else if (values[5] > 1500)
        {
            if (!stepperG.isMoving)
                stepperG.rotateAsync(20000);
        }
        else if (values[5] < 1500)
        {
            if (!stepperG.isMoving)
                stepperG.rotateAsync(-20000);
        }
    }
    if (values.size() >= 7)
    {
        if (values[6] == 2000)
        {
            if (lastRelayHighTime == -1)
            {
            }
            else if (millis() - lastRelayHighTime >= 80)
            {
                digitalWrite(33, LOW);
                lastRelayHighTime = -1;
            }
            if (servoStat == false)
            {
                lastRelayHighTime = millis();
                digitalWrite(33, HIGH);

                servoStat = true;
            }
        }
        else if (values[6] == 1000)
        {
            if (servoStat == true)
            {
                // Immediately release the relay when input returns to 1000
                digitalWrite(33, LOW);
                servoStat = false;
                lastRelayHighTime = -1;
            }
        }
    }
}

void handleCAN();

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
    // Green and Red are swapped electrically, so using Green shows Red
    fillStrip(CRGB::Red);
    isGreen = false;
}

void showBlue()
{
    fillStrip(CRGB::Green);
    isGreen = false;
}

void showGreen()
{
    isGreen = true;
    // Flash first before showing solid green.
    // Green and Red are swapped electrically, so using Red shows Green.
    for (int i = 0; i < 3; i++)
    {
        fillStrip(CRGB::Blue);
        uint32_t start = millis();
        while (millis() - start < 150)
        {
            handleCAN();
        }
        fillStrip(CRGB::Black);

        start = millis();
        while (millis() - start < 150)
        {
            handleCAN();
        }
    }
    fillStrip(CRGB::Blue);
}

void process_led_data(String s)
{
    if (s.length() >= 2)
    {
        if (s[1] == 'g')
        {
            showGreen();
            Serial.println("Green");
        }
        else if (s[1] == 'r')
        {
            showRed();
            Serial.println("Red");
        }
        else if (s[1] == 'b')
        {
            showBlue();
            Serial.println("Blue");
        }
    }
}

void receiveUdpPackets()
{
    int packetSize = Udp.parsePacket();
    while (packetSize > 0)
    {
        if ((size_t)packetSize >= UDP_BUFFER_SIZE)
        {
            Udp.read(packetBuffer, UDP_BUFFER_SIZE - 1);
            packetBuffer[UDP_BUFFER_SIZE - 1] = '\0';
        }
        else
        {
            const int bytesRead = Udp.read(packetBuffer, UDP_BUFFER_SIZE - 1);
            if (bytesRead > 0)
            {
                packetBuffer[bytesRead] = '\0';
                const String packet(packetBuffer);

                if (parseArmPacket(packet, armValues))
                {
                    lastArmTime = millis();
                    armTimeoutTriggered = false;
                    processArmValues(armValues);
                }
                else if (handleWheelPacket(packet))
                {
                    // Wheel packet handled
                }
                else if (packet.startsWith("#") && packet.endsWith("#"))
                {
                    process_led_data(packet);
                }
                else
                {
                    Serial.println(packet);
                }
            }
        }

        packetSize = Udp.parsePacket();
    }
}

void setupEthernet()
{
    if (!Ethernet.begin(ip, subnet, gateway))
    {
        return;
    }

    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
        return;
    }

    if (Udp.begin(LOCAL_PORT) == 0)
    {
        return;
    }
}

// --------------------------------------------------
// 0x9C status request: temperature, speed, angle
// --------------------------------------------------
void requestStatus()
{
    CAN_message_t msg = {};
    msg.id = 0x140 + MOTOR_ID;
    msg.flags.extended = 0;
    msg.flags.remote = 0;
    msg.len = 8;

    msg.buf[0] = 0x9C;
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    sendFrame(msg);
}

// --------------------------------------------------
// process incoming CAN
// --------------------------------------------------
void onCanMessage(const CAN_message_t &msg)
{
    for (auto odrive : odrives)
    {
        onReceive(msg, *odrive);
    }

    // Serial.print("INT RX ID: 0x");
    // Serial.println(msg.id, HEX);
}

void handleCAN()
{
    CAN_message_t rx;
    while (can_intf.read(rx))
    {
        onCanMessage(rx);

        if (rx.id != (0x240 + MOTOR_ID))
            continue;

        if (rx.buf[0] != 0x9C)
            continue;

        current_temp_c = (int8_t)rx.buf[1];
        current_speed_dps = le16_to_i16(rx.buf[4], rx.buf[5]);
        int16_t angle_raw = le16_to_i16(rx.buf[6], rx.buf[7]);

        // 0x9C angle is 1 degree/LSB
        static float initial_angle_offset = 0.0f;
        static bool angle_initialized = false;

        if (!angle_initialized)
        {
            initial_angle_offset = (float)angle_raw;
            angle_initialized = true;
        }

        current_angle_deg = (float)angle_raw - initial_angle_offset;

        applyMotorVelocity();
    }
}

void setup()
{
    Serial.begin(115200);
    can_intf.begin();
    can_intf.setBaudRate(500000);

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
    odrv7.onFeedback(onFeedback, &odrv7_user_data);
    odrv7.onStatus(onHeartbeat, &odrv7_user_data);

    TS4::begin();
    stepperB.setMaxSpeed(1500);
    stepperB.setAcceleration(100000);
    stepperG.setMaxSpeed(20000);
    stepperG.setAcceleration(1000000);
    delay(1);

    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    FastLED.show();

    setupEthernet();

    delay(500);
    setAcceleration(MOTOR_ID, 50);
    delay(1000);
    sendPositionCommand(MOTOR_ID, 0, 5);
    // servo1.attach(SERVO_PIN);
    // servo2.attach(SERVO_PIN2);
    // servo1.write(10);
    // servo2.write(10);
    pinMode(33, OUTPUT);
    showBlue();
}
void loop()
{
    static unsigned long last_odrive_check = 0;
    if (millis() - last_odrive_check > 10)
    {
        last_odrive_check = millis();
        if (!odrv0_user_data.available && odrv0_user_data.received_heartbeat)
        {
            odrv0_user_data.available = true;
            odrv0.clearErrors();
            delay(5);
            odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        }
        if (!odrv1_user_data.available && odrv1_user_data.received_heartbeat)
        {
            odrv1_user_data.available = true;
            odrv1.clearErrors();
            delay(5);
            odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv1.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        }
        if (!odrv2_user_data.available && odrv2_user_data.received_heartbeat)
        {
            odrv2_user_data.available = true;
            odrv2.clearErrors();
            delay(5);
            odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv2.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        }
        if (!odrv3_user_data.available && odrv3_user_data.received_heartbeat)
        {
            odrv3_user_data.available = true;
            odrv3.clearErrors();
            delay(5);
            odrv3.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv3.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        }
        if (!odrv4_user_data.available && odrv4_user_data.received_heartbeat)
        {
            odrv4_user_data.available = true;
            odrv4.clearErrors();
            delay(5);
            odrv4.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv4.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ);
            odrv4.setPosition(0.0);
        }
        if (!odrv5_user_data.available && odrv5_user_data.received_heartbeat)
        {
            odrv5_user_data.available = true;
            odrv5.clearErrors();
            delay(5);
            odrv5.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv5.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
            // odrv5.setPosition(-100.0);
        }
        if (!odrv7_user_data.available && odrv7_user_data.received_heartbeat)
        {
            odrv7_user_data.available = true;
            odrv7.clearErrors();
            delay(5);
            odrv7.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
            odrv7.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ);
            odrv7.setPosition(0.0);
        }
    }

    if (millis() - last_status_request_ms >= 20)
    {
        last_status_request_ms = millis();
        requestStatus();
    }

    if (millis() - lastWheelTime > 500 && lastWheelTime > 0)
    {
        if (!wheelTimeoutTriggered)
        {
            // wheel(1500, 1500);
            wheelTimeoutTriggered = true;
        }
    }

    if (millis() - lastArmTime > 500 && lastArmTime > 0)
    {
        if (!armTimeoutTriggered)
        {
            setVelocity(0);
            odrv5.setVelocity(0);
            if (stepperB.isMoving)
                stepperB.emergencyStop();
            if (stepperG.isMoving)
                stepperG.emergencyStop();
            armTimeoutTriggered = true;
        }
    }
    if (isGreen)
    {
        showGreen();
    }

    handleCAN();
    receiveUdpPackets();
}

main.cpp
Displaying main.cpp.