#include <Arduino.h>
#include <Wire.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "teensystep4.h"

using namespace TS4;


byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress myIP(192, 168, 1, 177);


const IPAddress allowedIP(192, 168, 1, 11);

const unsigned int port = 8888;
EthernetUDP Udp;

const uint8_t STEP_PIN = 1;
const uint8_t DIR_PIN  = 3;

Stepper stepperB(STEP_PIN, DIR_PIN);

// Tune these
const int    SPEED_STEPS_PER_S = 16000;
const int    ACCEL_STEPS_PER_S2 = 500;
const float  DEADBAND_DEG = 2.0f;     // stop when within +/- this many degrees
const uint32_t CONTROL_PERIOD_MS = 20; // control loop update rate

// -------------------- AS5600 --------------------
static const uint8_t AS5600_ADDR = 0x36;

// Desired absolute azimuth (True North reference)
float desired_deg = NAN;

// Encoder offset that makes True North = 0 deg
float north_offset_deg = 0.0f;
bool isCalibrated = false;

// Control state
static int last_dir = 0;
static uint32_t lastControlMs = 0;

static float wrap360(float x) {
  while (x < 0.0f) x += 360.0f;
  while (x >= 360.0f) x -= 360.0f;
  return x;
}

// shortest signed difference desired-current in [-180, +180]
static float shortestErrorDeg(float desired, float current) {
  float e = fmodf(desired - current + 540.0f, 360.0f) - 180.0f;
  return e;
}

// Read AS5600 RAW ANGLE registers (0x0C/0x0D). Returns NAN on failure.
float readAS5600DegRaw() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); // RAW ANGLE (MSB)
  if (Wire.endTransmission(false) != 0) return NAN;

  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return NAN;

  uint16_t high = Wire.read();
  uint16_t low  = Wire.read();
  uint16_t raw = ((high & 0x0F) << 8) | low; // 12-bit
  return (raw * 360.0f) / 4096.0f;
}

// Current azimuth in True North reference, 0..360
float readCurrentAzimuthDeg() {
  float enc = readAS5600DegRaw();
  if (isnan(enc)) return NAN;
  if (!isCalibrated) return NAN; // enforce your requirement
  return wrap360(enc - north_offset_deg);
}

void stopMotor() {
  if (stepperB.isMoving) stepperB.emergencyStop();
  last_dir = 0;
}

void startMotorDir(int dir) {
  // dir: -1 left, +1 right
  if (dir == 0) { stopMotor(); return; }

  if (stepperB.isMoving && dir != last_dir) {
    stepperB.emergencyStop();
  }
  if (dir != last_dir) {
    // Start moving in that direction. Rotate a long chunk; control loop will stop it near target.
    stepperB.rotateAsync((long)SPEED_STEPS_PER_S * dir);
    last_dir = dir;
  }
}

// Control loop: compare desired_deg vs encoder and decide LEFT/RIGHT/STOP
void updateControl() {
  if (!isCalibrated) return;      // do nothing until CAL
  if (isnan(desired_deg)) return; // no target set

  float current = readCurrentAzimuthDeg();
  if (isnan(current)) return;

  float err = shortestErrorDeg(desired_deg, current);

  int dir = 0;
  if (err > DEADBAND_DEG) dir = +1;
  else if (err < -DEADBAND_DEG) dir = -1;
  else dir = 0;

  if (dir == 0) stopMotor();
  else startMotorDir(dir);
}

// Parse commands:
// - CAL
// - AZ:123.45
// - LEFT / RIGHT / STOP (optional manual)
void applyCommand(const char* cmd) {
  // trim leading spaces
  while (*cmd == ' ' || *cmd == '\t' || *cmd == '\n' || *cmd == '\r') cmd++;

  if (strncmp(cmd, "CAL", 3) == 0) {
    float enc = readAS5600DegRaw();
    if (!isnan(enc)) {
      north_offset_deg = enc;
      isCalibrated = true;
      desired_deg = NAN; // clear target until next AZ
      stopMotor();
      Serial.print("CAL OK. north_offset_deg=");
      Serial.println(north_offset_deg, 2);
    } else {
      Serial.println("CAL failed: AS5600 read error");
    }
    return;
  }

  if (strncmp(cmd, "AZ:", 3) == 0) {
    if (!isCalibrated) {
      Serial.println("Ignoring AZ: not calibrated (send CAL while pointing True North)");
      return;
    }
    float val = atof(cmd + 3);
    desired_deg = wrap360(val);
    Serial.print("AZ set to ");
    Serial.println(desired_deg, 2);
    return;
  }

  // if (strncmp(cmd, "ROT:0", 5) == 0) {
  //   if (!isCalibrated) {
  //     Serial.println("Ignoring ROT:0: not calibrated (send CAL while pointing True North)");
  //     return;
  //   }
  //   desired_deg = 0.0f;
  //   Serial.println("ROT:0 - Rotating to 0 degrees (True North)");
  //   return;
  // }

  // Optional manual override
//   if (strncmp(cmd, "LEFT", 4) == 0) {
//     desired_deg = NAN;
//     isCalibrated = isCalibrated; // no change
//     startMotorDir(-1);
//     Serial.println("Manual LEFT");
//     return;
//   }
//   if (strncmp(cmd, "RIGHT", 5) == 0) {
//     desired_deg = NAN;
//     startMotorDir(+1);
//     Serial.println("Manual RIGHT");
//     return;
//   }
//   if (strncmp(cmd, "STOP", 4) == 0) {
//     desired_deg = NAN;
//     stopMotor();
//     Serial.println("Manual STOP");
//     return;
//   }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  TS4::begin();
  stepperB.setMaxSpeed(SPEED_STEPS_PER_S);
  stepperB.setAcceleration(ACCEL_STEPS_PER_S2);
  stepperB.setPosition(0);

  Wire.begin();
  Wire.setClock(400000);

  Ethernet.begin(mac, myIP);
  delay(100);
  Udp.begin(port);

  Serial.print("Teensy IP: ");
  Serial.println(Ethernet.localIP());
  Serial.println("Waiting for CAL (point antenna to True North, then send 'CAL')");
}

void loop() {
  // UDP receive
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    IPAddress src = Udp.remoteIP();
    if (src != allowedIP) {
      while (Udp.available()) { uint8_t tmp; Udp.read(&tmp, 1); }
      Serial.print("Ignored packet from ");
      Serial.println(src);
    } else {
      char buf[128] = {0};
      int len = Udp.read((uint8_t*)buf, sizeof(buf) - 1);
      if (len > 0) {
        buf[len] = '\0';
        applyCommand(buf);
      }
    }
  }

  // Control loop at fixed rate
  uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    updateControl();
  }
}