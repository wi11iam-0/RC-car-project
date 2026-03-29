#include "motor_driver.h"
#include <math.h>

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline uint32_t maxDutyForRes(uint8_t resBits) {
  return (1u << resBits) - 1u;
}

void MotorDriver::begin(uint8_t pinA, uint8_t pinB, uint8_t pwmChannelA, uint8_t pwmChannelB) {
  _pinA = pinA;
  _pinB = pinB;
  _pwmChannelA = pwmChannelA;
  _pwmChannelB = pwmChannelB;

  pinMode(_pinA, OUTPUT);
  pinMode(_pinB, OUTPUT);
  digitalWrite(_pinA, LOW);
  digitalWrite(_pinB, LOW);

  // Two independent PWM channels so we can PWM either direction pin.
  ledcSetup(_pwmChannelA, kPwmFreqHz, kPwmResBits);
  ledcSetup(_pwmChannelB, kPwmFreqHz, kPwmResBits);

  ledcAttachPin(_pinA, _pwmChannelA);
  ledcAttachPin(_pinB, _pwmChannelB);

  // Start stopped
  writeDutyA(0);
  writeDutyB(0);
}

void MotorDriver::writeDutyA(uint32_t duty) {
  const uint32_t maxDuty = maxDutyForRes(kPwmResBits);
  if (duty > maxDuty) duty = maxDuty;
  ledcWrite(_pwmChannelA, duty);
}

void MotorDriver::writeDutyB(uint32_t duty) {
  const uint32_t maxDuty = maxDutyForRes(kPwmResBits);
  if (duty > maxDuty) duty = maxDuty;
  ledcWrite(_pwmChannelB, duty);
}

void MotorDriver::setCommand(float signedCmd) {
  // Clamp and apply a tiny deadband to avoid jitter
  float cmd = clampf(signedCmd, -1.0f, 1.0f);
  if (fabsf(cmd) < 0.02f) cmd = 0.0f;

  const float mag = fabsf(cmd);
  const uint32_t duty = (uint32_t)(mag * (float)maxDutyForRes(kPwmResBits));

  if (cmd > 0.0f) {
    // Forward: PWM on A, force B LOW (duty 0)
    writeDutyB(0);
    writeDutyA(duty);
  } else if (cmd < 0.0f) {
    // Reverse: PWM on B, force A LOW (duty 0)
    writeDutyA(0);
    writeDutyB(duty);
  } else {
    // Coast stop
    writeDutyA(0);
    writeDutyB(0);
  }
}

void MotorDriver::stop() {
  setCommand(0.0f);
}

void MotorDriver::brake() {
  // Safe default for this 2-pin bridge: coast.
  stop();
}
