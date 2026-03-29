#pragma once
#include <Arduino.h>

// motor driver for a simple 2-input H-bridge
// - pin A handles forward pwm while pin B stays low
// - pin B handles reverse pwm while pin A stays low
// stop/coast just means both sides are low
class MotorDriver {
public:
  // signed motor command from -1.0 to +1.0
  // negative = reverse
  // positive = forward
  // zero = stop
  //
  // these defaults avoid the usual AI thinker camera pins
  // and also stay away from LEDC channel 0
  void begin(uint8_t pinA = 14,
             uint8_t pinB = 15,
             uint8_t pwmChannelA = 1,
             uint8_t pwmChannelB = 2);

  void setCommand(float signedCmd); // -1..+1
  void stop();                      // same as setCommand(0)

  // with this 2-pin setup, proper active braking depends on the hardware
  // for now brake() just behaves like a normal stop unless changed later
  void brake();

private:
  uint8_t _pinA = 255;
  uint8_t _pinB = 255;

  // each motor object keeps track of its own pwm channels
  uint8_t _pwmChannelA = 255;
  uint8_t _pwmChannelB = 255;

  // pwm settings
  static constexpr uint32_t kPwmFreqHz  = 20000; // 20 kHz, keeps it quieter
  static constexpr uint8_t  kPwmResBits = 10;    // range is 0..1023

  void writeDutyA(uint32_t duty);
  void writeDutyB(uint32_t duty);
};