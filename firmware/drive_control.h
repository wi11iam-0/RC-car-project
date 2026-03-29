#pragma once
#include <Arduino.h>
#include "network_module.h"   // for RemoteControllerState

// DriveControl: maps controller state -> left/right motor commands (-1..+1)
// - D-pad up/down latches direction (Forward/Reverse) for straight driving
// - Direction changes are ignored unless throttle is ~0
// - Throttle (0..1) becomes speed magnitude
// - LX (left stick X-axis) commands turning:
//     LX < 0 => turn left  (left motor reverse, right motor forward)
//     LX > 0 => turn right (left motor forward, right motor reverse)
//   Turn strength scales with |LX|.
// - Failsafe: if controller packets stop for > failsafeTimeoutMs -> both = 0

class DriveControl {
public:
  enum class Direction : int8_t { Forward = 1, Reverse = -1 };

  struct DriveCmd {
    float left = 0.0f;   // -1..+1
    float right = 0.0f;  // -1..+1
  };

  struct Config {
    Direction initialDirection = Direction::Forward;

    // Only allow direction changes when throttle is below this threshold
    float throttleEpsForDirChange = 0.03f;

    // Ignore tiny throttle noise
    float throttleDeadband = 0.02f;

    // Ignore tiny LX noise around centre
    float lxDeadband = 0.08f;

    // Stop motor if last controller packet is older than this
    uint32_t failsafeTimeoutMs = 500;
  };

  void begin(const Config& cfg);
  void begin();  // overload: uses defaults

  // Returns left/right motor commands in [-1..+1].
  DriveCmd update(const RemoteControllerState& cs,
                  uint32_t lastControllerRxMs,
                  uint32_t nowMs);

  Direction getDirection() const { return _dir; }

private:
  Config _cfg{};
  Direction _dir = Direction::Forward;

  static float clampf(float x, float lo, float hi);
};
