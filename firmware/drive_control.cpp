#include "drive_control.h"
#include <math.h>

float DriveControl::clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void DriveControl::begin() {
  begin(Config{});
}

void DriveControl::begin(const Config& cfg) {
  _cfg = cfg;
  _dir = cfg.initialDirection;
}

DriveControl::DriveCmd DriveControl::update(const RemoteControllerState& cs,
                                            uint32_t lastControllerRxMs,
                                            uint32_t nowMs) {
  DriveCmd out{};

  // ---------- FAILSAFE ----------
  if (lastControllerRxMs == 0 || (nowMs - lastControllerRxMs) > _cfg.failsafeTimeoutMs) {
    return out;
  }

  if (!cs.valid) {
    return out;
  }

  const float throttle = clampf(cs.throttle, 0.0f, 1.0f);

  // ---------- DIRECTION LATCH ----------
  if (throttle <= _cfg.throttleEpsForDirChange) {
    if (cs.dpadY == 1) {
      _dir = Direction::Forward;
    } else if (cs.dpadY == -1) {
      _dir = Direction::Reverse;
    }
  }

  // ---------- THROTTLE MAPPING ----------
  float mag = throttle;
  if (mag < _cfg.throttleDeadband) mag = 0.0f;

  // ---------- TURNING (LX) ----------
  const float lx = clampf(cs.lx, -1.0f, 1.0f);
  const float absLx = fabsf(lx);

  if (mag > 0.0f && absLx >= _cfg.lxDeadband) {
    const float turnMag = clampf(mag * absLx, 0.0f, 1.0f);
    if (lx < 0.0f) {
      out.left  = -turnMag;
      out.right =  turnMag;
    } else {
      out.left  =  turnMag;
      out.right = -turnMag;
    }
    return out;
  }

  // ---------- STRAIGHT ----------
  const float straight = clampf(mag * (int8_t)_dir, -1.0f, 1.0f);
  out.left = straight;
  out.right = straight;
  return out;
}
