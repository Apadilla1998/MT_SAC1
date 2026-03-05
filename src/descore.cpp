// src/descore.cpp
#include "descore.h"

double armTargetDeg = ARM_DOWN_DEG;
bool   armActive    = true;

// Slew State (kept)
double lCmd = 0.0;
double rCmd = 0.0;

static inline double clampTarget(double t) {
  return clampD(t, ARM_UP_DEG, ARM_DOWN_DEG);
}

static inline double slewTo(double curr, double target, double maxDelta) {
  if (target > curr + maxDelta) return curr + maxDelta;
  if (target < curr - maxDelta) return curr - maxDelta;
  return target;
}

void armSetTarget(double targetDeg) { armTargetDeg = clampTarget(targetDeg); }
void armSetUp()  { armTargetDeg = ARM_UP_DEG; }
void armSetDown(){ armTargetDeg = ARM_DOWN_DEG; }
void armEnable(bool en) { armActive = en; if (!en) { DescoreMotor.stop(vex::hold); } }

// ---- SIMPLE ARM LOGIC WITH HARD STOPS + "FAST IN MIDDLE" SPEED PROFILE ----
void armUpdateSimple() {
  if (!armActive) { DescoreMotor.stop(vex::hold); return; }

  const double currentDeg = Descore.angle(vex::deg); // potV2
  const double error      = armTargetDeg - currentDeg;

  // SAFETY CAPS: Hard stop if we are past limits and trying to go further
  if (currentDeg <= ARM_UP_DEG && error < 0) {
    DescoreMotor.stop(vex::hold);
    lCmd = 0; rCmd = 0;
    return;
  }
  if (currentDeg >= ARM_DOWN_DEG && error > 0) {
    DescoreMotor.stop(vex::hold);
    lCmd = 0; rCmd = 0;
    return;
  }

  const double absErr = std::fabs(error);

  // Deadband near target
  if (absErr < ARM_DEADBAND) {
    DescoreMotor.stop(vex::hold);
    lCmd = 0; rCmd = 0;
    return;
  }

  // -----------------------------
  // Speed depends on POSITION, not error:
  // - slow near ARM_UP_DEG
  // - slow near ARM_DOWN_DEG
  // - fastest near the midpoint between them
  // -----------------------------
  double range = (ARM_DOWN_DEG - ARM_UP_DEG);
  if (range < 1e-6) range = 1.0;

  // Normalize position: 0 at UP, 1 at DOWN
  double pos = (currentDeg - ARM_UP_DEG) / range;
  pos = clampD(pos, 0.0, 1.0);

  // midFactor: 1 at middle, 0 at ends
  double midFactor = 1.0 - 2.0 * std::fabs(pos - 0.5);
  if (midFactor < 0.0) midFactor = 0.0;

  // Optional shaping for a punchier middle (comment out if you want linear)
  midFactor = midFactor * midFactor;

  // Power magnitude (ends -> ARM_MIN_PWR, middle -> ARM_MAX_PWR)
  const double mag = ARM_MIN_PWR + midFactor * (ARM_MAX_PWR - ARM_MIN_PWR);

  // Apply direction based on where target is
  double power = (error > 0) ? mag : -mag;

  // Map pot direction to motor direction (in case motor sign is flipped)
  if (!ARM_FWD_INCREASES_POT) power = -power;

  // Slew-rate limit using lCmd (prevents banging / instant reversals)
  const double maxDelta = ARM_SLEW_PCT_PER_S * ARM_DT_S;
  lCmd = slewTo(lCmd, power, maxDelta);

  // Final clamp (safety)
  lCmd = clampD(lCmd, -100.0, 100.0);

  // Drive motor (FIXED: use directionType::rev, not vex::rev)
  vex::directionType dir =
      (lCmd >= 0) ? vex::directionType::fwd : vex::directionType::rev;

  DescoreMotor.spin(dir, std::fabs(lCmd), vex::pct);
}

void armMoveTo(double targetDeg, int timeoutMs) {
  armSetTarget(targetDeg);

  vex::timer t; t.reset();
  while (t.time(vex::msec) < timeoutMs) {
    armUpdateSimple();
    const double e = armTargetDeg - Descore.angle(vex::deg);
    if (std::fabs(e) < ARM_DEADBAND) break;
    vex::wait(10, vex::msec);
  }
  DescoreMotor.stop(vex::hold);
  lCmd = 0;
}

void armMoveBy(double deltaDeg, int timeoutMs) {
  armMoveTo(Descore.angle(vex::deg) + deltaDeg, timeoutMs);
}

void printArmAngleControllerUpdate() {
  static int tick = 0;
  if (++tick % 5 != 0) return; // ~100ms if called every 20ms

  const double a = Descore.angle(vex::deg);
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Arm:%6.1f T:%6.1f   ", a, armTargetDeg);
}