#ifndef DESCORE_H
#define DESCORE_H

#include "robot_config.h"
#include "utils.h"
#include <cmath>

// ---- ARM SETTINGS (Left=Up, Right=Down) ----
// 1. TARGETS (POT readings in degrees)
constexpr double ARM_DOWN_DEG = 239.0;
constexpr double ARM_UP_DEG   = 181.4;

// 2. VARIABLE SPEED (NO PID)
// Slow near either end-cap, fastest in the middle between UP and DOWN.
constexpr double ARM_MIN_PWR = 10.0;  // power near either end (raise if it stalls)
constexpr double ARM_MAX_PWR = 60.0;  // power at the midpoint (fastest)

// 3. DEADBAND
constexpr double ARM_DEADBAND = 3.0;

// Optional: helps prevent “banging”
constexpr double ARM_SLEW_PCT_PER_S = 500.0; // command change limit (pct/sec)
constexpr double ARM_DT_S           = 0.02;  // call armUpdateSimple() every 20ms

// If pot angle increases when DescoreMotor spins fwd, keep true.
// If it decreases, set false.
constexpr bool ARM_FWD_INCREASES_POT = true;

// Shared state (defined once in descore.cpp)
extern double armTargetDeg;
extern bool   armActive;

// Slew State (kept for compatibility)
extern double lCmd;
extern double rCmd;

// Functions
void armSetTarget(double targetDeg);
void armSetUp();
void armSetDown();
void armEnable(bool en);

void armUpdateSimple();

void armMoveTo(double targetDeg, int timeoutMs);
void armMoveBy(double deltaDeg, int timeoutMs);

void printArmAngleControllerUpdate();

#endif