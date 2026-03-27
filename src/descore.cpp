#include "descore.h"
#include "robot_config.h"
#include "utils.h"
#include <cmath>

using namespace vex;

// ============================================================
//  Lever Arm — pot-based position control (LeverArmPot on H)
// ============================================================

// Tune these two angles to match your physical robot.
// Read LeverArmPot.angle(deg) with the arm at each position to calibrate.
static constexpr double kLeverOriginDeg   = 10.0;   // stored / stowed position
static constexpr double kLeverDeployedDeg = 105.0;  // fully deployed position

// Proportional gain — increase if response is sluggish, decrease if it oscillates
static constexpr double kLeverKp          = 0.85;

// Dead-band: within this many degrees of target, stop and hold
static constexpr double kLeverDeadbandDeg = 3.0;

// Output clamp — keeps the motor from overpowering the mechanism
static constexpr double kLeverMaxPct      = 75.0;

// --------------------------------------------------------
// updateLeverArm
//   deployed = true  → move to kLeverDeployedDeg
//   deployed = false → return to kLeverOriginDeg
//
// Call this every control loop tick (≥ 20 ms).
// --------------------------------------------------------
void updateLeverArm(bool deployed) {
    const double targetDeg  = deployed ? kLeverDeployedDeg : kLeverOriginDeg;
    const double currentDeg = LeverArmPot.angle(deg);
    const double error      = targetDeg - currentDeg;

    if (std::fabs(error) < kLeverDeadbandDeg) {
        LeverArm.stop(hold);
        return;
    }

    double output = kLeverKp * error;
    output = clampD(output, -kLeverMaxPct, kLeverMaxPct);

    if (output > 0.0) {
        LeverArm.spin(fwd,     std::fabs(output), pct);
    } else {
        LeverArm.spin(reverse, std::fabs(output), pct);
    }
}

// Call once at startup if you want the arm to snap to origin immediately
void initLeverArm() {
    LeverArm.stop(hold);
}

// ============================================================
//  Descore Arm — print current pot angle to the controller
//  (uses DescorePot on G)
// ============================================================
void printArmAngleControllerUpdate() {
    const double angle = DescorePot.angle(deg);

    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("ARM: %.1f deg", angle);
}