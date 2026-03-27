#include "descore.h"
#include "robot_config.h"
#include "utils.h"
#include <cmath>

using namespace vex;

// Lever arm positions
static constexpr double kLeverOriginDeg   = 12.9; //0.5
static constexpr double kLeverDeployedDeg = 9.3;

// Basic control tuning
static constexpr double kLeverKp          = 0.85;
static constexpr double kLeverDeadbandDeg = 3.0;
static constexpr double kLeverMaxPct      = 75.0;

// Move lever to deployed or origin position
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
        LeverArm.spin(fwd, std::fabs(output), pct);
    } else {
        LeverArm.spin(reverse, std::fabs(output), pct);
    }
}

// Hold lever at startup
void initLeverArm() {
    LeverArm.stop(hold);
}

// Show descore arm angle on controller
void printArmAngleControllerUpdate() {
    //const double angle = DescorePot.angle(deg);
    const double angle = LeverArmPot.angle(deg);

    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("ARM: %.1f deg", angle);
}