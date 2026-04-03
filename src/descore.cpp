#include "descore.h"
#include "robot_config.h"
#include "utils.h"
#include <cmath>

using namespace vex;

// Lever arm positions using LeverArmPot
static constexpr double kLeverOriginDeg   = 1.2;
static constexpr double kLeverDeployedDeg = 248.0;

// Control tuning
static constexpr double kLeverKp          = 0.7;
static constexpr double kLeverDeadbandDeg = 1.0;
static constexpr double kLeverMaxPct      = 75.0;
static constexpr double kLeverMinPct      = 8.0;

// Hold for 1 second after releasing the button
static constexpr int kReleaseHoldMs = 1000;

// State for release-hold behavior
static bool   gPrevDeployed      = false;
static bool   gReleaseHoldActive = false;
static double gReleaseHoldDeg    = 0.0;
static timer  gReleaseTimer;

static double leverAngleDeg() {
    return LeverArmPot.angle(deg);
}

void updateLeverArm(bool deployed) {
    const double currentDeg = leverAngleDeg();

    // Detect button release: pressed last loop, not pressed now
    if (gPrevDeployed && !deployed) {
        gReleaseHoldActive = true;
        gReleaseHoldDeg    = currentDeg;   // hold wherever it was released
        gReleaseTimer.reset();
    }

    gPrevDeployed = deployed;

    double targetDeg;

    if (deployed) {
        // While button is held, go to deployed and cancel release-hold
        gReleaseHoldActive = false;
        targetDeg = kLeverDeployedDeg;
    } else if (gReleaseHoldActive && gReleaseTimer.time(msec) < kReleaseHoldMs) {
        // For 1 second after release, hold current release position
        targetDeg = gReleaseHoldDeg;
    } else {
        // After 1 second, go back home
        gReleaseHoldActive = false;
        targetDeg = kLeverOriginDeg;
    }

    const double error = targetDeg - currentDeg;

    if (std::fabs(error) <= kLeverDeadbandDeg) {
        LeverArm.stop(hold);
        return;
    }

    double output = kLeverKp * error;

    if (output > 0.0) {
        output = clampD(output, kLeverMinPct, kLeverMaxPct);
        LeverArm.spin(fwd, output, pct);
    } else {
        output = clampD(-output, kLeverMinPct, kLeverMaxPct);
        LeverArm.spin(reverse, output, pct);
    }
}

void initLeverArm() {
    LeverArm.setStopping(coast);
    gPrevDeployed      = false;
    gReleaseHoldActive = false;
    gReleaseHoldDeg    = leverAngleDeg();
}

void printArmAngleControllerUpdate() {
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("ARM: %.1f deg", leverAngleDeg());
}