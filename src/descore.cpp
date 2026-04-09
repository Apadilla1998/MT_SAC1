#include "Descore.h"
#include "robot_config.h"
#include "utils.h"
#include <cmath>

using namespace vex;

// Descore target position
static constexpr double kDescoreTargetDeg   = 35.2;

// Control tuning
static constexpr double kDescoreKp          = 0.001;
static constexpr double kDescoreDeadbandDeg = 2;
static constexpr double kDescoreMaxPct      = 20.0;
static constexpr double kDescoreMinPct      = 8.0;

static double descoreAngleDeg() {
    return DescorePot.angle(deg);
}

static void moveDescoreTo(double targetDeg) {
    const double currentDeg = descoreAngleDeg();
    const double error      = targetDeg - currentDeg;

    if (std::fabs(error) <= kDescoreDeadbandDeg) {
        DescoreMotor.stop(hold);
        return;
    }

    double output = kDescoreKp * error;

    if (output > 0.0) {
        output = clampD(output, kDescoreMinPct, kDescoreMaxPct);
        DescoreMotor.spin(fwd, output, pct);
    } else {
        output = clampD(-output, kDescoreMinPct, kDescoreMaxPct);
        DescoreMotor.spin(reverse, output, pct);
    }
}

void updateDescore(bool active) {
    // Always go to 35.2 and hold there
    moveDescoreTo(kDescoreTargetDeg);
}

void initDescore() {
    DescoreMotor.setStopping(hold);
}

void printDescoreAngleControllerUpdate() {
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("DSCR: %.1f deg", descoreAngleDeg());
}