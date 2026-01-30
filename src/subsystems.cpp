// subsystems.cpp
#include "subsystems.h"
#include "robot_config.h"
#include <algorithm>
#include <cmath>

using namespace vex;

Alliance myAlliance = BLUE;
Wings wings;

volatile bool g_sorterEnabled = false;

static volatile bool g_sorterOverrideActive = false;

static volatile int g_intakeDir = 0;
static volatile double g_intakePct = 0.0;

static volatile int g_outakeDir = 0;
static volatile double g_outakePct = 0.0;

void setSorterEnabled(bool enabled) {
    g_sorterEnabled = enabled;
}

void Wings::toggle() {
    state = !state;
    wingsPiston.set(state);
}

void Wings::set(bool s) {
    state = s;
    wingsPiston.set(state);
}

void runIntake(double speedPct) {
    g_intakeDir = 1;
    g_intakePct = std::fabs(speedPct);

    if (g_sorterOverrideActive) return;

    MainIntake.spin(fwd, speedPct, pct);
    ColorIntake.spin(fwd, speedPct, pct);
}

void reverseIntake(double speedPct) {
    g_intakeDir = -1;
    g_intakePct = std::fabs(speedPct);

    if (g_sorterOverrideActive) return;

    MainIntake.spin(reverse, speedPct, pct);
    ColorIntake.spin(reverse, speedPct, pct);
}

void stopIntake() {
    g_intakeDir = 0;
    g_intakePct = 0.0;

    if (g_sorterOverrideActive) return;

    MainIntake.stop(coast);
    ColorIntake.stop(coast);
}

void runOutake(double speedPct) {
    g_outakeDir = 1;
    g_outakePct = std::fabs(speedPct);

    Outtake.spin(fwd, speedPct, pct);
}

void reverseOutake(double speedPct) {
    g_outakeDir = -1;
    g_outakePct = std::fabs(speedPct);

    Outtake.spin(reverse, speedPct, pct);
}

void stopOutake() {
    g_outakeDir = 0;
    g_outakePct = 0.0;

    Outtake.stop(coast);
}

void moveArmRight(double speedPct) {
    DescoreMotor.spin(fwd, speedPct, pct);
}

void moveArmLeft(double speedPct) {
    DescoreMotor.spin(reverse, speedPct, pct);
}

void stopArm() {
    DescoreMotor.stop(hold);
}

void runIntakeAuto(double speedPct) {
    setSorterEnabled(true);
    runIntake(speedPct);
}

void reverseIntakeAuto(double speedPct) {
    setSorterEnabled(false);
    reverseIntake(speedPct);
}

void stopIntakeAuto() {
    stopIntake();
    setSorterEnabled(false);
}

static double filteredHue() {
    static double buf[5] = {0, 0, 0, 0, 0};
    static int idx = 0;
    static int count = 0;

    buf[idx] = ballSensor.hue();
    idx = (idx + 1) % 5;
    if (count < 5) count++;

    double tmp[5];
    for (int i = 0; i < count; i++) tmp[i] = buf[i];

    std::sort(tmp, tmp + count);
    return tmp[count / 2];
}

int intakeTaskFn() {
    ballSensor.setLightPower(100, percent);

    const int ACCEPT_MS = 120;
    const int REJECT_MS = 220;
    const int COOLDOWN_MS_AFTER = 400;

    int cooldownMs = 0;

    while (true) {
        if (!g_sorterEnabled) {
            cooldownMs = 0;
            if (g_sorterOverrideActive) {
                ColorIntake.stop(coast);
                MainIntake.stop(coast);
                g_sorterOverrideActive = false;
            }
            wait(20, msec);
            continue;
        }

        if (cooldownMs > 0) {
            cooldownMs -= 20;
            wait(20, msec);
            continue;
        }

        if (!ballSensor.isNearObject()) {
            wait(20, msec);
            continue;
        }

        double hue = filteredHue();

        bool isRed  = (hue < 20 || hue > 340);
        bool isBlue = (hue > 200 && hue < 250);

        if (!isRed && !isBlue) {
            wait(20, msec);
            continue;
        }

        bool isOpponent =
            (myAlliance == RED  && isBlue) ||
            (myAlliance == BLUE && isRed);

        g_sorterOverrideActive = true;

        MainIntake.stop(coast);

        if (isOpponent) {
            ColorIntake.spin(reverse, 100, pct);
            wait(REJECT_MS, msec);
        } else {
            ColorIntake.spin(fwd, 100, pct);
            wait(ACCEPT_MS, msec);
        }

        ColorIntake.stop(coast);

        g_sorterOverrideActive = false;

        cooldownMs = COOLDOWN_MS_AFTER;

        wait(20, msec);
    }

    return 0;
}
