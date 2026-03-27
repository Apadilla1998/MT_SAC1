#include "subsystems.h"
#include "robot_config.h"
#include <algorithm>
#include <cmath>

using namespace vex;

// Globals (declared in subsystems.h)
Alliance myAlliance;
Wings wings;
BallLoader ballLoader;

volatile bool g_sorterEnabled = false;

// Internal state (tracks requested commands)
static volatile bool   g_sorterOverrideActive = false;

static volatile int    g_intakeDir = 0;
static volatile double g_intakePct = 0.0;

// Tunables
static constexpr int kLoopMs = 5;
static constexpr int kHueBufN = 5;

// Hue thresholds
static constexpr double kRedLowMax  = 30.0;
static constexpr double kRedHighMin = 340.0;
static constexpr double kBlueMin    = 180.0;
static constexpr double kBlueMax    = 270.0;

// FAST decision window
static constexpr int kConfirmSamples = 6;

// Reject timings
static constexpr int kRejectMs   = 280;
static constexpr int kCooldownMs = 10;

// Reject motor strength
static constexpr int kRejectIntakePct = 100;

// Public API
void setSorterEnabled(bool enabled) { g_sorterEnabled = enabled; }

void Wings::toggle() { state = !state; wingsPiston.set(state); }
void Wings::set(bool s) { state = s; wingsPiston.set(state); }

void BallLoader::toggles() { state = !state; loader.set(state); }
void BallLoader::sets(bool s) { state = s; loader.set(state); }

// -------------------- Intake --------------------
void runIntake(double speedPct) {
    g_intakeDir = 1;
    g_intakePct = std::fabs(speedPct);

    if (g_sorterOverrideActive) return;

    Intake.spin(fwd, speedPct, pct);
}

void reverseIntake(double speedPct) {
    g_intakeDir = -1;
    g_intakePct = std::fabs(speedPct);

    if (g_sorterOverrideActive) return;

    Intake.spin(reverse, speedPct, pct);
}

void stopIntake() {
    g_intakeDir = 0;
    g_intakePct = 0.0;

    if (g_sorterOverrideActive) return;

    Intake.stop(coast);
}

// -------------------- Descore arm --------------------
void moveArmRight(double speedPct) { DescoreMotor.spin(fwd, speedPct, pct); }
void moveArmLeft (double speedPct) { DescoreMotor.spin(reverse, speedPct, pct); }
void stopArm()                     { DescoreMotor.stop(hold); }

// -------------------- Auto helpers --------------------
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

// Internal helpers
enum class BallColor { RED, BLUE, UNKNOWN };

static BallColor classifyHue(double hue) {
    const bool isRed  = (hue <= kRedLowMax) || (hue >= kRedHighMin);
    const bool isBlue = (hue >= kBlueMin) && (hue <= kBlueMax);
    if (isRed)  return BallColor::RED;
    if (isBlue) return BallColor::BLUE;
    return BallColor::UNKNOWN;
}

static bool isOpponentBall(BallColor c) {
    if (c == BallColor::UNKNOWN) return false;
    return (myAlliance == Alliance::RED  && c == BallColor::BLUE) ||
           (myAlliance == Alliance::BLUE && c == BallColor::RED);
}

static double filteredHueMedian() {
    static double buf[kHueBufN] = {0, 0, 0, 0, 0};
    static int idx = 0;
    static int count = 0;

    buf[idx] = ballSensor.hue();
    idx = (idx + 1) % kHueBufN;
    if (count < kHueBufN) count++;

    double tmp[kHueBufN];
    for (int i = 0; i < count; i++) tmp[i] = buf[i];
    std::sort(tmp, tmp + count);
    return tmp[count / 2];
}

static void restoreIntakeFromRequested() {
    const int dir = g_intakeDir;
    const double pctReq = g_intakePct;

    if (dir == 0 || pctReq <= 0.0) {
        Intake.stop(coast);
    } else if (dir > 0) {
        Intake.spin(fwd, pctReq, pct);
    } else {
        Intake.spin(reverse, pctReq, pct);
    }
}

static void beginRejectOverride() {
    g_sorterOverrideActive = true;
    Intake.stop(coast);
}

static void endSorterOverride() {
    Intake.stop(coast);

    g_sorterOverrideActive = false;

    restoreIntakeFromRequested();
}

static void runRejectMotors() {
    Intake.spin(reverse, kRejectIntakePct, pct);
}

// Sorter task
int intakeTaskFn() {
    ballSensor.setLightPower(100, percent);

    int cooldownMs = 0;
    int redCount   = 0;
    int blueCount  = 0;

    while (true) {
        if (!g_sorterEnabled) {
            if (g_sorterOverrideActive) endSorterOverride();
            cooldownMs = 0;
            redCount = 0;
            blueCount = 0;
            wait(kLoopMs, msec);
            continue;
        }

        if (cooldownMs > 0) {
            cooldownMs -= kLoopMs;
            wait(kLoopMs, msec);
            continue;
        }

        if (!ballSensor.isNearObject()) {
            redCount = 0;
            blueCount = 0;
            wait(kLoopMs, msec);
            continue;
        }

        const double hue = filteredHueMedian();
        const BallColor c = classifyHue(hue);

        if (c == BallColor::RED) {
            redCount++;
            blueCount = 0;
        } else if (c == BallColor::BLUE) {
            blueCount++;
            redCount = 0;
        } else {
            redCount = 0;
            blueCount = 0;
            wait(kLoopMs, msec);
            continue;
        }

        if (redCount < kConfirmSamples && blueCount < kConfirmSamples) {
            wait(kLoopMs, msec);
            continue;
        }

        const BallColor decided =
            (redCount >= kConfirmSamples) ? BallColor::RED : BallColor::BLUE;

        if (!isOpponentBall(decided)) {
            redCount = 0;
            blueCount = 0;
            cooldownMs = 40;
            wait(kLoopMs, msec);
            continue;
        }

        beginRejectOverride();
        runRejectMotors();

        int tMs = 0;
        while (tMs < kRejectMs) {
            wait(kLoopMs, msec);
            tMs += kLoopMs;
        }

        endSorterOverride();

        redCount = 0;
        blueCount = 0;
        cooldownMs = kCooldownMs;

        wait(kLoopMs, msec);
    }

    return 0;
}