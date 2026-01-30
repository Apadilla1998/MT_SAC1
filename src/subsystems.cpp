// subsystems.cpp
#include "subsystems.h"
#include "robot_config.h"
#include <algorithm>
#include <cmath>

using namespace vex;

// =====================================================
// Globals (declared in subsystems.h)
// =====================================================
Alliance myAlliance = BLUE;
Wings wings;

volatile bool g_sorterEnabled = false;

// =====================================================
// Internal state (tracks *requested* commands)
// =====================================================
static volatile bool   g_sorterOverrideActive = false;

static volatile int    g_intakeDir = 0;      // -1 rev, 0 stop, +1 fwd
static volatile double g_intakePct = 0.0;    // 0..100

static volatile int    g_outakeDir = 0;      // -1 rev, 0 stop, +1 fwd
static volatile double g_outakePct = 0.0;    // 0..100

// =====================================================
// Tunables
// =====================================================
static constexpr int kLoopMs = 20;

static constexpr int kHueBufN = 5;

// hue thresholds (tweak if needed)
static constexpr double kRedLowMax  = 20.0;
static constexpr double kRedHighMin = 340.0;
static constexpr double kBlueMin    = 200.0;
static constexpr double kBlueMax    = 250.0;

// sorter timings
static constexpr int kAcceptMs   = 120;
static constexpr int kRejectMs   = 220;
static constexpr int kCooldownMs = 400;

// =====================================================
// Public API
// =====================================================
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

// -------------------- Intake --------------------
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

// -------------------- Outtake --------------------
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

// -------------------- Descore arm --------------------
void moveArmRight(double speedPct) { DescoreMotor.spin(fwd,     speedPct, pct); }
void moveArmLeft (double speedPct) { DescoreMotor.spin(reverse, speedPct, pct); }
void stopArm()                    { DescoreMotor.stop(hold); }

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

// =====================================================
// Internal helpers
// =====================================================
enum class BallColor { RED, BLUE, UNKNOWN };

static BallColor classifyHue(double hue) {
    const bool isRed  = (hue < kRedLowMax || hue > kRedHighMin);
    const bool isBlue = (hue > kBlueMin && hue < kBlueMax);
    if (isRed)  return BallColor::RED;
    if (isBlue) return BallColor::BLUE;
    return BallColor::UNKNOWN;
}

static bool isOpponentBall(BallColor c) {
    if (c == BallColor::UNKNOWN) return false;
    return (myAlliance == RED  && c == BallColor::BLUE) ||
           (myAlliance == BLUE && c == BallColor::RED);
}

static double filteredHueMedian() {
    static double buf[kHueBufN] = {0,0,0,0,0};
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
        MainIntake.stop(coast);
        ColorIntake.stop(coast);
    } else if (dir > 0) {
        MainIntake.spin(fwd,  pctReq, pct);
        ColorIntake.spin(fwd, pctReq, pct);
    } else {
        MainIntake.spin(reverse,  pctReq, pct);
        ColorIntake.spin(reverse, pctReq, pct);
    }
}

static void beginSorterOverride() {
    g_sorterOverrideActive = true;
    MainIntake.stop(coast);     // ONLY ColorIntake spins during sort
}

static void endSorterOverride() {
    ColorIntake.stop(coast);
    g_sorterOverrideActive = false;
    restoreIntakeFromRequested();
}

// =====================================================
// Sorter task (start with: task sorter(intakeTaskFn);)
// =====================================================
int intakeTaskFn() {
    ballSensor.setLightPower(100, percent);

    int cooldownMs = 0;

    while (true) {
        if (!g_sorterEnabled) {
            if (g_sorterOverrideActive) {
                endSorterOverride();
            }
            cooldownMs = 0;
            wait(kLoopMs, msec);
            continue;
        }

        if (cooldownMs > 0) {
            cooldownMs -= kLoopMs;
            wait(kLoopMs, msec);
            continue;
        }

        if (!ballSensor.isNearObject()) {
            wait(kLoopMs, msec);
            continue;
        }

        const double hue = filteredHueMedian();
        const BallColor c = classifyHue(hue);

        if (c == BallColor::UNKNOWN) {
            wait(kLoopMs, msec);
            continue;
        }

        beginSorterOverride();

        if (isOpponentBall(c)) {
            ColorIntake.spin(reverse, 100, pct);
            wait(kRejectMs, msec);
        } else {
            ColorIntake.spin(fwd, 100, pct);
            wait(kAcceptMs, msec);
        }

        endSorterOverride();
        cooldownMs = kCooldownMs;

        wait(kLoopMs, msec);
    }

    return 0;
}
