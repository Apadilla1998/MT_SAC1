#include "subsystems.h"
#include "robot_config.h"
#include <algorithm>
#include <cmath>

using namespace vex;

// ============================================================
// Globals
// ============================================================
SortTargetColor g_sortTargetColor = SortTargetColor::BLUE;
Alliance myAlliance = Alliance::BLUE;

Wings wings;
BallLoader ballLoader;

volatile bool g_sorterEnabled = false;

// Requested intake state so it can be restored after sorting
static volatile bool   g_sorterOverrideActive = false;
static volatile int    g_intakeDir = 0;     // 1=fwd, -1=rev, 0=stop
static volatile double g_intakePct = 0.0;

// ============================================================
// Tunables
// ============================================================
static constexpr int kLoopMs = 5;
static constexpr int kHueBufN = 5;

// Ball color ranges
static constexpr double kRedLowMax  = 30.0;
static constexpr double kRedHighMin = 340.0;
static constexpr double kBlueMin    = 180.0;
static constexpr double kBlueMax    = 270.0;

// Detection confidence
static constexpr int kConfirmSamples = 5;
static constexpr int kCooldownMs     = 80;

// Sorter positions
static constexpr double kColorSortHomeDeg      = 135; // 138.4
static constexpr double kColorSortRejectDeg    = 86;
static constexpr double kColorSortToleranceDeg = 0.5;

// Sorter motor control
static constexpr double kColorSortKp        = 0.65;
static constexpr double kColorSortMaxPct    = 75.0;
static constexpr double kColorSortMinPct    = 10.0;
static constexpr int    kColorSortTimeoutMs = 400;

// Hold time at reject position
static constexpr int kRejectHoldMs = 150;

// forward declaration
static void moveColorSortToAngle(double targetDeg, int timeoutMs = kColorSortTimeoutMs);

// ============================================================
// Public API
// ============================================================
void setSorterEnabled(bool enabled) {
    g_sorterEnabled = enabled;
}

void setSortTargetColor(SortTargetColor color) {
    g_sortTargetColor = color;
}

void homeColorSort() {
    moveColorSortToAngle(kColorSortHomeDeg);
}

void Wings::toggle() {
    state = !state;
    wingsPiston.set(state);
}

void Wings::set(bool s) {
    state = s;
    wingsPiston.set(state);
}

void BallLoader::toggles() {
    state = !state;
    loader.set(state);
}

void BallLoader::sets(bool s) {
    state = s;
    loader.set(state);
}

// ============================================================
// Intake
// ============================================================
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

// ============================================================
// Descore arm
// ============================================================
void moveArmRight(double speedPct) {
    DescoreMotor.spin(fwd, speedPct, pct);
}

void moveArmLeft(double speedPct) {
    DescoreMotor.spin(reverse, speedPct, pct);
}

void stopArm() {
    DescoreMotor.stop(hold);
}

// ============================================================
// Autonomous helpers
// ============================================================
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

// ============================================================
// Internal helpers
// ============================================================
enum class BallColor {
    RED,
    BLUE,
    UNKNOWN
};

static BallColor classifyHue(double hue) {
    const bool isRed  = (hue <= kRedLowMax) || (hue >= kRedHighMin);
    const bool isBlue = (hue >= kBlueMin) && (hue <= kBlueMax);

    if (isRed)  return BallColor::RED;
    if (isBlue) return BallColor::BLUE;
    return BallColor::UNKNOWN;
}

static bool shouldSortBall(BallColor c) {
    if (c == BallColor::UNKNOWN) return false;

    switch (g_sortTargetColor) {
        case SortTargetColor::OFF:
            return false;
        case SortTargetColor::RED:
            return c == BallColor::RED;
        case SortTargetColor::BLUE:
            return c == BallColor::BLUE;
        default:
            return false;
    }
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

static void beginSortOverride() {
    g_sorterOverrideActive = true;
}

static void endSortOverride() {
    g_sorterOverrideActive = false;
    restoreIntakeFromRequested();
}

static void spinColorSortMotor(double signedPct) {
    if (signedPct >= 0.0) {
        colorSortMotor.spin(fwd, signedPct, pct);
    } else {
        colorSortMotor.spin(reverse, -signedPct, pct);
    }
}

static void moveColorSortToAngle(double targetDeg, int timeoutMs) {
    timer t;
    t.reset();

    while (t.time(msec) < timeoutMs) {
        const double currentDeg = ColorSort.angle(deg);
        const double error = targetDeg - currentDeg;

        if (std::fabs(error) <= kColorSortToleranceDeg) {
            colorSortMotor.stop(hold);
            return;
        }

        double output = kColorSortKp * error;

        if (output >  kColorSortMaxPct) output =  kColorSortMaxPct;
        if (output < -kColorSortMaxPct) output = -kColorSortMaxPct;

        if (std::fabs(output) < kColorSortMinPct) {
            output = (error > 0.0) ? kColorSortMinPct : -kColorSortMinPct;
        }

        spinColorSortMotor(output);
        wait(10, msec);
    }

    colorSortMotor.stop(hold);
}

static void doColorSortReject() {
    beginSortOverride();

    moveColorSortToAngle(kColorSortRejectDeg);
    wait(kRejectHoldMs, msec);
    moveColorSortToAngle(kColorSortHomeDeg);

    endSortOverride();
}

// ============================================================
// Sorter task
// ============================================================
int intakeTaskFn() {
    ballSensor.setLightPower(100, percent);

    // Home the sorter at startup
    moveColorSortToAngle(kColorSortHomeDeg);

    int cooldownMs = 0;
    int redCount   = 0;
    int blueCount  = 0;
    bool waitForClear = false;

    while (true) {
        if (!g_sorterEnabled) {
            if (g_sorterOverrideActive) endSortOverride();
            redCount = 0;
            blueCount = 0;
            cooldownMs = 0;
            waitForClear = false;
            wait(kLoopMs, msec);
            continue;
        }

        if (waitForClear) {
            if (!ballSensor.isNearObject()) {
                waitForClear = false;
            }
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

        if (!shouldSortBall(decided)) {
            redCount = 0;
            blueCount = 0;
            cooldownMs = 40;
            wait(kLoopMs, msec);
            continue;
        }

        doColorSortReject();

        redCount = 0;
        blueCount = 0;
        cooldownMs = kCooldownMs;
        waitForClear = true;

        wait(kLoopMs, msec);
    }

    return 0;
}