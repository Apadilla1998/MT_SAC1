#include "subsystems.h"
#include "robot_config.h"
#include <algorithm>
#include <cmath>

using namespace vex;

// Globals (declared in subsystems.h)
Alliance myAlliance = BLUE;
Wings wings;
BallLoader ballLoader;

volatile bool g_sorterEnabled = false;

// Internal state (tracks *requested* commands)
static volatile bool   g_sorterOverrideActive = false;

static volatile int    g_intakeDir = 0;
static volatile double g_intakePct = 0.0;

static volatile int    g_outakeDir = 0;
static volatile double g_outakePct = 0.0;

// Tunables
static constexpr int kLoopMs = 5; // how often you sample: kLoopsMS * KConfrim Samples for window
static constexpr int kHueBufN = 5; //decrease for faster reaction min 5

// Hue thresholds 
static constexpr double kRedLowMax  = 30.0;
static constexpr double kRedHighMin = 340.0;
static constexpr double kBlueMin    = 180.0;
static constexpr double kBlueMax    = 270.0;

// static constexpr double kRedLowMax  = 10.0;
// static constexpr double kRedHighMin = 365.0;
// static constexpr double kBlueMin    = 220.0;
// static constexpr double kBlueMax    = 245.0;

// FAST decision window
static constexpr int kConfirmSamples = 6; //incrase for fewer wrong rejects

// Reject timings
static constexpr int kRejectMs   = 280; //how long you spit it out
static constexpr int kCooldownMs = 10; //how long you wait before sorting

//reverse motors down a bit when it detects the color for the main intake

// Reject motor strengths
static constexpr int kRejectColorIntakePct = 100;

// If reject goes the wrong way, flip this dir between -1 and +1
// static constexpr int kRejectOuttakePct = 20;
// static constexpr int kRejectOuttakeDir = -1; // -1 = reverse, +1 = forward

// Public API
void setSorterEnabled(bool enabled) { g_sorterEnabled = enabled; }

void Wings::toggle() { state = !state; wingsPiston.set(state); }
void Wings::set(bool s) { state = s; wingsPiston.set(state);}

void BallLoader::toggles() {state = !state; loader.set(state); }
void BallLoader::sets(bool s){ state = s; loader.set(state); }




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

    if (g_sorterOverrideActive) return;

    Outtake.spin(fwd, speedPct, pct);
}

void reverseOutake(double speedPct) {
    g_outakeDir = -1;
    g_outakePct = std::fabs(speedPct);

    if (g_sorterOverrideActive) return;

    Outtake.spin(reverse, speedPct, pct);
}

void stopOutake() {
    g_outakeDir = 0;
    g_outakePct = 0.0;

    if (g_sorterOverrideActive) return;

    Outtake.stop(coast);
}

// -------------------- Descore arm --------------------
void moveArmRight(double speedPct) { DescoreMotor.spinFor(1000, msec); }
void moveArmLeft (double speedPct) { DescoreMotor.spin(reverse, speedPct, pct); }
void stopArm()                     { DescoreMotor.stop(hold); }

// -------------------- Auto helpers --------------------
void runIntakeAuto(double speedPct) { setSorterEnabled(true);  runIntake(speedPct); }
void reverseIntakeAuto(double speedPct) { setSorterEnabled(false); reverseIntake(speedPct); }
void stopIntakeAuto() { stopIntake(); setSorterEnabled(false); }

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

static void restoreOuttakeFromRequested() {
    const int dir = g_outakeDir;
    const double pctReq = g_outakePct;

    if (dir == 0 || pctReq <= 0.0) {
        Outtake.stop(coast);
    } else if (dir > 0) {
        Outtake.spin(fwd, pctReq, pct);
    } else {
        Outtake.spin(reverse, pctReq, pct);
    }
}

// CHANGED: helper to keep MainIntake forward during reject
static void keepMainIntakeForwardDuringReject() {
    const double pctReq = g_intakePct;
    if (pctReq > 0.0 && g_intakeDir != 0) {
        // Always forward during reject (per your request)
        MainIntake.spin(fwd, pctReq, pct);
    } else {
        MainIntake.stop(coast);
    }
}

// CHANGED: don’t stop MainIntake here anymore
static void beginRejectOverride() {
    g_sorterOverrideActive = true;

    // Keep main intake pulling forward
    keepMainIntakeForwardDuringReject();

    // Only stop these so the reject logic fully owns them
    ColorIntake.stop(coast);
    Outtake.stop(coast);
}

static void endSorterOverride() {
    ColorIntake.stop(coast);
    Outtake.stop(coast);

    g_sorterOverrideActive = false;

    restoreIntakeFromRequested();
    restoreOuttakeFromRequested();
}

static void runRejectMotors() {
    ColorIntake.spin(reverse, kRejectColorIntakePct, pct);
    
    //OuttakeA.spin(reverse, kRejectColorIntakePct, pct);

    // if (kRejectOuttakeDir < 0) Outtake.spin(reverse, kRejectOuttakePct, pct);
    // else                       Outtake.spin(fwd,     kRejectOuttakePct, pct);
}

// Sorter task
int intakeTaskFn() {
    ballSensor.setLightPower(100, percent);

    int cooldownMs = 0;
    int redCount  = 0;
    int blueCount = 0;

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

        const BallColor decided = (redCount >= kConfirmSamples) ? BallColor::RED : BallColor::BLUE;

        if (!isOpponentBall(decided)) {
            redCount = 0;
            blueCount = 0;
            cooldownMs = 40;
            wait(kLoopMs, msec);
            continue;
        }

        beginRejectOverride();
        runRejectMotors();

        // CHANGED: during the reject window, keep MainIntake pulling forward
        int tMs = 0;
        while (tMs < kRejectMs) {
            keepMainIntakeForwardDuringReject();
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
