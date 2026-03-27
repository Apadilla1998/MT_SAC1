#include "manual.h"
#include "robot_config.h"
#include "subsystems.h"
#include "odom.h"
#include "utils.h"
#include <cmath>
#include "descore.h"
#include <algorithm>

using namespace vex;

// ============================================================
//  Tunables / Constants
// ============================================================
static constexpr double kFwdCurve  = 0.20;
static constexpr double kTurnCurve = 0.250;

static constexpr double kLeftBias  = 1.00;
static constexpr double kRightBias = 1.00;

static constexpr double kDriveAccelPctPerS = 2000.0;
static constexpr double kDriveDecelPctPerS = 1500.0;
static constexpr double kDt                = 0.025;

static constexpr double kDriveScaleFast      = 1.0;
static constexpr double kTurnScaleFast       = 0.25;
static constexpr double kTurnMaxPct          = 80.0;
static constexpr double kTurnBoostAtFullFwd  = 0.25;

static constexpr int    kDeadbandPct = 0;
static constexpr int    kDriveUnlockJoyThreshPct = 8;

static constexpr int kIntakePct  = 100;
static constexpr int kReversePct = 100;   // R2 reverse is always full power

// R1 = 100% drivetrain override
static constexpr double kDriveScaleR1 = 1.00;
static constexpr double kTurnScaleR1  = 1.00;

// ============================================================
//  State
// ============================================================
static double g_fwdCmd = 0.0;
static double g_trnCmd = 0.0;

static bool g_prevUp   = false, g_prevX  = false, g_prevY = false;
static bool g_prevB    = false, g_prevL1 = false;
static bool g_prevDown = false;
static bool g_prevLeft = false, g_prevRight = false;

static bool g_isFast   = true;
static bool g_showOdom = false;

static int  g_screenTimerMs = 0;
static int  g_ballTimerMs   = 0;

static bool g_driveStopped = true;
static bool g_driveLocked  = false;
static bool g_forceScreenUpdate = false;

static int  g_r1RumbleTimerMs   = 0;
static bool g_sorterEnabledUser = false;

// Simplified intake mode — no outtake, no SCORE
enum class IntakeMode { OFF, INTAKE, REVERSE };
static IntakeMode g_intakeMode = IntakeMode::OFF;

// ============================================================
//  Helpers
// ============================================================
static inline double applyCurvePct(double inputPct, double curve) {
    const double v = inputPct / 100.0;
    return ((curve * std::pow(v, 3)) + ((1.0 - curve) * v)) * 100.0;
}

static inline double wrap360(double d) {
    while (d >= 360.0) d -= 360.0;
    while (d < 0.0)    d += 360.0;
    return d;
}

static inline double slewTo(double target, double current, double accel, double decel) {
    double delta = target - current;
    const bool increasingMag = (std::fabs(target) > std::fabs(current));
    const double maxStep = (increasingMag ? accel : decel) * kDt;
    delta = clampD(delta, -maxStep, maxStep);
    return current + delta;
}

static inline void normalizeArcade(double& leftPct, double& rightPct) {
    const double maxMag = std::max(std::fabs(leftPct), std::fabs(rightPct));
    if (maxMag > 100.0) {
        const double s = 100.0 / maxMag;
        leftPct  *= s;
        rightPct *= s;
    }
}

static inline double readForwardAxisPct() {
    return Controller1.Axis3.position(pct);
}

static inline double readTurnAxisPct() {
    return Controller1.Axis1.position(pct);
}

// ============================================================
//  Driver screen
// ============================================================
static double filteredHueManual() {
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

static void updateBallLine() {
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3, 1);

    const double hue = wrap360(filteredHueManual());
    const bool near  = ballSensor.isNearObject();

    if (!near) {
        Controller1.Screen.print("BALL:NONE H:%5.1f", hue);
        return;
    }

    const bool isRed  = (hue < 20 || hue > 340);
    const bool isBlue = (hue > 200 && hue < 250);
    const char* c = isRed ? "RED" : (isBlue ? "BLUE" : "UNK");

    const bool isOpponent =
        (myAlliance == Alliance::RED  && isBlue) ||
        (myAlliance == Alliance::BLUE && isRed);

    Controller1.Screen.print("BALL:%s H:%5.1f %s", c, hue, isOpponent ? "OPP" : "ALLY");
}

static void updateControllerScreen() {
    Controller1.Screen.clearLine(1);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.clearLine(3);

    if (!g_showOdom) {
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("LOCK:%s T:%s",
            g_driveLocked ? "ON " : "OFF",
            ballLoader.isExtended() ? "UP" : "DN");

        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("W:%s SORT:%s",
            wings.isExtended() ? "UP" : "DN",
            g_sorterEnabledUser ? "ON" : "OFF");

        updateBallLine();
    } else {
        const double x_cm  = robotPose.x * 100.0;
        const double y_cm  = robotPose.y * 100.0;
        const double thDeg = wrap360(radToDeg(robotPose.theta));

        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("X:%6.1f cm", x_cm);

        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("Y:%6.1f cm", y_cm);

        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print("T:%6.1f deg", thDeg);
    }
}

static void screenTick(bool needsUpdate) {
    g_screenTimerMs += 10;
    const int screenPeriodMs = g_showOdom ? 200 : 4000;

    if (needsUpdate || g_screenTimerMs >= screenPeriodMs) {
        updateControllerScreen();
        g_screenTimerMs = 0;
    }

    g_ballTimerMs += 20;
    if (!g_showOdom && g_ballTimerMs >= 2000) {
        updateBallLine();
        g_ballTimerMs = 0;
    }
}

static void handleR1ContinuousRumble() {
    const bool r1 = Controller1.ButtonR1.pressing();

    if (r1) {
        g_r1RumbleTimerMs += 20;
        if (g_r1RumbleTimerMs >= 300) {
            Controller1.rumble("-");
            g_r1RumbleTimerMs = 0;
        }
    } else {
        g_r1RumbleTimerMs = 0;
    }
}

static void engageDriveHold() {
    LeftMotorGroup.setStopping(hold);
    RightMotorGroup.setStopping(hold);
    LeftMotorGroup.stop(hold);
    RightMotorGroup.stop(hold);
    g_driveStopped = true;
    g_fwdCmd = 0.0;
    g_trnCmd = 0.0;
}

static void disengageDriveHold() {
    LeftMotorGroup.setStopping(coast);
    RightMotorGroup.setStopping(coast);
    g_driveStopped = true;
}

// ============================================================
//  Drive — turn direction inverted vs. previous build
// ============================================================
static void handleDrive() {
    if (g_driveLocked) {
        const double rawFwd  = readForwardAxisPct();
        const double rawTurn = readTurnAxisPct();

        if (std::fabs(rawFwd)  > kDriveUnlockJoyThreshPct ||
            std::fabs(rawTurn) > kDriveUnlockJoyThreshPct) {
            g_driveLocked = false;
            disengageDriveHold();
            g_forceScreenUpdate = true;
        } else {
            engageDriveHold();
            return;
        }
    }

    double fwdIn = applyCurvePct(readForwardAxisPct(), kFwdCurve);
    double trnIn = applyCurvePct(readTurnAxisPct(),    kTurnCurve);

    if (std::fabs(fwdIn) < kDeadbandPct) fwdIn = 0.0;
    if (std::fabs(trnIn) < kDeadbandPct) trnIn = 0.0;

    const bool neutralInput = (fwdIn == 0.0 && trnIn == 0.0);
    const bool boostR1 = Controller1.ButtonR1.pressing();

    const double driveScale = boostR1 ? kDriveScaleR1 : kDriveScaleFast;
    const double turnScale  = boostR1 ? kTurnScaleR1  : kTurnScaleFast;

    double fwdReq = fwdIn * driveScale;
    double trnReq = trnIn * turnScale;

    const double turnMax = boostR1 ? 100.0 : kTurnMaxPct;
    trnReq = clampD(trnReq, -turnMax, turnMax);

    if (neutralInput) {
        fwdReq = 0.0;
        trnReq = 0.0;
    }

    g_fwdCmd = slewTo(fwdReq, g_fwdCmd, kDriveAccelPctPerS, kDriveDecelPctPerS);

    const double fwdMag = std::fabs(g_fwdCmd);
    const double boost  = 1.0 + kTurnBoostAtFullFwd * (fwdMag / 100.0);
    g_trnCmd = clampD(trnReq * boost, -turnMax, turnMax);

    // Turn direction inverted: subtract trnCmd on left, add on right
    double lRaw = (g_fwdCmd - g_trnCmd) * kLeftBias;
    double rRaw = (g_fwdCmd + g_trnCmd) * kRightBias;

    normalizeArcade(lRaw, rRaw);

    const double lOut = clampPct(lRaw);
    const double rOut = clampPct(rRaw);

    if (neutralInput) {
        if (std::fabs(g_fwdCmd) < 0.8 && std::fabs(g_trnCmd) < 0.8) {
            if (!g_driveStopped) {
                LeftMotorGroup.stop();
                RightMotorGroup.stop();
                g_driveStopped = true;
            }
            return;
        }
    }

    g_driveStopped = false;
    LeftMotorGroup.spin(forward, lOut, pct);
    RightMotorGroup.spin(forward, rOut, pct);
}

// ============================================================
//  Toggles (Up/X/Y/B/Down)
// ============================================================
static void handleToggles(bool& needsUpdate) {
    const bool up = Controller1.ButtonUp.pressing();
    if (up && !g_prevUp) {
        wings.toggle();
        needsUpdate = true;
    }
    g_prevUp = up;

    const bool x = Controller1.ButtonX.pressing();
    if (x && !g_prevX) {
        g_sorterEnabledUser = !g_sorterEnabledUser;
        setSorterEnabled(g_sorterEnabledUser);
        Controller1.rumble(g_sorterEnabledUser ? "." : "-");
        needsUpdate = true;
    }
    g_prevX = x;

    const bool y = Controller1.ButtonY.pressing();
    if (y && !g_prevY) {
        g_showOdom = !g_showOdom;
        needsUpdate = true;
    }
    g_prevY = y;

    const bool b = Controller1.ButtonB.pressing();
    if (b && !g_prevB) {
        g_driveLocked = !g_driveLocked;
        if (g_driveLocked) {
            engageDriveHold();
            Controller1.rumble("-");
        } else {
            disengageDriveHold();
            Controller1.rumble(".");
        }
        needsUpdate = true;
    }
    g_prevB = b;

    const bool down = Controller1.ButtonDown.pressing();
    if (down && !g_prevDown) {
        ballLoader.toggles();
        needsUpdate = true;
    }
    g_prevDown = down;
}

// ============================================================
//  Intake
//  L1 toggle → run intake forward at kIntakePct
//  R2 held   → reverse intake at kReversePct (100%)
// ============================================================
static void handleIntake() {
    const bool l1 = Controller1.ButtonL1.pressing();
    const bool r2 = Controller1.ButtonR2.pressing();

    if (l1 && !g_prevL1) {
        g_intakeMode = (g_intakeMode == IntakeMode::INTAKE)
                     ? IntakeMode::OFF
                     : IntakeMode::INTAKE;
    }
    g_prevL1 = l1;

    // R2 overrides the toggle while held
    IntakeMode mode = g_intakeMode;
    if (r2) mode = IntakeMode::REVERSE;

    // Sorter only runs when intake is forward and user has enabled it
    const bool sorterShouldRun =
        g_sorterEnabledUser && (mode == IntakeMode::INTAKE);
    setSorterEnabled(sorterShouldRun);

    switch (mode) {
        case IntakeMode::OFF:
            stopIntake();
            break;

        case IntakeMode::INTAKE:
            runIntake(kIntakePct);
            break;

        case IntakeMode::REVERSE:
            reverseIntake(kReversePct);
            break;
    }
}

// ============================================================
//  Lever Arm — L2 deploys, release returns to origin
//  Pot-based proportional control lives in descore.cpp
// ============================================================
static void handleLeverArm() {
    const bool l2 = Controller1.ButtonL2.pressing();
    updateLeverArm(l2);
}

// ============================================================
//  Descore Arm — Left / Right d-pad, R1 for boost
// ============================================================
static void handleDescore() {
    const bool boost = Controller1.ButtonR1.pressing();
    const int  armPct = boost ? 100 : 25;

    const bool left  = Controller1.ButtonLeft.pressing();
    const bool right = Controller1.ButtonRight.pressing();

    if (left  && !g_prevLeft)  Controller1.rumble(".");
    if (right && !g_prevRight) Controller1.rumble(".");

    g_prevLeft  = left;
    g_prevRight = right;

    if (left)        moveArmLeft(armPct);
    else if (right)  moveArmRight(armPct);
    else             stopArm();
}

// ============================================================
//  Main driver control loop
// ============================================================
void usercontrol() {
    ballSensor.setLightPower(100, percent);

    LeftMotorGroup.setStopping(coast);
    RightMotorGroup.setStopping(coast);

    g_sorterEnabledUser = false;
    setSorterEnabled(false);

    initLeverArm();

    updateControllerScreen();

    while (true) {
        bool needsUpdate = false;

        handleDrive();

        if (g_forceScreenUpdate) {
            needsUpdate = true;
            g_forceScreenUpdate = false;
        }

        handleDescore();
        handleLeverArm();
        handleToggles(needsUpdate);
        handleIntake();

        handleR1ContinuousRumble();
        screenTick(needsUpdate);

        wait(20, msec);
    }
}