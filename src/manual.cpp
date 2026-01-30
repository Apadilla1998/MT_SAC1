// manual.cpp
#include "manual.h"
#include "robot_config.h"
#include "subsystems.h"
#include "odom.h"
#include "utils.h"
#include <cmath>
#include <algorithm>

using namespace vex;

// ============================
// Tunables / Constants
// ============================

static constexpr int    kOuttakeNormalPct   = 40;
static constexpr int    kOuttakeWingsUpPct  = 100;

static constexpr double kOuttakeAccelPctPerS = 600.0;
static constexpr double kOuttakeDecelPctPerS = 900.0;

static constexpr double kFwdCurve  = 0.20;
static constexpr double kTurnCurve = 0.250;

static constexpr double kLeftBias  = 1.00;
static constexpr double kRightBias = 1.00;

static constexpr double kDriveAccelPctPerS = 350.0;
static constexpr double kDriveDecelPctPerS = 290.0;
static constexpr double kDt               = 0.02; // 20ms loop

static constexpr double kDriveScaleFast = 1.00;
static constexpr double kDriveScaleSlow = 0.40;

static constexpr double kTurnScaleFast  = 0.55;
static constexpr double kTurnScaleSlow  = 0.35;
static constexpr double kTurnMaxPct     = 80.0;

static constexpr double kTurnAccelPctPerS = 900.0;
static constexpr double kTurnDecelPctPerS = 1100.0;

static constexpr int    kDeadbandPct = 0;

// Drive lock behavior
static constexpr int    kDriveUnlockJoyThreshPct = 8; // joystick movement unlocks hold

// ============================
// State
// ============================

static double g_fwdCmd = 0.0;
static double g_trnCmd = 0.0;

// outtake ramp state
static double g_outtakeCmd = 0.0;

static bool g_prevR1 = false, g_prevUp = false, g_prevX = false, g_prevY = false;
static bool g_prevB  = false;

static bool g_isFast   = true;
static bool g_showOdom = false;

static int  g_screenTimerMs = 0;
static int  g_ballTimerMs   = 0;

static bool g_driveStopped = true;

// drive lock
static bool g_driveLocked = false;
static bool g_forceScreenUpdate = false;

// ============================
// Small helpers
// ============================

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

// joystick controls
static inline double readForwardAxisPct() {
    const double a3 = Controller1.Axis3.position(pct);
    const double a2 = Controller1.Axis2.position(pct);
    return (std::fabs(a3) >= std::fabs(a2)) ? a3 : a2;
}

static inline double readTurnAxisPct() {
    const double a1 = Controller1.Axis1.position(pct);
    const double a4 = Controller1.Axis4.position(pct);
    return (std::fabs(a1) >= std::fabs(a4)) ? a1 : a4;
}

// ============================
// Ball line (driver screen)
// ============================

static double filteredHueManual() {
    static double buf[5] = {0,0,0,0,0};
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

    if (!ballSensor.isNearObject()) {
        Controller1.Screen.print("BALL: NONE");
        return;
    }

    const double hue = filteredHueManual();

    const bool isRed  = (hue < 20 || hue > 340);
    const bool isBlue = (hue > 200 && hue < 250);

    const char* c = isRed ? "RED" : (isBlue ? "BLUE" : "UNK");

    const bool isOpponent =
        (myAlliance == RED  && isBlue) ||
        (myAlliance == BLUE && isRed);

    Controller1.Screen.print("BALL:%s H:%5.1f %s", c, hue, isOpponent ? "OPP" : "ALLY");
}

static void updateControllerScreen() {
    Controller1.Screen.clearLine(1);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.clearLine(3);

    if (!g_showOdom) {
        Controller1.Screen.setCursor(1, 1);
        if (g_driveLocked) {
            Controller1.Screen.print("SPEED:%s LOCK", g_isFast ? "FAST" : "SLOW");
        } else {
            Controller1.Screen.print("SPEED: %s", g_isFast ? "FAST" : "SLOW");
        }

        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("WINGS: %s", wings.isExtended() ? "UP" : "DOWN");

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
    g_screenTimerMs += 20;
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

// ============================
// Control handlers
// ============================

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
    // do not force-stop here; let handleDrive take over smoothly
    g_driveStopped = true;
}

static void handleDrive() {
    // If locked, unlock automatically on joystick movement
    if (g_driveLocked) {
        const double rawFwd  = readForwardAxisPct();
        const double rawTurn = readTurnAxisPct();

        if (std::fabs(rawFwd) > kDriveUnlockJoyThreshPct ||
            std::fabs(rawTurn) > kDriveUnlockJoyThreshPct) {
            g_driveLocked = false;
            disengageDriveHold();
            Controller1.rumble(".");
            g_forceScreenUpdate = true;
            // fall through and drive normally this cycle
        } else {
            engageDriveHold();
            return;
        }
    }

    double fwdIn = applyCurvePct(readForwardAxisPct(), kFwdCurve);
    double trnIn = applyCurvePct(readTurnAxisPct(),   kTurnCurve);

    if (std::fabs(fwdIn) < kDeadbandPct) fwdIn = 0.0;
    if (std::fabs(trnIn) < kDeadbandPct) trnIn = 0.0;

    const bool neutralInput = (fwdIn == 0.0 && trnIn == 0.0);

    const double driveScale = g_isFast ? kDriveScaleFast : kDriveScaleSlow;
    const double turnScale  = g_isFast ? kTurnScaleFast  : kTurnScaleSlow;

    double fwdReq = fwdIn * driveScale;
    double trnReq = trnIn * turnScale;
    trnReq = clampD(trnReq, -kTurnMaxPct, kTurnMaxPct);

    if (neutralInput) {
        fwdReq = 0.0;
        trnReq = 0.0;
    }

    g_fwdCmd = slewTo(fwdReq, g_fwdCmd, kDriveAccelPctPerS, kDriveDecelPctPerS);
    g_trnCmd = slewTo(trnReq, g_trnCmd, kTurnAccelPctPerS,  kTurnDecelPctPerS);

    const double lOut = clampPct((g_fwdCmd + g_trnCmd) * kLeftBias);
    const double rOut = clampPct((g_fwdCmd - g_trnCmd) * kRightBias);

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

static void handleToggles(bool& needsUpdate) {
    const bool r1 = Controller1.ButtonR1.pressing();
    if (r1 && !g_prevR1) {
        g_isFast = !g_isFast;
        Controller1.rumble(".");
        needsUpdate = true;
    }
    g_prevR1 = r1;

    const bool up = Controller1.ButtonUp.pressing();
    if (up && !g_prevUp) {
        wings.toggle();
        needsUpdate = true;
    }
    g_prevUp = up;

    const bool x = Controller1.ButtonX.pressing();
    if (x && !g_prevX) {
        resetOdometry();
        Controller1.rumble("-");
        needsUpdate = true;
    }
    g_prevX = x;

    const bool y = Controller1.ButtonY.pressing();
    if (y && !g_prevY) {
        g_showOdom = !g_showOdom;
        needsUpdate = true;
    }
    g_prevY = y;

    // B toggle: lock wheels (hold). Moving joystick unlocks automatically.
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
}

// Fixed: one unified handler owns BOTH intake + outtake, so they don't fight each other
// Controls (priority): R2 reverse > L2 outtake > L1 intake > none
static void handleIntakeOuttake() {
    const int outtakeBase = wings.isExtended() ? kOuttakeWingsUpPct : kOuttakeNormalPct;

    const bool intakeFwd  = Controller1.ButtonL1.pressing();
    const bool outtakeFwd = Controller1.ButtonL2.pressing();
    const bool reverseAll = Controller1.ButtonR2.pressing();

    int intakeDir = 0;
    int intakePct = 0;
    double outtakeTarget = 0.0;

    if (reverseAll) {
        intakeDir = -1;
        intakePct = 100;
        outtakeTarget = -outtakeBase;
        setSorterEnabled(false);
    } else if (outtakeFwd) {
        intakeDir = +1;
        intakePct = 100;
        outtakeTarget = +outtakeBase;
        setSorterEnabled(true);
    } else if (intakeFwd) {
        intakeDir = +1;
        intakePct = 100;
        outtakeTarget = 0.0; // keep outtake off while just intaking
        setSorterEnabled(true);
    } else {
        intakeDir = 0;
        intakePct = 0;
        outtakeTarget = 0.0;
        setSorterEnabled(false);
    }

    // Intake motor(s)
    if (intakeDir > 0) {
        runIntake(intakePct);
    } else if (intakeDir < 0) {
        reverseIntake(intakePct);
    } else {
        stopIntake();
    }

    // Outtake motor(s) with ramp
    const bool increasingMag = (std::fabs(outtakeTarget) > std::fabs(g_outtakeCmd));
    const double rate = increasingMag ? kOuttakeAccelPctPerS : kOuttakeDecelPctPerS;
    const double step = rate * kDt;

    double delta = outtakeTarget - g_outtakeCmd;
    delta = clampD(delta, -step, step);
    g_outtakeCmd += delta;

    if (std::fabs(g_outtakeCmd) < 1.0) {
        stopOutake();
        g_outtakeCmd = 0.0;
    } else if (g_outtakeCmd > 0.0) {
        runOutake((int)std::fabs(g_outtakeCmd));
    } else {
        reverseOutake((int)std::fabs(g_outtakeCmd));
    }
}

static void handleDescore() {
    if (Controller1.ButtonLeft.pressing()) {
        moveArmLeft(25);
    } else if (Controller1.ButtonRight.pressing()) {
        moveArmRight(25);
    } else {
        stopArm();
    }
}

// ============================
// usercontrol()
// ============================

void usercontrol() {
    ballSensor.setLightPower(100, percent);

    // default: coast (B toggles hold)
    LeftMotorGroup.setStopping(coast);
    RightMotorGroup.setStopping(coast);

    updateControllerScreen();

    while (true) {
        bool needsUpdate = false;

        handleDrive();

        if (g_forceScreenUpdate) {
            needsUpdate = true;
            g_forceScreenUpdate = false;
        }

        handleDescore();
        handleToggles(needsUpdate);

        // unified intake+outtake so they don't conflict
        handleIntakeOuttake();

        screenTick(needsUpdate);

        wait(20, msec);
    }
}
