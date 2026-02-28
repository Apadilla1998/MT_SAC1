#include "manual.h"
#include "robot_config.h"
#include "subsystems.h"
#include "odom.h"
#include "utils.h"
#include <cmath>
#include <algorithm>

using namespace vex;

// Tunables / Constants
static constexpr int    kOuttakeNormalPct   = 100;
static constexpr int    kOuttakeWingsUpPct  = 100;

static constexpr double kOuttakeAccelPctPerS = 600.0;
static constexpr double kOuttakeDecelPctPerS = 900.0;

static constexpr double kFwdCurve  = 0.20;
static constexpr double kTurnCurve = 0.250;

static constexpr double kLeftBias  = 1.00;
static constexpr double kRightBias = 1.00;

static constexpr double kDriveAccelPctPerS = 1500.0;
static constexpr double kDriveDecelPctPerS = 950.0;
static constexpr double kDt               = 0.025;

static constexpr double kDriveScaleFast = 0.75; //was 60
static constexpr double kTurnScaleFast  = 0.45;
static constexpr double kTurnMaxPct     = 80.0;
static constexpr double kTurnBoostAtFullFwd = 0.25;

static constexpr int    kDeadbandPct = 0; 
static constexpr int    kDriveUnlockJoyThreshPct = 8;

static constexpr int kIntakePct            = 75;
static constexpr int kScoreIntakePct       = 75;
static constexpr int kReversePct           = 25;
static constexpr int boost_NUMER           = 100;

// R1 = 100% drivetrain override
static constexpr double kDriveScaleR1 = 1.00;
static constexpr double kTurnScaleR1  = 1.00;

// State
static double g_fwdCmd = 0.0;
static double g_trnCmd = 0.0;
static double g_outtakeCmd = 0.0;

static bool g_prevUp = false, g_prevX = false, g_prevY = false;
static bool g_prevB  = false, g_prevL1 = false;
static bool g_prevDown = false;

// NEW: only for Left/Right press edge rumble
static bool g_prevLeft = false;
static bool g_prevRight = false;

static bool g_isFast   = true;
static bool g_showOdom = false;

static int  g_screenTimerMs = 0;
static int  g_ballTimerMs   = 0;

static bool g_driveStopped = true;
static bool g_driveLocked = false;
static bool g_forceScreenUpdate = false;

// R1 continuous rumble timer
static int g_r1RumbleTimerMs = 0;

// Color sorting enable/disable (X toggle)
static bool g_sorterEnabledUser = true;

// Intake toggle mode
enum class IntakeMode { OFF, INTAKE, SCORE, REVERSE };
static IntakeMode g_intakeMode = IntakeMode::OFF;

// Helpers
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

// Driver screen
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

    // Always read hue (even if no object is near)
    const double hue = wrap360(filteredHueManual());
    const bool near  = ballSensor.isNearObject();

    if (!near) {
        // Still show hue even when NONE
        Controller1.Screen.print("BALL:NONE H:%5.1f", hue);
        return;
    }

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

// R1 continuous rumble
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

// Drive control
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

static void handleDrive() {
    if (g_driveLocked) {
        const double rawFwd  = readForwardAxisPct();
        const double rawTurn = readTurnAxisPct();

        if (std::fabs(rawFwd) > kDriveUnlockJoyThreshPct ||
            std::fabs(rawTurn) > kDriveUnlockJoyThreshPct) {
            g_driveLocked = false;
            disengageDriveHold();
            g_forceScreenUpdate = true; // no rumble here
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
    const bool boostR1 = Controller1.ButtonR1.pressing();

    const double driveScale = boostR1 ? kDriveScaleR1
                                      : (g_isFast ? kDriveScaleFast : kDriveScaleFast);

    const double turnScale  = boostR1 ? kTurnScaleR1
                                      : (g_isFast ? kTurnScaleFast  : kTurnScaleFast);

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

    double lRaw = (g_fwdCmd + g_trnCmd) * kLeftBias;
    double rRaw = (g_fwdCmd - g_trnCmd) * kRightBias;

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

static void handleToggles(bool& needsUpdate) {
    // Up: Long Goal Piston toggle (no rumble)
    const bool up = Controller1.ButtonUp.pressing();
    if (up && !g_prevUp) {
        wings.toggle();
        needsUpdate = true;
    }
    g_prevUp = up;

    // X: toggle color sorting enable/disable (WITH rumble)
    const bool x = Controller1.ButtonX.pressing();
    if (x && !g_prevX) {
        g_sorterEnabledUser = !g_sorterEnabledUser;
        setSorterEnabled(g_sorterEnabledUser);
        Controller1.rumble(g_sorterEnabledUser ? "." : "-");
        needsUpdate = true;
    }
    g_prevX = x;

    // Y: toggle odom screen (no rumble)
    const bool y = Controller1.ButtonY.pressing();
    if (y && !g_prevY) {
        g_showOdom = !g_showOdom;
        needsUpdate = true;
    }
    g_prevY = y;

    // B: drive lock toggle (WITH rumble)
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

    // Down: Tongue piston toggle (no rumble)
    const bool down = Controller1.ButtonDown.pressing();
    if (down && !g_prevDown) {
        ballLoader.toggles();
        needsUpdate = true;
    }
    g_prevDown = down;
}

static void handleIntakeOuttake() {
    const bool boost = Controller1.ButtonR1.pressing();

    const bool l1 = Controller1.ButtonL1.pressing(); // toggle intake
    const bool l2 = Controller1.ButtonL2.pressing(); // hold outtake
    const bool r2 = Controller1.ButtonR2.pressing(); // hold reverse
    const bool a  = Controller1.ButtonA.pressing();  // NEW: momentary override

    // L1 toggle logic (only INTAKE <-> OFF)
    if (l1 && !g_prevL1) {
        g_intakeMode = (g_intakeMode == IntakeMode::INTAKE) ? IntakeMode::OFF : IntakeMode::INTAKE;
    }
    g_prevL1 = l1;

    const int outtakeBase = wings.isExtended() ? kOuttakeWingsUpPct : kOuttakeNormalPct;
    const int outtakeMag  = boost ? 100 : outtakeBase;

    // Momentary overrides
    IntakeMode mode = g_intakeMode;
    if (r2)      mode = IntakeMode::REVERSE;
    else if (l2) mode = IntakeMode::SCORE;

    int intakeDir = 0;
    int intakePct = 0;
    double outtakeTarget = 0.0;

    switch (mode) {
        case IntakeMode::OFF:
            intakeDir     = 0;
            intakePct     = 0;
            outtakeTarget = 0.0;
            break;

        case IntakeMode::INTAKE: // L1 toggle
            intakeDir     = +1;
            intakePct     = boost ? boost_NUMER : kIntakePct;
            outtakeTarget = 0.0; // keep main outtake off
            break;

        case IntakeMode::SCORE:  // L2 hold
            intakeDir     = +1;
            intakePct     = boost ? boost_NUMER : kScoreIntakePct;
            outtakeTarget = +outtakeMag;
            break;

        case IntakeMode::REVERSE: // R2 hold
            intakeDir     = -1;
            intakePct     = boost ? boost_NUMER : kReversePct;
            outtakeTarget = -outtakeMag;
            break;
    }

    // While holding A, turn sorting OFF (since we're manually reversing rollers)
    const bool sorterShouldRun =
        g_sorterEnabledUser &&
        !a &&
        (mode != IntakeMode::OFF) &&
        (mode != IntakeMode::REVERSE);

    setSorterEnabled(sorterShouldRun);

  
    // Intake motor control change y to make it so that mainintake is the only thing that spins

    if (a) {
        // If we're currently intaking/scoring, reuse that speed; otherwise default to intake speed.
        const int aPct = (intakePct != 0) ? intakePct : (boost ? boost_NUMER : kIntakePct);

        // mainIntake stays forward, colorIntake reverses
        MainIntake.spin(forward, aPct, pct);
        ColorIntake.spin(reverse, aPct, pct);
    } else {
        // Normal behavior
        if (intakeDir > 0)      { runIntake(intakePct); }
        else if (intakeDir < 0) reverseIntake(intakePct);
        else                    stopIntake();
    }

    if (!a && mode == IntakeMode::INTAKE) {
        OuttakeB.spin(reverse, 25, pct);
        OuttakeA.spin(forward, 50, percent);
    } else {
        OuttakeB.stop(coast);
        OuttakeA.stop(coast);
    }

  
    // Main outtake smoothing (unchanged)
if (mode != IntakeMode::INTAKE) {
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
} else {
    // L1 toggled intake: you control B/C manually
    g_outtakeCmd = 0.0;
}

  
    // OuttakeA control

  
    // OuttakeB / OuttakeC when L1 is toggled (INTAKE mode)

}

static void handleDescore() {
    const bool boost = Controller1.ButtonR1.pressing();
    const int  armPct = boost ? 100 : 25;

    const bool left  = Controller1.ButtonLeft.pressing();
    const bool right = Controller1.ButtonRight.pressing();

    // Rumble ONLY on press edges for Left/Right
    if (left && !g_prevLeft)   Controller1.rumble(".");
    if (right && !g_prevRight) Controller1.rumble(".");

    g_prevLeft  = left;
    g_prevRight = right;

    if (left)       moveArmLeft(armPct);
    else if (right) moveArmRight(armPct);
    else            stopArm();
}

void usercontrol() {
    ballSensor.setLightPower(100, percent);

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
        handleIntakeOuttake();

        handleR1ContinuousRumble();
        screenTick(needsUpdate);

        wait(20, msec);
    }
}
