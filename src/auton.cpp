#include "autons.h"
#include "motion.h"
#include "subsystems.h"
#include "robot_config.h"
#include "drive.h"
#include "utils.h"
#include "vex.h"
#include "manual.h"
#include "vision_utils.h"
#include "Lever.h"
#include "Descore.h"
#include <cmath>

using namespace vex;

AutonRoutine selectedAuton = AutonRoutine::ROUTINEONE;

// ============================================================
//  Local helpers
// ============================================================

static void driveDistanceByMotors(double distIn, double speedPct, int timeoutMs) {
    const double wheelDiamIn = 3.25;
    const double wheelCircIn = M_PI * wheelDiamIn;
    const double wheelDeg    = (distIn / wheelCircIn) * 360.0;

    LeftMotorGroup.resetPosition();
    RightMotorGroup.resetPosition();

    timer t; t.reset();
    const int dir = (distIn >= 0) ? 1 : -1;

    while (t.time(msec) < timeoutMs) {
        double l   = std::fabs(LeftMotorGroup.position(deg));
        double r   = std::fabs(RightMotorGroup.position(deg));
        double avg = 0.5 * (l + r);

        if (avg >= std::fabs(wheelDeg)) break;

        tankDrive(dir * speedPct, dir * speedPct);
        wait(10, msec);
    }

    stopDrive(brake);
}

static void setWingsState(bool desiredOut, bool &wingsOut) {
    if (wingsOut != desiredOut) {
        wings.toggle();
        wingsOut = desiredOut;
        wait(120, msec);
    }
}

static void wingFlick(bool &wingsOut) {
    if (!wingsOut) {
        wings.toggle();
        wingsOut = true;
        wait(40, msec);
    }

    wings.toggle();
    wingsOut = false;
    wait(50, msec);

    wings.toggle();
    wingsOut = true;
    wait(80, msec);
}

static void scoreBurst(int cycles, int onMs = 250, int offMs = 60) {
    for (int i = 0; i < cycles; i++) {
        runIntake(100);
        wait(onMs, msec);

        stopIntake();
        wait(offMs, msec);
    }
}

static void loaderPeck(int cycles,
                       double fwdIn   = 2.0,
                       double backIn  = -3.0,
                       double fwdPct  = 22,
                       double backPct = 28,
                       int    pauseMs = 120) {
    for (int i = 0; i < cycles; i++) {
        driveDistanceByMotors(fwdIn,  fwdPct,  1000);
        wait(10, msec);
        driveDistanceByMotors(backIn, backPct, 1000);
        wait(pauseMs, msec);
    }
}

static void scoreTopGoal(MotionController &m, bool &wingsOut,
                         int burstCycles = 6, int finalFeedMs = 700) {
    setWingsState(true, wingsOut);

    m.drive(0.40, 1600, 40);

    visionAlignOnlyToCenterId(1, 1600, 0.16, 0.0, 6, 8, 6, 158.0, 0.0);

    m.drive(0.75, 1700, 50);
    m.addFix(-90);

    driveDistanceByMotors(8, 60, 900);

    reverseIntake(40);
    wait(150, msec);

    scoreBurst(burstCycles, 250, 60);

    runIntake(100);
    wait(finalFeedMs, msec);

    stopIntake();
    wait(10, msec);
}

// ============================================================
//  Autonomous routines
// ============================================================

static void HardCodedRightSkills() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    wait(10, msec);

    wings.toggle();
    m.driveHeadingCC(-0.812, 8000, 40, 0);
    wait(10, msec);
    m.turnTo(90, 3000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec);

    runIntake(100);
    wait(10, msec);

    driveDistanceByMotors(-10.5, 30, 6000);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    stopIntake();
    driveDistanceByMotors(6, 20, 5000);
    ballLoader.toggles();
    wait(10, msec);

    m.turnTo(45, 4500);
    driveDistanceByMotors(22, 20, 7000);
    wait(10, msec);

    m.turnTo(90, 45000);
    m.driveHeadingCC(1.83, 9000, 40, 90);

    m.turnTo(180, 5000);
    driveDistanceByMotors(18, 25, 6000);
    wait(10, msec);

    m.turnTo(-90, 6000);
    driveDistanceByMotors(9, 25, 6000);
    wait(10, msec);

    reverseIntake(20);
    wait(700, msec);

    stopIntake();
    wait(10, msec);

    runIntake(100);
    wait(900, msec);

    reverseIntake(40);
    wait(700, msec);

    runIntake(100);
    wait(3000, msec);

    stopIntake();
    wait(10, msec);

    // going toward loader
    driveDistanceByMotors(-6, 25, 6000);
    ballLoader.toggles();
    wait(10, msec);

    runIntake(100);
    driveDistanceByMotors(-20, 27, 8000);
    wait(10, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    stopIntake();

    driveDistanceByMotors(6, 30, 6000);
    ballLoader.toggles();
    wait(10, msec);

    m.driveHeadingCC(18, 6000, 30, -90);
    runIntake(100);
    wait(10, msec);
}

static void ShitSkills() {
    // Outtake removed — routine is intentionally empty
}

static void skills2() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(true);

    m.driveHeadingCC(-0.84, 2500, 50, 0);
    wait(10, msec);
    m.turnTo(90, 2000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec);

    runIntake(100);
    wait(10, msec);

    m.driveHeadingCC(-0.301, 2000, 50, 90);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(900, msec);

    stopIntake();
    wait(10, msec);

    m.drive(0.35, 2900, 50);
    ballLoader.toggles();
    runIntake(100);
    wait(1000, msec);

    stopIntake();
    setSorterEnabled(false);
    wait(10, msec);

    m.turnBy(136, 8000);
    wait(10, msec);

    m.drive(-1.18, 5500, 50);
    wait(10, msec);

    reverseIntake(40);
    wait(2, sec);

    runIntake(20);
    wait(900, msec);

    stopIntake();
    wait(10, msec);

    reverseIntake(40);
    wait(3, sec);

    stopIntake();
    wait(10, msec);
}

// For strong teams
static void HardCodedRightMessUp2() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(true);

    m.driveHeadingCC(-0.84, 2500, 50, 0);
    wait(10, msec);
    m.turnTo(90, 2000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec);

    runIntake(100);
    wait(10, msec);

    m.driveHeadingCC(-0.301, 2000, 50, 90);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(900, msec);

    stopIntake();
    wait(10, msec);

    m.drive(0.35, 2900, 50);
    ballLoader.toggles();
    runIntake(100);
    wait(1000, msec);

    stopIntake();
    setSorterEnabled(false);
    wait(10, msec);

    m.turnBy(136, 8000);
    wait(10, msec);

    m.drive(-1.18, 5500, 50);
    wait(10, msec);

    reverseIntake(40);
    wait(2, sec);

    runIntake(20);
    wait(900, msec);

    stopIntake();
    wait(10, msec);

    reverseIntake(40);
    wait(3, sec);

    stopIntake();
    wait(10, msec);
}

static void HardCodedRightMessUp() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(true);

    driveDistanceByMotors(-30, 24, 7000);
    wait(10, msec);
    m.turnTo(90, 3000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec);

    runIntake(100);
    wait(10, msec);

    driveDistanceByMotors(-8, 20, 7000);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    stopIntake();
    wait(10, msec);

    driveDistanceByMotors(13, 15, 9000);
    ballLoader.toggles();
    wait(10, msec);

    m.turnBy(139, 8000);
    wait(10, msec);

    runIntake(10);
    wait(500, msec);

    stopIntake();
    driveDistanceByMotors(-43.0, 30, 7000);
    wait(10, msec);

    setSorterEnabled(false);
    reverseIntake(40);
    wait(2, sec);

    runIntake(20);
    wait(900, msec);

    stopIntake();
    wait(10, msec);

    reverseIntake(40);
    wait(3, sec);

    stopIntake();
    wait(10, msec);

    // go to other side of middle goal
    driveDistanceByMotors(13, 24, 9000);
    m.turnTo(90, 7000);
    wait(10, msec);
    driveDistanceByMotors(26, 25, 9000);
    wait(10, msec);

    m.turnTo(43, 6000);
    driveDistanceByMotors(11, 17, 7000);
    wait(10, msec);

    RightMotorGroup.stop(hold);
    LeftMotorGroup.stop(hold);
}

// For weak teams
static void HardCodedRight() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(true);

    driveDistanceByMotors(-30, 24, 7000);
    wait(10, msec);
    m.turnTo(90, 3000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec);

    runIntake(100);
    wait(10, msec);

    driveDistanceByMotors(-8, 20, 7000);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    // going toward goal
    stopIntake();
    wait(10, msec);
    wings.toggle();
    driveDistanceByMotors(33, 20, 7000);
    wait(10, msec);

    runIntake(40);
    wait(4, sec);

    reverseIntake(20);
    wait(2, sec);

    runIntake(40);
    wait(4, sec);

    stopIntake();
    wait(10, msec);

    // going toward loader
    setSorterEnabled(true);
    driveDistanceByMotors(-3, 20, 7000);
    wings.toggle();
    wait(10, msec);

    runIntake(100);
    driveDistanceByMotors(-25.5, 20, 6000);
    wait(10, msec);

    driveDistanceByMotors(-10, 20, 7000);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(900, msec);

    stopIntake();
    wait(10, msec);

    // going toward middle goal
    driveDistanceByMotors(13, 15, 9000);
    ballLoader.toggles();
    wait(10, msec);

    m.turnBy(139, 8000);
    wait(10, msec);

    runIntake(10);
    wait(500, msec);

    stopIntake();
    driveDistanceByMotors(-43.0, 30, 7000);
    wait(10, msec);

    setSorterEnabled(false);
    reverseIntake(40);
    wait(2, sec);

    runIntake(20);
    wait(900, msec);

    stopIntake();
    wait(10, msec);

    reverseIntake(40);
    wait(3, sec);

    stopIntake();
    wait(10, msec);

    ballLoader.toggles();
    wait(10, msec);
}

static void Auton_SKILLS() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(false);

    // going towards the loader
    m.driveHeading(-0.84, 2500, 50, 0);
    wait(10, msec);
    m.turnTo(90, 2000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec);

    runIntake(100);
    wait(10, msec);

    m.driveHeadingCC(-0.301, 2000, 50, 90);
    wait(900, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000);
    wait(400, msec);

    stopIntake();
    wait(10, msec);

    // going toward opposite side
    driveDistanceByMotors(6, 30, 1000);
    reverseIntake(20);
    wait(250, msec);

    stopIntake();

    runIntake(20);
    m.turnTo(45, 1500);
    wait(10, msec);

    m.driveHeadingCC(0.55, 3000, 40, 45);
    wait(10, msec);

    m.turnTo(90, 1500);
    m.driveHeading(2, 3000, 40, 90);
    wait(10, msec);

    // turning towards goal and depositing
    reverseIntake(40);
    wait(200, msec);

    stopIntake();
    wings.toggle();
    wait(10, msec);

    m.turnTo(180, 2000);
    wings.toggle();
    wait(200, msec);
    wings.toggle();
    m.drive(0.42, 3000, 40);
    wait(10, msec);

    m.turnTo(-90, 2000);
    visionAlignOnlyToCenterId(1, 3000, 0.13, 0.0, 5, 6, 8, 158.0, 0.0);
    wait(800, msec);

    m.drive(0.50, 2000, 50);
    m.addFix(-90);
    driveDistanceByMotors(8, 100, 1500);

    reverseIntake(40);
    wait(250, msec);

    for (int i = 0; i < 7; i++) {
        runIntake(100);
        wait(500, msec);
        stopIntake();
        wait(10, msec);
    }

    wait(1, sec);

    wings.toggle();
    runIntake(100);
    wait(10, msec);

    m.driveHeading(-0.8, 2000, 30, -90);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000);
    wait(500, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000);
    wait(500, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000);
    wait(100, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000);
    wait(900, msec);
}

static void SimpleAutonRight() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(false);

    // going towards the loader
    m.driveHeading(-0.84, 2500, 50, 0);
    wait(10, msec);
    m.turnTo(90, 2000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec);

    runIntake(100);
    wait(10, msec);

    m.driveHeadingCC(-0.301, 2000, 50, 90);
    wait(900, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000);
    wait(400, msec);

    stopIntake();
    wait(10, msec);

    // going toward opposite side
    driveDistanceByMotors(6, 30, 1000);
    reverseIntake(20);
    wait(250, msec);

    stopIntake();

    runIntake(20);
    m.turnTo(45, 1500);
    wait(10, msec);

    m.driveHeadingCC(0.55, 3000, 40, 45);
    wait(10, msec);

    m.turnTo(90, 1500);
    m.driveHeading(2, 3000, 40, 90);
    wait(10, msec);

    // turning towards goal and depositing
    reverseIntake(40);
    wait(200, msec);

    stopIntake();
    wings.toggle();
    wait(10, msec);

    wings.toggle();
    wait(50, msec);
    wings.toggle();
    m.turnTo(180, 2000);
    m.drive(0.42, 3000, 40);
    wait(10, msec);

    m.turnTo(-90, 2000);
    visionAlignOnlyToCenterId(1, 3000, 0.13, 0.0, 5, 6, 8, 158.0, 0.0);
    wait(800, msec);

    m.drive(0.50, 2000, 50);
    m.addFix(-90);
    driveDistanceByMotors(8, 100, 1500);

    reverseIntake(40);
    wait(250, msec);

    for (int i = 0; i < 8; i++) {
        runIntake(100);
        wait(500, msec);
        stopIntake();
        wait(10, msec);
    }

    runIntake(100);
    wait(1000, msec);
    stopIntake();
    wait(10, msec);

    wait(1, sec);

    wings.toggle();
    runIntake(100);
    wait(10, msec);

    m.driveHeading(-0.8, 2000, 30, -90);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000);
    wait(500, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000);
    wait(500, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000);
    wait(100, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000);
    wait(400, msec);

    stopIntake();
    wait(10, msec);

    wings.toggle();
    m.drive(0.4, 2000, 30);
    visionAlignOnlyToCenterId(1, 3000, 0.13, 0.0, 5, 6, 8, 158.0, 0.0);
    wait(800, msec);

    m.drive(0.75, 2000, 30);
    m.addFix(-90);
    driveDistanceByMotors(8, 100, 1500);

    reverseIntake(40);
    wait(250, msec);

    for (int i = 0; i < 9; i++) {
        runIntake(100);
        wait(500, msec);
        stopIntake();
        wait(10, msec);
    }

    runIntake(100);
    wait(900, msec);
    stopIntake();
    wait(10, msec);

    runIntake(100);
    wait(1200, msec);
    stopIntake();
    wait(10, msec);

    driveDistanceByMotors(-5, 20, 2000);
    driveDistanceByMotors(6, 20, 2000);
}

static void SimpleAutonLeft() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(true);

    // going towards the loader
    m.driveHeadingCC(-0.84, 2500, 50, 0);
    wait(10, msec);
    m.turnTo(90, 2000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec);

    runIntake(100);
    wait(10, msec);

    m.driveHeadingCC(-0.301, 2000, 50, 90);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 2000);
    wait(900, msec);

    stopIntake();
    wait(10, msec);

    wings.toggle();
    m.drive(0.60, 5000, 50);
    wait(800, msec);

    m.drive(0.50, 3000, 50);
    m.addFix(90);
    runIntake(100);
    wait(900, msec);

    reverseIntake(25);
    wait(400, msec);

    runIntake(100);
    wait(2500, msec);

    stopIntake();
    wait(10, msec);
}

static void blueRight() {
    MotionController m;

    m.drive(0.82, 6000, 80);
    wait(10, msec);

    m.turnTo(90, 5000);
    wait(10, msec);

    m.drive(0.29, 5000, 80);
    wait(10, msec);
}

static void autoCorrectBlueRight() {
    MotionController m;
    m.setAutoCorrectEnabled(true);

    m.drive(0.75, 5500, 100);
    wait(10, msec);

    m.turnTo(90, 4000);
    wait(10, msec);

    runIntakeAuto(100);

    m.drive(0.29, 5500, 100);
    wait(10, msec);

    stopIntakeAuto();
    wait(10, msec);

    m.drive(-0.3, 4500, 100);
    m.turnBy(-90, 4000);
    wait(10, msec);

    reverseIntakeAuto(100);
    wait(500, msec);
    stopIntakeAuto();
    wait(10, msec);

    m.turnBy(90, 4000);
    wait(10, msec);

    m.turnBy(132, 7500);
    wait(10, msec);

    m.driveHeadingCC(1.18, 7000, 100, 225);
    wait(10, msec);

    reverseIntakeAuto(75);
    wait(3, sec);

    stopIntakeAuto();
    wait(10, msec);
}

static void autoCorrectRedLeft() {
    MotionController m;
    m.setAutoCorrectEnabled(true);

    while (true) {
        visionAlignOnlyToCenterIdTwo(1);
    }
}

static void autoCorrectBlueLeft() {
    runIntakeAuto(100);
    wait(2, sec);
    stopIntakeAuto();
    wait(10, msec);
}

static void autoCorrectRedRight() {}

static void blueLeft() { while (true) printArmAngleControllerUpdate(); }

static void Trash() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(true);

    m.driveHeadingCC(-30, 24, 7000, 0);
    wait(10, msec);
    m.turnTo(90, 3000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec);

    runIntake(100);
    wait(10, msec);

    driveDistanceByMotors(-13, 20, 7000);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    stopIntake();
    setSorterEnabled(false);
    wait(10, msec);

    driveDistanceByMotors(13, 15, 9000);
    ballLoader.toggles();
    wait(10, msec);

    m.turnBy(139, 8000);
    wait(10, msec);

    runIntake(10);
    wait(500, msec);

    stopIntake();
    m.driveHeadingCC(-43.0, 30, 7000, 139);
    wait(10, msec);

    reverseIntake(40);
    wait(2, sec);

    runIntake(20);
    wait(900, msec);

    stopIntake();
    wait(10, msec);

    reverseIntake(40);
    wait(3, sec);

    stopIntake();
    wait(10, msec);

    // going toward loader
    setSorterEnabled(true);
    m.driveHeadingCC(44.0, 30, 9000, 139);
    m.turnTo(90, 5000);
    wait(10, msec);

    ballLoader.toggles();
    runIntake(100);
    driveDistanceByMotors(-13, 25, 5000);
    wait(10, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(500, msec);

    stopIntake();

    // going toward goal
    stopIntake();
    wait(10, msec);
    wings.toggle();
    m.driveHeadingCC(33, 20, 7000, 90);
    wait(10, msec);

    runIntake(40);
    wait(4, sec);

    reverseIntake(20);
    wait(2, sec);

    runIntake(40);
    wait(4, sec);

    stopIntake();
    wait(10, msec);
}

static void SkillsRun() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(false);

    bool wingsOut = false;

    while (true) {
        visionAlignOnlyToCenterIdTwo(1);
    }
   
}

static void wip() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(false);

    bool wingsOut = false;

     while (true) {
        visionAlignOnlyToCenterId(1);
    } 
}

static void RoutineOne(){
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(false);
    updateDescore(true);

    m.drive(-0.70, 2500, 100); // was 0.83
    m.turnTo(90, 2000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec);

    runIntake(100);
    wait(10, msec);

    m.driveHeadingCC(-0.25, 2000, 50, 90);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000);
    wait(900, msec);

    stopIntake();
    wait(10, msec);

    m.drive(0.35, 2900, 50);
    ballLoader.toggles();
    runIntake(100);
    wait(1000, msec);

    stopIntake();
    wait(10, msec);

    wings.toggle();
    m.drive(0.25, 1600, 40);
    wait(10, msec);

    // visionAlignOnlyToCenterId(1);
}

// ============================================================
//  Dispatch
// ============================================================
void runAutonomous() {
    setSorterEnabled(false);

    switch (selectedAuton) {
        case AutonRoutine::NONE:                    break;
        case AutonRoutine::TEST:                    break;
        case AutonRoutine::RED_LEFT:                break;
        case AutonRoutine::RED_RIGHT:               break;
        case AutonRoutine::BLUE_LEFT:               blueLeft();              break;
        case AutonRoutine::BLUE_RIGHT:              blueRight();             break;
        case AutonRoutine::AUTO_CORRECT_BLUE_RIGHT: autoCorrectBlueRight();  break;
        case AutonRoutine::AUTO_CORRECT_BLUE_LEFT:  autoCorrectBlueLeft();   break;
        case AutonRoutine::AUTO_CORRECT_RED_RIGHT:  autoCorrectRedRight();   break;
        case AutonRoutine::AUTO_CORRECT_RED_LEFT:   autoCorrectRedLeft();    break;
        case AutonRoutine::SIMPLE_AUTON_LEFT:       SimpleAutonLeft();       break;
        case AutonRoutine::SIMPLE_AUTON_RIGHT:      SimpleAutonRight();      break;
        case AutonRoutine::SKILLS:                  HardCodedRightSkills();  break;
        case AutonRoutine::HARD_CODED_RIGHT:        HardCodedRight();        break;
        case AutonRoutine::HARD_CODED_MESSUP_RIGHT: HardCodedRightMessUp();  break;
        case AutonRoutine::HARD_CODED_MESSUP_RIGHT2:HardCodedRightMessUp2(); break;
        case AutonRoutine::SHIT_SKILLS:             ShitSkills();            break;
        case AutonRoutine::SKILLS2:                 skills2();               break;
        case AutonRoutine::BAKERS:                  Auton_SKILLS();          break;
        case AutonRoutine::AUTON_SKILLS:            SkillsRun();             break;
        case AutonRoutine::WIP:                     wip();                   break;
        case AutonRoutine::ROUTINEONE:              RoutineOne();            break;
        default:                                    break;
    }

    setSorterEnabled(false);
}