#include "autons.h"
#include "motion.h"
#include "subsystems.h"
#include "vex.h"

using namespace vex;

AutonRoutine selectedAuton = AutonRoutine::AUTO_CORRECT_BLUE_RIGHT;

static void SimpleAutonRight() {
    MotionController m;
    m.setAutoCorrectEnabled(true);

    moveArmLeft(100);
    wait(800, msec);

    stopArm();
    wait(10, msec);

    m.drive(1.12, 6000, 100);
    m.turnTo(-41.0, 6000);
    wait(10, msec);

    m.drive(0.10, 4500, 100);
    wait(10, msec);

    reverseOutake(100);
    reverseIntakeAuto(100);
    wait(2, sec);

    stopIntakeAuto();
    stopOutake();
    wait(10, msec);

    m.drive(-0.40, 5000, 100);
    m.turnBy(180, 4000);
    wait(10, msec);
}

static void SimpleAutonLeft() {
    MotionController m;
    m.setAutoCorrectEnabled(true);

    moveArmLeft(100);
    wait(800, msec);

    stopArm();
    wait(10, msec);

    m.drive(1.22, 6000, 100);
    m.turnTo(40.5, 5000);
    wait(10, msec);

    runIntakeAuto(100);
    runOutake(40);
    wait(2, sec);

    stopIntakeAuto();
    stopOutake();
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

    moveArmLeft(100);
    wait(10, msec);

    stopArm();
    wait(10, msec);

    m.drive(0.75, 5500, 100);
    wait(10, msec);

    m.turnTo(90, 4000);
    wait(10, msec);

    runIntakeAuto(100);
    runOutake(100);

    m.drive(0.29, 5500, 100);
    wait(10, msec);

    stopIntakeAuto();
    stopOutake();
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
    reverseOutake(100);
    wait(3, sec);

    stopIntakeAuto();
    stopOutake();
    wait(10, msec);
}

static void autoCorrectRedLeft() {
    MotionController m;
    m.setAutoCorrectEnabled(true);

    m.driveAC(0.82, 4000, 80);
    wait(10, msec);

    m.turnTo(90, 4000);
    wait(10, msec);

    m.driveAC(0.30, 4000, 80);
    wait(5000, msec);

    m.driveHeadingAC(-0.05, 4000, 80, 90);
    wait(10, msec);
}

static void autoCorrectBlueLeft() {
    runIntakeAuto(100);
    wait(2, sec);
    stopIntakeAuto();
    wait(10, msec);
}

static void autoCorrectRedRight() {}

static void blueLeft() {}

void runAutonomous() {
    setSorterEnabled(false);

    switch (selectedAuton) {
        case AutonRoutine::NONE: break;
        case AutonRoutine::TEST: break;
        case AutonRoutine::RED_LEFT: break;
        case AutonRoutine::RED_RIGHT: break;
        case AutonRoutine::BLUE_LEFT: blueLeft(); break;
        case AutonRoutine::BLUE_RIGHT: blueRight(); break;
        case AutonRoutine::AUTO_CORRECT_BLUE_RIGHT: autoCorrectBlueRight(); break;
        case AutonRoutine::AUTO_CORRECT_BLUE_LEFT: autoCorrectBlueLeft(); break;
        case AutonRoutine::AUTO_CORRECT_RED_RIGHT: autoCorrectRedRight(); break;
        case AutonRoutine::AUTO_CORRECT_RED_LEFT: autoCorrectRedLeft(); break;
        case AutonRoutine::SIMPLE_AUTON_LEFT: SimpleAutonLeft(); break;
        case AutonRoutine::SIMPLE_AUTON_RIGHT: SimpleAutonRight(); break;
        case AutonRoutine::SKILLS: break;
        default: break;
    }

    setSorterEnabled(false);
}
