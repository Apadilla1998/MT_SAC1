#include "autons.h"
#include "motion.h"
#include "subsystems.h"
#include "robot_config.h"
#include "drive.h"
#include "utils.h"
#include "vex.h"
#include <cmath>

using namespace vex;

AutonRoutine selectedAuton = AutonRoutine::HARD_CODED_MESSUP_RIGHT;

static void driveDistanceByMotors(double distIn, double speedPct, int timeoutMs) {
    const double wheelDiamIn = 3.25;
    const double wheelCircIn = M_PI * wheelDiamIn;
    const double wheelDeg = (distIn / wheelCircIn) * 360.0;

    LeftMotorGroup.resetPosition();
    RightMotorGroup.resetPosition();

    timer t; t.reset();
    const int dir = (distIn >= 0) ? 1 : -1;

    while (t.time(msec) < timeoutMs) {
        double l = std::fabs(LeftMotorGroup.position(deg));
        double r = std::fabs(RightMotorGroup.position(deg));
        double avg = 0.5 * (l + r);

        if (avg >= std::fabs(wheelDeg)) break;

        tankDrive(dir * speedPct, dir * speedPct);
        wait(10, msec);
    }

    stopDrive(brake);
}

static void HardCodedRightSkills(){

}

static void HardCodedRightMessUp(){
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
    OuttakeA.spin(forward, 100, percent);
    wait(10, msec);

    driveDistanceByMotors(-8, 20, 7000);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000); //was -6
    wait(500, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000); //was -6
    wait(500, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    // driveDistanceByMotors(1, 17, 5000);
    // wait(10, msec);
    // driveDistanceByMotors(-2, 17, 5000);
    // wait(900, msec);

    stopIntake();
    OuttakeA.stop();
    wait(10, msec);

    driveDistanceByMotors(9, 15, 9000);
    ballLoader.toggles();
    wait(10, msec);

    m.turnBy(139, 8000);
    wait(10, msec);

    runIntake(10);
    wait(500, msec);

    stopIntake();
    driveDistanceByMotors(-43.0, 30, 7000);
    wait(10, msec);

    // wings.toggle();
    setSorterEnabled(false);
    reverseIntake(40);
    reverseOutake(40);
    wait(2, sec);

    runIntake(20);
    runOutake(20);
    wait(900, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);
    
    reverseIntake(40);
    reverseOutake(40);
    wait(3, sec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    //go to other side of middle goal
    driveDistanceByMotors(13, 24, 9000);
    m.turnTo(90, 7000);
    wait(10, msec);
    driveDistanceByMotors(30, 25, 9000);
    wait(10, msec);

    m.turnTo(43, 6000);
    driveDistanceByMotors(7, 17, 7000);
    wait(10, msec);

    RightMotorGroup.stop(hold);
    LeftMotorGroup.stop(hold);

}

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
    OuttakeA.spin(forward, 100, percent);
    wait(10, msec);
    driveDistanceByMotors(-8, 20, 7000);
    wait(900, msec);

    driveDistanceByMotors(4, 17, 7000);
    driveDistanceByMotors(-5, 15, 7000); //was -6
    wait(900, msec);
    driveDistanceByMotors(4, 17, 7000);
    driveDistanceByMotors(-5, 15, 7000);
    wait(900, msec);

    stopIntake();
    OuttakeA.stop();
    wait(10, msec);

    driveDistanceByMotors(10, 15, 9000);
    ballLoader.toggles();
    wait(10, msec);

    m.turnBy(140, 8000);
    wait(10, msec);

    runIntake(10);
    wait(500, msec);

    stopIntake();
    driveDistanceByMotors(-45.5, 24, 7000);
    wait(10, msec);

    // wings.toggle();
    setSorterEnabled(false);
    reverseIntake(40);
    reverseOutake(40);
    wait(2, sec);

    runIntake(20);
    runOutake(20);
    wait(800, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);
    
    reverseIntake(40);
    reverseOutake(40);
    wait(3, sec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    driveDistanceByMotors(40.0, 24, 7000);
    m.turnBy(-138, 7000);
    wait(10, msec);

    // driveDistanceByMotors(3, 20, 5000);
    // ballLoader.toggles();
    // wait(1, sec);

    // runIntake(100);Q
    // OuttakeA.spin(forward, 100, pct);
    // driveDistanceByMotors(-5, 15, 7000);
    // wait(10, msec);

    // driveDistanceByMotors(4, 17, 7000);
    // driveDistanceByMotors(-5, 15, 7000);
    // wait(900, msec);
    // driveDistanceByMotors(4, 17, 7000);
    // driveDistanceByMotors(-5, 15, 7000);
    // wait(900, msec);

    // stopIntake();
    // OuttakeA.stop();
    // driveDistanceByMotors(5, 17, 7000);
    // wait(10, msec);
    
}

static void SimpleAutonRight() {
    MotionController m;
    m.setAutoCorrectEnabled(true);

    DescoreMotor.setStopping(brakeType::hold);
    DescoreMotor.spinFor(forward, 25, degrees);

    m.drive(-0.5, 6000, 50);
    // m.turnTo(-41.0, 6000);
    // wait(10, msec);
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
        case AutonRoutine::SKILLS: HardCodedRightSkills(); break;
        case AutonRoutine::HARD_CODED_RIGHT: HardCodedRight(); break;
        case AutonRoutine::HARD_CODED_MESSUP_RIGHT: HardCodedRightMessUp(); break;
        default: break;
    }

    setSorterEnabled(false);
}
