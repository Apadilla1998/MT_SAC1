#include "autons.h"
#include "motion.h"
#include "subsystems.h"
#include "robot_config.h"
#include "drive.h"
#include "utils.h"
#include "vex.h"

using namespace vex;

AutonRoutine selectedAuton = AutonRoutine::HARD_CODED_RIGHT;

static void driveDistanceByMotors(double distIn, double speedPct, int timeoutMs) {
  // CHANGE THIS to your real wheel diameter:
  const double wheelDiamIn = 3.25; // example

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

static void driveForMsHoldHeading(double fwdPct, int ms, double targetHeadingDeg,
                                  double kH = 1.6, double maxTurnPct = 25.0,
                                  brakeType stopMode = brakeType::brake) {
  int t = 0;
  while (t < ms) {
    const double curr = inertial_sensor.heading(deg);     // 0..360
    const double err  = wrap180(targetHeadingDeg - curr); // -180..180

    const double turn = clampD(kH * err, -maxTurnPct, maxTurnPct);

    // Use your drive helper
    tankDrive(fwdPct + turn, fwdPct - turn);

    wait(10, msec);
    t += 10;
  }
  stopDrive(stopMode);
}


static void HardCodedRight() {
    inertial_sensor.setRotation(0, deg);

    MotionController m;
    m.setAutoCorrectEnabled(false);
    

    driveDistanceByMotors(-42.2, 12, 7000); //was -22.2 or -42.2

    m.turnTo(-50.0, 5000);
    wait(10, msec);
    driveDistanceByMotors(-6.5, 12, 7000); // was -1.93
    wait(10, msec);

    reverseIntake(25);
    reverseOutake(25);
    wait(3, sec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    // driveDistanceByMotors(28.2, 20, 6000);
    // m.turnTo(45, 2500);
    // wait(10, msec);


}

/*
//   inertial_sensor.setRotation(0, deg);

//   static MotionController m;
//   m.setAutoCorrectEnabled(false);

//   // 1) Fixed distance (inches) using motor encoders
//   driveDistanceByMotors(54.3, 70, 6000);   // 24 inches forward

//   // 2) Turn 45 degrees using your PID (IMU)
//   m.turnTo(-41.0, 1200);
//   wait(10, msec);
//   driveDistanceByMotors(0.39, 70, 4000);

//   // 3) Reverse intake for fixed time
//   MainIntake.spin(reverse, 100, pct);
//   ColorIntake.spin(reverse, 100, pct);
//   Outtake.spin(reverse, 100, pct);
//   wait(350, msec);

//   MainIntake.stop(brake);
//   ColorIntake.stop(brake);

*/


static void SimpleAutonRight() {
    MotionController m;
    m.setAutoCorrectEnabled(true);

    DescoreMotor.setStopping(brakeType::hold);   
    DescoreMotor.spinFor(forward, 25, degrees); 

    m.driveHeadingCC(0.5, 6000, 50, 0);
    m.turnTo(-41.0, 6000);
    wait(10, msec);

    // m.drive(0.10, 4500, 100);
    // wait(10, msec);

    // reverseOutake(100);
    // reverseIntakeAuto(100);
    // wait(2, sec);

    // stopIntakeAuto();
    // stopOutake();
    // wait(10, msec);

    // m.drive(-0.40, 5000, 100);
    // m.turnBy(180, 4000);
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
        case AutonRoutine::SKILLS: break;
        case AutonRoutine::HARD_CODED_RIGHT: HardCodedRight(); break;
        default: break;
    }

    setSorterEnabled(false);
}
