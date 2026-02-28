#include "autons.h"
#include "motion.h"
#include "subsystems.h"
#include "robot_config.h"
#include "drive.h"
#include "utils.h"
#include "vex.h"
#include <cmath>

using namespace vex;

AutonRoutine selectedAuton = AutonRoutine::HARD_CODED_MESSUP_RIGHT2; //HARD_CODED_MESSUP_RIGHT, HARD_CODED_RIGHT, HARD_CODED_MESSUP_RIGHT2

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
    MotionController m;
    m.setAutoCorrectEnabled(true);
    //moveArmRight(100);
    wait(10, msec);
    //setSorterEnabled(false);

    //DescoreMotor.spinFor(500, msec);
    wings.toggle();
    m.driveHeadingCC(-0.812, 8000, 40, 0);
    wait(10, msec);
    m.turnTo(90, 3000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec); 

    runIntake(100);
    OuttakeA.spin(forward, 100, percent);
    wait(10, msec);

    driveDistanceByMotors(-10.5, 30, 6000);
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000); //was -6
    wait(500, msec);

    // OuttakeC.spin(forward, 50, percent);
    // OuttakeB.spin(forward, 50, percent);
    // wait(400, msec);

    // OuttakeC.stop();
    // OuttakeB.stop();
    OuttakeB.spin(reverse, 25, pct);
    OuttakeA.spin(forward, 50, percent);
    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000); //was -6
    wait(900, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000); //was -6
    wait(500, msec);

    stopIntake();
    stopOutake();
    driveDistanceByMotors(6, 20, 5000);
    ballLoader.toggles();
    wait(10, msec);

    m.turnTo(45, 4500);
    driveDistanceByMotors(22, 20, 7000);
    wait(10, msec);

    m.turnTo(90, 45000);
    m.driveHeadingCC(1.83, 9000, 40, 90);

    m.turnTo(180, 5000);
    driveDistanceByMotors(18, 25, 6000); //was 18
    wait(10, msec);

    m.turnTo(-90, 6000);
    // moveArmRight(100);
    // wings.toggle();
    driveDistanceByMotors(9, 25, 6000);
    wait(10, msec);

    reverseIntake(20);
    reverseOutake(20);
    wait(700, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(100);
    wait(900, msec);
    
    reverseIntake(40);
    reverseOutake(40);
    wait(700, msec);

    runIntake(100);
    runOutake(100);
    wait(3000, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

////////////////////////////////////////////
//going towaerd loader
    driveDistanceByMotors(-6, 25, 6000);
    ballLoader.toggles();
    wait(10, msec);

    runIntake(100);
    runOutake(100);
    driveDistanceByMotors(-20, 27, 8000);
    wait(10, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000); //was -6
    wait(500, msec);

    OuttakeC.spin(forward, 50, percent);
    OuttakeB.spin(forward, 50, percent);
    wait(400, msec);

    OuttakeC.stop();
    OuttakeB.stop();
    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000); //was -6
    wait(900, msec);

    driveDistanceByMotors(1, 17, 5000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 5000);
    wait(500, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000); //was -6
    wait(500, msec);

    stopIntake();
    stopOutake();

    driveDistanceByMotors(6, 30, 6000);
    ballLoader.toggles();
    wait(10, msec);

    m.driveHeadingCC(18, 6000, 30, -90);
    runIntake(100);
    runOutake(100);
    wait(10, msec);
    
}

//this one for strong teams
static void HardCodedRightMessUp2(){
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(true);
    
    driveDistanceByMotors(-30, 24, 7000); //was 32
    wait(10, msec);
    m.turnTo(90, 3000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec); 

    runIntake(100);
    OuttakeA.spin(forward, 100, percent);
    wait(10, msec);

    driveDistanceByMotors(-12, 20, 7000); //was 13
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

    // driveDistanceByMotors(1, 17, 5000);
    // wait(10, msec);
    // driveDistanceByMotors(-2, 17, 5000);
    // wait(500, msec);

    // driveDistanceByMotors(1, 17, 5000);
    // wait(10, msec);
    // driveDistanceByMotors(-2, 17, 5000);
    // wait(900, msec);

    stopIntake();
    OuttakeA.stop();
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
    driveDistanceByMotors(-43.0, 30, 7000);
    wait(10, msec);

    // wings.toggle();
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

/////////////////////////////////////////////////////////////
//going toward loader
    setSorterEnabled(true);
    driveDistanceByMotors(44.0, 30, 9000);
    m.turnTo(90, 5000);
    wait(10, msec);

    ballLoader.toggles();
    runIntake(100);
    runOutake(100);
    driveDistanceByMotors(-13, 25, 5000);
    wait(10, msec);

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

    stopIntake();
    stopOutake();
/////////////////////////////////////////////////////////////////////////

    //going toward goal
//////////////////////////////////////////////////
    stopIntake();
    OuttakeA.stop();
    OuttakeB.stop();
    OuttakeC.stop();
    wait(10, msec);
    wings.toggle();
    driveDistanceByMotors(33, 20, 7000);
    wait(10, msec);

    runIntake(40);
    runOutake(75);
    wait(4, sec);

    reverseOutake(20);
    reverseIntake(20);
    wait(2, sec);

    runIntake(40);
    runOutake(75);
    wait(4, sec);

    stopIntake();
    stopOutake();
    wait(10, msec);
///////////////////////////////////////////////////////////////////////
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

    // driveDistanceByMotors(1, 17, 5000);
    // wait(10, msec);
    // driveDistanceByMotors(-2, 17, 5000);
    // wait(500, msec);

    // driveDistanceByMotors(1, 17, 5000);
    // wait(10, msec);
    // driveDistanceByMotors(-2, 17, 5000);
    // wait(900, msec);

    stopIntake();
    OuttakeA.stop();
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
    driveDistanceByMotors(26, 25, 9000);
    wait(10, msec);

    m.turnTo(43, 6000);
    driveDistanceByMotors(11, 17, 7000);
    wait(10, msec);

    RightMotorGroup.stop(hold);
    LeftMotorGroup.stop(hold);
}

//this one for weak teams
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

    //going toward goal
    ///////////////////////
    stopIntake();
    OuttakeA.stop();
    OuttakeB.stop();
    OuttakeC.stop();
    wait(10, msec);
    wings.toggle();
    driveDistanceByMotors(33, 20, 7000);
    wait(10, msec);

    runIntake(40);
    runOutake(75);
    wait(4, sec);

    reverseOutake(20);
    reverseIntake(20);
    wait(2, sec);

    runIntake(40);
    runOutake(75);
    wait(4, sec);

    stopIntake();
    stopOutake();
    wait(10, msec);
    /////////////////////


    //goign toward loader
    /////////////////////////////////
    setSorterEnabled(true);
    driveDistanceByMotors(-3, 20, 7000);
    wings.toggle();
    wait(10, msec);

    runIntake(100);
    OuttakeA.spin(forward, 100, percent);
    driveDistanceByMotors(-25.5, 20, 6000);
    wait(10, msec);

    // OuttakeC.spin(reverse, 50, pct);
    // OuttakeB.spin(forward, 25, percent);
    // wait(10, msec);
    driveDistanceByMotors(-10, 20, 7000);
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
    wait(900, msec);

    stopIntake();
    OuttakeA.stop();
    wait(10, msec);
    ////////////////////////

    //going toward middle goal
    ////////////////////////////////
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

    ballLoader.toggles();
    wait(10, msec);
    ////////////////////////////
    
}

static void SimpleAutonRight() {
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
    OuttakeA.spin(forward, 100, percent);
    wait(10, msec);

    driveDistanceByMotors(-13, 20, 7000);
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

    // driveDistanceByMotors(1, 17, 5000);
    // wait(10, msec);
    // driveDistanceByMotors(-2, 17, 5000);
    // wait(500, msec);

    // driveDistanceByMotors(1, 17, 5000);
    // wait(10, msec);
    // driveDistanceByMotors(-2, 17, 5000);
    // wait(900, msec);

    stopIntake();
    OuttakeA.stop();
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

    // wings.toggle();
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

/////////////////////////////////////////////////////////////
//going toward loader
    setSorterEnabled(true);
    m.driveHeadingCC(44.0, 30, 9000, 139);
    m.turnTo(90, 5000);
    wait(10, msec);

    ballLoader.toggles();
    runIntake(100);
    runOutake(100);
    driveDistanceByMotors(-13, 25, 5000);
    wait(10, msec);

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

    stopIntake();
    stopOutake();
/////////////////////////////////////////////////////////////////////////

    //going toward goal
//////////////////////////////////////////////////
    stopIntake();
    OuttakeA.stop();
    OuttakeB.stop();
    OuttakeC.stop();
    wait(10, msec);
    wings.toggle();
    m.driveHeadingCC(33, 20, 7000, 90);
    wait(10, msec);

    runIntake(40);
    runOutake(75);
    wait(4, sec);

    reverseOutake(20);
    reverseIntake(20);
    wait(2, sec);

    runIntake(40);
    runOutake(75);
    wait(4, sec);

    stopIntake();
    stopOutake();
    wait(10, msec);
///////////////////////////////////////////////////////////////////////
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
        case AutonRoutine::HARD_CODED_MESSUP_RIGHT2: HardCodedRightMessUp2(); break;
        default: break;
    }

    setSorterEnabled(false);
}
