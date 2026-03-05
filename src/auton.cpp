#include "autons.h"
#include "motion.h"
#include "subsystems.h"
#include "robot_config.h"
#include "drive.h"
#include "utils.h"
#include "vex.h"
#include "manual.h"
#include "vision_utils.h"
#include "descore.h"
#include <cmath>

using namespace vex;

AutonRoutine selectedAuton = AutonRoutine::SIMPLE_AUTON_RIGHT; //HARD_CODED_MESSUP_RIGHT, HARD_CODED_RIGHT, HARD_CODED_MESSUP_RIGHT2, IMPLE_AUTON_RIGHT



    // bool loaderSeen = visionTurnToCenter(
    //     GOAL,      // signature
    //     1200,      // timeoutMs
    //     0.35,      // kP
    //     0.10,      // kD
    //     4,         // deadbandPx
    //     35,        // maxTurnPct
    //     10,        // lostFramesToFail
    //     158.0,     // centerX
    //     15.0       // offsetX (set 0.0 if you don't want bias)
    // );

// If it stops too far: increase targetWidthPx (e.g. 130–160).

// If it rams too hard: lower drivePct (e.g. 15–18).

// If it wiggles: increase deadbandPx to 6 or lower kTurnP to 0.25.

    // // Optional: if it didn't see the loader, still continue auton
    // if (!loaderSeen) {
    //     // quick safety stop (visionTurnToCenter already stops, this is just extra)
    //     stopDrive(brake);
    // }

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
    m.driveHeadingCC(-0.812, 8000, 40, 0);                  //sam wuz here
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

static void ShitSkills(){
    OuttakeB.spinFor(1, sec);
    wait(200, msec);
    OuttakeB.stop();
}

//this one for strong teams
static void HardCodedRightMessUp2(){
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(true);
    
    driveDistanceByMotors(-32, 24, 7000); //was 32
    wait(10, msec);
    m.turnTo(90, 3000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec); 

    runIntake(100);
    OuttakeA.spin(forward, 100, percent);
    wait(10, msec);

    driveDistanceByMotors(-12, 20, 5000); //was 13
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

    driveDistanceByMotors(10, 15, 9000); //was 13, 10, 9
    ballLoader.toggles();
    runIntake(100);
    // OuttakeA.spin(forward, 100, percent);
    wait(1000, msec);

    stopIntake();
    // OuttakeA.stop();
    setSorterEnabled(false);
    wait(10, msec);

    m.turnBy(136, 8000); // WAS 139
    wait(10, msec);

    // runIntake(10);
    // wait(900, msec);

    // stopIntake();
    driveDistanceByMotors(-46.0, 30, 9000);
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

// /////////////////////////////////////////////////////////////
// //going toward loader
//     setSorterEnabled(true);
//     m.driveHeadingCC(1.143, 9000, 40, 46);
//     m.turnTo(90, 5000);
//     wait(10, msec);

//     ballLoader.toggles();
//     wait(1, sec);

//     runIntake(100);
//     OuttakeA.spin(forward, 100, percent);
//     //runOutake(100);
//     driveDistanceByMotors(-9, 25, 5000);
//     wait(10, msec);

//     driveDistanceByMotors(1, 17, 7000);
//     wait(10, msec);
//     driveDistanceByMotors(-2, 17, 7000); //was -6
//     wait(500, msec);

//     driveDistanceByMotors(1, 17, 5000);
//     wait(10, msec);
//     driveDistanceByMotors(-2, 17, 5000);
//     wait(500, msec);

//     driveDistanceByMotors(1, 17, 7000);
//     wait(10, msec);
//     driveDistanceByMotors(-2, 17, 7000); //was -6
//     wait(500, msec);

//     stopIntake();
//     stopOutake();
// /////////////////////////////////////////////////////////////////////////

//     //going toward goal
// //////////////////////////////////////////////////
//     stopIntake();
//     stopOutake();
//     wait(10, msec);
//     wings.toggle();
//     //driveDistanceByMotors(33, 20, 7000);
//     m.driveHeadingCC(0.8382, 5500, 40, 90);
//     wait(10, msec);

//     runIntake(40);
//     runOutake(75);
//     wait(4, sec);

//     reverseOutake(20);
//     reverseIntake(20);
//     wait(2, sec);

//     runIntake(40);
//     runOutake(75);
//     wait(4, sec);

//     stopIntake();
//     stopOutake();
//     wait(10, msec);
// ///////////////////////////////////////////////////////////////////////
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
    setSorterEnabled(false);  

    // armMoveTo(239, 1000);

//////////////////////////////////////////////////////////////// going towards the loader
    m.driveHeadingCC(-0.84, 2500, 50, 0);
    wait(10, msec);
    m.turnTo(90, 2000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec); 

    runIntake(100);
    OuttakeA.spin(forward, 100, percent);
    wait(10, msec);

    m.driveHeadingCC(-0.3002, 2000, 50, 90); //was -0.3302
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000); //was -6
    wait(900, msec);

    stopIntake();
    OuttakeA.stop();
    wait(10, msec);
//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// going toward opposite side
    driveDistanceByMotors(6, 30, 1000);
    ballLoader.toggles();
    m.turnTo(45, 1500);
    wait(10, msec);

    //driveDistanceByMotors(22, 20, 3000);
    m.driveHeadingCC(0.55, 3000, 40, 45);
    wait(10, msec);

    m.turnTo(90, 1500);
    m.driveHeadingCC(2, 3000, 60, 90);
    wait(10, msec);
// /////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////turning towards goal and depositing

    m.turnTo(180, 2000);
    //driveDistanceByMotors(19, 30, 3000);
    m.driveHeadingCC(0.45, 3000, 40, 180);
    wait(10, msec);

    wings.toggle();
    m.turnTo(-90, 2000);
    bool seen = visionAlignOnlyToCenterId(
        1,      // GOAL signature
        2000,
        0.35,
        0.0,
        5,
        12,
        8,
        158.0,
        0.0
    );

if (!seen) stopDrive(brake);

    driveDistanceByMotors(22, 30, 5000);
    runIntake(100);
    runOutake(100);
    wait(900, msec);

    reverseIntake(25);
    reverseOutake(25);
    wait(400, msec);

    runIntake(100);
    runOutake(100);
    wait(1, sec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    // ballLoader.toggles();
    // wait(1, sec); 

    // m.driveHeadingCC(-1.1, 3000, 40, -90);
    // runIntake(100);
    // OuttakeA.spin(forward, 100, percent);
    // wait(10, msec);

    // m.driveHeadingCC(-0.3002, 2000, 50, -90); //was -0.3302
    // wait(900, msec);

    // driveDistanceByMotors(1, 17, 7000);
    // wait(10, msec);
    // driveDistanceByMotors(-2, 17, 7000); //was -6
    // wait(900, msec);

    // stopIntake();
    // OuttakeA.stop();
    // wait(10, msec);


    // ballLoader.toggles();

    // m.driveHeadingCC(0.8, 3000, 50, 90);
    // visionTurnToCenterId(
    //     1,
    //     2000,
    //     0.35,
    //     0.0,
    //     5,
    //     5,
    //     10,
    //     158.0,
    //     0.0   // offsetX = 0 for tuning
    // );

    // m.driveHeadingCC(0.3, 3000, 50, 90);



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

    // m.turnTo(90, 3000);
    // wait(10, msec);

    // Signature ID = 1 (because you constructed GOAL with id=1)
    bool loaderSeen = visionTurnToCenterId(
        1,      // sigId
        1200,   // timeoutMs
        0.01,   // kP
        0.00,   // kD
        4,      // deadbandPx
        35,     // maxTurnPct
        10,     // lostFramesToFail
        158.0,  // centerX
        15.0    // offsetX (set 0 if you don't want bias)
    );
    (void)loaderSeen; // prevents unused-variable warning
}

static void autoCorrectBlueLeft() {
    runIntakeAuto(100);
    wait(2, sec);
    stopIntakeAuto();
    wait(10, msec);
}

static void autoCorrectRedRight() {}

static void blueLeft() {  while(true) printArmAngleControllerUpdate(); }

static void Trash(){
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
        case AutonRoutine::SHIT_SKILLS: ShitSkills(); break;
        default: break;
    }

    setSorterEnabled(false);
}
