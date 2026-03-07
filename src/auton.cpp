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

static void skills2(){
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
    OuttakeA.spin(forward, 100, percent);
    wait(10, msec);

    m.driveHeadingCC(-0.301, 2000, 50, 90); //was -0.3302
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000); //was -6
    wait(900, msec);

    // driveDistanceByMotors(1, 17, 5000);
    // wait(10, msec);
    // driveDistanceByMotors(-2, 17, 5000);
    // wait(500, msec);

    stopIntake();
    OuttakeA.stop();
    wait(10, msec);

    m.drive(0.35, 2900, 50); //was 13, 10, 9
    ballLoader.toggles();
    runIntake(100);
    // OuttakeA.spin(forward, 100, percent);
    wait(1000, msec);

    stopIntake();
    OuttakeA.stop();
    setSorterEnabled(false);
    wait(10, msec);

    m.turnBy(136, 8000); // WAS 139
    wait(10, msec);
    
    m.drive(-1.18, 5500, 50);
    wait(10, msec);

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
}

//this one for strong teams
static void HardCodedRightMessUp2(){
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
    OuttakeA.spin(forward, 100, percent);
    wait(10, msec);

    m.driveHeadingCC(-0.301, 2000, 50, 90); //was -0.3302
    wait(900, msec);

    driveDistanceByMotors(1, 17, 7000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 7000); //was -6
    wait(900, msec);

    // driveDistanceByMotors(1, 17, 5000);
    // wait(10, msec);
    // driveDistanceByMotors(-2, 17, 5000);
    // wait(500, msec);

    stopIntake();
    OuttakeA.stop();
    wait(10, msec);

    m.drive(0.35, 2900, 50); //was 13, 10, 9
    ballLoader.toggles();
    runIntake(100);
    // OuttakeA.spin(forward, 100, percent);
    wait(1000, msec);

    stopIntake();
    OuttakeA.stop();
    setSorterEnabled(false);
    wait(10, msec);

    m.turnBy(136, 8000); // WAS 139
    wait(10, msec);

    // runIntake(10);
    // wait(900, msec);

    // stopIntake();
    // driveDistanceByMotors(-46.0, 30, 9000);
    m.drive(-1.18, 5500, 50);
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
    m.driveHeading(-0.84, 2500, 50, 0);
    wait(10, msec);
    m.turnTo(90, 2000);
    wait(10, msec);

    ballLoader.toggles();
    wait(1, sec); 

    runIntake(100);
    OuttakeA.spin(forward, 100, percent);
    wait(10, msec);

    m.driveHeadingCC(-0.301, 2000, 50, 90); //was -0.3302
    wait(900, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000); //was -6
    wait(400, msec);

    stopIntake();
    OuttakeA.stop();
    wait(10, msec);
//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// going toward opposite side
    driveDistanceByMotors(6, 30, 1000);
    // reverseOutake(20);
    // reverseIntake(20);
    // wait(0.500, msec);

    // stopIntake();
    // stopOutake();
    reverseIntake(20);
    reverseOutake(20);
    wait(0.250, msec);

    stopIntake();
    stopOutake();

    // ballLoader.toggles();
    runIntake(20);
    m.turnTo(45, 1500);
    wait(10, msec);

    //driveDistanceByMotors(22, 20, 3000);
    m.driveHeadingCC(0.55, 3000, 40, 45);
    wait(10, msec);

    m.turnTo(90, 1500);
    m.driveHeading(2, 3000, 40, 90);
    wait(10, msec);
// /////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////turning towards goal and depositing

    reverseIntake(40);
    wait(200, msec);

    stopIntake();
    wings.toggle();
    wait(10, msec);

    // wings.toggle();
    // wait(50, msec);
    // wings.toggle();
    m.turnTo(180, 2000);
    //driveDistanceByMotors(19, 30, 3000);
    wings.toggle();
    wait(50, msec);
    wings.toggle();
    m.drive(0.44, 2000, 40); //was 0.42
    wait(10, msec);


    m.turnTo(-90, 2000);
    visionAlignOnlyToCenterId(
        1,      // GOAL signature
        3000,
        0.13,
        0.0,
        5,
        6,
        8,
        158.0,
        0.0
    );
    wait(0.800, msec);

// if (!seen) stopDrive(brake);

    m.drive(0.50, 2000, 50);
    // driveDistanceByMotors(20, 50, 2000);
    m.addFix(-90);
    driveDistanceByMotors(8, 100, 1500);

    reverseIntake(40);
    reverseOutake(40);
    wait(250, msec);

    runIntake(100);
    runOutake(80);
    wait(500, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(80);
    wait(500, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(80);
    wait(500, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(80);
    wait(500, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(80);
    wait(500, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(80);
    wait(900, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    // ballLoader.toggles();
    wait(1, sec); 

    // visionAlignOnlyToCenterId(
    //     1,      // GOAL signature
    //     3000,
    //     0.13,
    //     0.0,
    //     5,
    //     6,
    //     8,
    //     158.0,
    //     0.0
    // );

    // // driveDistanceByMotors(-0.6, 30, 2500);
    // // driveDistanceByMotors(-23.6, 30, 3000);
    wings.toggle();
    runIntake(100);
    OuttakeA.spin(forward, 100, percent);
    wait(10, msec);

    m.driveHeading(-0.8, 2000, 30, -90);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000); //was -6
    wait(500, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000); //was -6
    wait(500, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000); //was -6
    wait(100, msec);

    driveDistanceByMotors(2, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-3, 20, 2000); //was -6
    wait(400, msec);


    stopIntake();
    OuttakeA.stop();
    wait(10, msec);

    wings.toggle();
    m.drive(0.5, 1000, 30);
    visionAlignOnlyToCenterId(
        1,      // GOAL signature
        3000,
        0.13,
        0.0,
        5,
        6,
        8,
        158.0,
        0.0
    );
    wait(0.800, msec);

    m.drive(0.6, 2000, 30);
    m.addFix(-90);
    driveDistanceByMotors(8, 100, 1500);
    
    reverseIntake(40);
    reverseOutake(40);
    wait(250, msec);

    runIntake(100);
    runOutake(80);
    wait(500, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(80);
    wait(500, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(80);
    wait(500, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(80);
    wait(500, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(80);
    wait(500, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(80);
    wait(500, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    runIntake(100);
    runOutake(80);
    wait(900, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    // driveDistanceByMotors(-2, 20, 1000);
    wings.toggle();

    // wings.toggle();
    // driveDistanceByMotors(-2, 50, 1000);

    // runIntake(100);
    // wait(500, msec);
    // stopIntake();
    // wait(10, msec);

    // wings.toggle();
    // driveDistanceByMotors(5, 50, 1000);

    // runIntake(100);
    // runOutake(80);
    // wait(500, msec);

    // stopIntake();
    // stopOutake();
    // wait(10, msec);

}

static void SimpleAutonLeft() {
    MotionController m;
    m.setAutoCorrectEnabled(true);
    setSorterEnabled(true);  

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

    m.driveHeadingCC(-0.301, 2000, 50, 90); //was -0.3302
    wait(900, msec);

    driveDistanceByMotors(1, 17, 2000);
    wait(10, msec);
    driveDistanceByMotors(-2, 17, 2000); //was -6
    wait(900, msec);

    stopIntake();
    OuttakeA.stop();
    wait(10, msec);

    wings.toggle();
    m.drive(0.60, 5000, 50);
    // visionAlignOnlyToCenterId(
    //     1,      // GOAL signature
    //     2000,
    //     0.18,
    //     0.0,
    //     5,
    //     8,
    //     8,
    //     158.0,
    //     0.0
    // );
    wait(800, msec);

    m.drive(0.50, 3000, 50);
    m.addFix(90);
    runIntake(100);
    runOutake(100);
    wait(900, msec);

    reverseIntake(25);
    reverseOutake(25);
    wait(400, msec);

    runIntake(100);
    runOutake(100);
    wait(2.5, sec);

    stopIntake();
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

    while(true) {
    visionAlignOnlyToCenterId(
        1,      // GOAL signature
        3000,
        0.13,
        0.0,
        5,
        6,
        8,
        158.0,
        0.0
    );
}
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
        case AutonRoutine::SKILLS2: skills2(); break;
        default: break;
    }

    setSorterEnabled(false);
}
