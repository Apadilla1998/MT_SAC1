// ============================
// robot_config.cpp
// ============================
#include "robot_config.h"
#include <cmath>

using namespace vex;

brain Brain;
controller Controller1;

// ---------------- Drive ----------------
motor LeftA(PORT3, ratio6_1, false);
motor LeftB(PORT4,  ratio6_1, false);
motor LeftC(PORT5,  ratio6_1, false);
motor LeftD(PORT6, ratio6_1, false);
motor_group LeftMotorGroup(LeftA, LeftB, LeftC, LeftD);

motor RightA(PORT7, ratio6_1, true);
motor RightB(PORT8, ratio6_1, true);
motor RightC(PORT9, ratio6_1, true);
motor RightD(PORT10, ratio6_1, true);
motor_group RightMotorGroup(RightA, RightB, RightC, RightD);


// ---------------- Intake and Outtake ----------------
motor LeverArm(PORT20, ratio6_1, true);
motor Intake(PORT2, ratio6_1, true);


//------------------Descore------------------
motor DescoreMotor(PORT11, ratio6_1, true);

// ---------------- Sensors ----------------
inertial inertial_sensor(PORT17);
rotation verticalRot(PORT16, true);     // was true

optical ballSensor(PORT18);
// optical ballSensor2(PORT14);
vex::vision::signature GOAL(
  1,
  6045, 6711, 6378,
  -3005, -2555, -2780,
  2.5,
  0
);

vex::vision VisionSensor(vex::PORT19, 30, GOAL);

// ---------------- Others ----------------
digital_out wingsPiston(Brain.ThreeWirePort.C);
digital_out loader(Brain.ThreeWirePort.E);
digital_out OuttakePiston(Brain.ThreeWirePort.D);

potV2 ColorSort(Brain.ThreeWirePort.F);
potV2 DescorePot(Brain.ThreeWirePort.G);
potV2 LeverArmPot(Brain.ThreeWirePort.H);


// ---------------- Config ----------------
namespace config {
    const double TRACK_WIDTH_M = 0.320;
    const double TRACKING_WHEEL_CIRCUMFERENCE_M = 0.050 * M_PI;
    const double ARCADE_DEADBAND = 0.0;
    const double SIDE_OFFSET_M = 0.0;
    const double VERT_OFFSET_M = 0.0;
}