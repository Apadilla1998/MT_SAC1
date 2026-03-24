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

// ---------------- Intake ----------------
motor MainIntake(PORT2, ratio6_1, true);
motor ColorIntake(PORT1, ratio6_1, false);

// ---------------- Outtake ----------------
motor OuttakeA(PORT20, ratio6_1, true); // first one
motor OuttakeB(PORT1,  ratio6_1, false); // towards end
motor OuttakeC(PORT1, ratio6_1, false); // top one
motor_group Outtake(OuttakeA, OuttakeB, OuttakeC);

potV2 Descore(Brain.ThreeWirePort.D);

// ---------------- Pneumatics ----------------
digital_out wingsPiston(Brain.ThreeWirePort.A);
digital_out loader(Brain.ThreeWirePort.B);

// ---------------- Sensors ----------------
inertial inertial_sensor(PORT18);

rotation verticalRot(PORT19, true);     // was true
rotation horizontalRot(PORT1, false);

optical ballSensor(PORT1);
// optical ballSensor2(PORT14);

// ---------------- Vision ----------------
vex::vision::signature GOAL(
  1,
  6045, 6711, 6378,
  -3005, -2555, -2780,
  2.5,
  0
);

vex::vision::signature SIG_2(2, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_3(3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4(4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5(5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6(6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7(7, 0, 0, 0, 0, 0, 0, 2.5, 0);

vex::vision VisionSensor(
  vex::PORT8,
  30,
  GOAL, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7
);

// ---------------- Other motors ----------------
motor DescoreMotor(PORT17, ratio18_1, true);

// ---------------- Config ----------------
namespace config {
    const double TRACK_WIDTH_M = 0.320;
    const double TRACKING_WHEEL_CIRCUMFERENCE_M = 0.050 * M_PI;
    const double ARCADE_DEADBAND = 0.0;
    const double SIDE_OFFSET_M = 0.0;
    const double VERT_OFFSET_M = 0.0;
}