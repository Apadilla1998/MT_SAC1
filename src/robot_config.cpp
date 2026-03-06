// ============================
// robot_config.cpp
// ============================
#include "robot_config.h"
#include <cmath>

using namespace vex;

brain Brain;
controller Controller1;

// ---------------- Drive ----------------
motor LeftA(PORT10, ratio6_1, false);
motor LeftB(PORT9,  ratio6_1, false);
motor LeftC(PORT8,  ratio6_1, true);  // bac
motor_group LeftMotorGroup(LeftA, LeftB, LeftC);

motor RightA(PORT1, ratio6_1, true);
motor RightB(PORT2, ratio6_1, true);
motor RightC(PORT3, ratio6_1, false); // bacl
motor_group RightMotorGroup(RightA, RightB, RightC);

// ---------------- Intake ----------------
motor MainIntake(PORT20, ratio6_1, false);
motor ColorIntake(PORT19, ratio6_1, false);

// ---------------- Outtake ----------------
motor OuttakeA(PORT11, ratio6_1, false); // first one
motor OuttakeB(PORT5,  ratio6_1, false); // towards end
motor OuttakeC(PORT16, ratio6_1, false); // top one
motor_group Outtake(OuttakeA, OuttakeB, OuttakeC);

potV2 Descore(Brain.ThreeWirePort.D);

// ---------------- Pneumatics ----------------
digital_out wingsPiston(Brain.ThreeWirePort.A);
digital_out loader(Brain.ThreeWirePort.B);

// ---------------- Sensors ----------------
inertial inertial_sensor(PORT21);

rotation verticalRot(PORT7, true);     // was true
rotation horizontalRot(PORT6, false);

optical ballSensor(PORT18);
// optical ballSensor2(PORT14);

// ---------------- Vision ----------------
// From your NEW Vision Utility JSON (brightness=10, signature ID=1)
vex::vision::signature GOAL(
  1,
  3359, 4429, 3894,
  -3295, -2827, -3061,
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
  vex::PORT12,
  10,
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