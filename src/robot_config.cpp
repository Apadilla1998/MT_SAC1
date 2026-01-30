#include "robot_config.h"
#include <cmath>

using namespace vex;

brain Brain;
controller Controller1;

motor LeftA(PORT3, ratio6_1, false);
motor LeftB(PORT19, ratio6_1, false);
motor LeftC(PORT10, ratio6_1, false);
motor_group LeftMotorGroup(LeftA, LeftB, LeftC);

motor RightA(PORT8, ratio6_1, true);
motor RightB(PORT12, ratio6_1, true);
motor RightC(PORT21, ratio6_1, true);
motor_group RightMotorGroup(RightA, RightB, RightC);

motor MainIntake(PORT11, ratio6_1, false);
motor ColorIntake(PORT13, ratio18_1, false);

motor Outtake(PORT20, ratio6_1, false);

digital_out wingsPiston(Brain.ThreeWirePort.B);
inertial inertial_sensor(PORT1);

rotation verticalRot(PORT9, false);
rotation horizontalRot(PORT6, false);

optical ballSensor(PORT15);

motor DescoreMotor(PORT17, ratio36_1, true);

namespace config {
    const double TRACK_WIDTH_M = 0.320;
    const double TRACKING_WHEEL_CIRCUMFERENCE_M = 0.050 * M_PI;
    const double ARCADE_DEADBAND = 0.0;
    const double SIDE_OFFSET_M = 0.0;
    const double VERT_OFFSET_M = 0.0;
}
