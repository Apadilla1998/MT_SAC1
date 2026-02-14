#include "robot_config.h"
#include <cmath>

using namespace vex;

brain Brain;
controller Controller1;

motor LeftA(PORT10, ratio6_1, false);
motor LeftB(PORT9, ratio6_1, false);
motor LeftC(PORT8, ratio6_1, true); //bac
motor_group LeftMotorGroup(LeftA, LeftB, LeftC);

motor RightA(PORT1, ratio6_1, true);
motor RightB(PORT2, ratio6_1, true);
motor RightC(PORT3, ratio6_1, false); //bacl
motor_group RightMotorGroup(RightA, RightB, RightC);

motor MainIntake(PORT20, ratio6_1, false);
motor ColorIntake(PORT19, ratio6_1, false);


motor OuttakeA(PORT11, ratio6_1, false);
motor OuttakeB(PORT5, ratio6_1, false);
motor OuttakeC(PORT16, ratio6_1, false);
motor_group Outtake(OuttakeA, OuttakeB, OuttakeC);

digital_out wingsPiston(Brain.ThreeWirePort.A);
digital_out loader(Brain.ThreeWirePort.B);

inertial inertial_sensor(PORT21);

rotation verticalRot(PORT7, false); //was true
rotation horizontalRot(PORT6, false);

vision AiSensor(PORT12);
optical ballSensor(PORT18);
//optical ballSensor2(PORT14);

motor DescoreMotor(PORT17, ratio36_1, true);

namespace config {
    const double TRACK_WIDTH_M = 0.320;
    const double TRACKING_WHEEL_CIRCUMFERENCE_M = 0.050 * M_PI;
    const double ARCADE_DEADBAND = 0.0;
    const double SIDE_OFFSET_M = 0.0;
    const double VERT_OFFSET_M = 0.0;
}
