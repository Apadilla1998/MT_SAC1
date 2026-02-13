#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "vex.h"

extern vex::brain Brain;
extern vex::controller Controller1;

extern vex::motor LeftA;
extern vex::motor LeftB;
extern vex::motor LeftC;
extern vex::motor_group LeftMotorGroup;

extern vex::motor RightA;
extern vex::motor RightB;
extern vex::motor RightC;
extern vex::motor_group RightMotorGroup;

extern vex::motor MainIntake;
extern vex::motor ColorIntake;

extern vex::motor OuttakeA;
extern vex::motor OuttakeB;
extern vex::motor OuttakeC;
extern vex::motor_group Outtake;


extern vex::digital_out wingsPiston;
extern vex::digital_out loader;
extern vex::inertial inertial_sensor;

extern vex::rotation verticalRot;
extern vex::rotation horizontalRot;

extern vex::optical ballSensor;

extern vex::motor DescoreMotor;

namespace config {
    extern const double TRACK_WIDTH_M;
    extern const double TRACKING_WHEEL_CIRCUMFERENCE_M;
    extern const double ARCADE_DEADBAND;
    extern const double SIDE_OFFSET_M;
    extern const double VERT_OFFSET_M;
}

#endif
