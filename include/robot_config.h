// ============================
// robot_config.h
// ============================
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "vex.h"
#include <cmath>

using namespace vex;

// Brain / Controller
extern brain Brain;
extern controller Controller1;

// Drive
extern motor LeftA;
extern motor LeftB;
extern motor LeftC;
extern motor_group LeftMotorGroup;

extern motor RightA;
extern motor RightB;
extern motor RightC;
extern motor_group RightMotorGroup;

// Intake / Outtake
extern motor MainIntake;
extern motor ColorIntake;

extern motor OuttakeA;
extern motor OuttakeB;
extern motor OuttakeC;
extern motor_group Outtake;

extern potV2 Descore;

// Pneumatics
extern digital_out wingsPiston;
extern digital_out loader;

// IMU + tracking
extern inertial inertial_sensor;
extern rotation verticalRot;
extern rotation horizontalRot;

// Vision (signatures + sensor)
extern vex::vision::signature GOAL;
extern vex::vision::signature SIG_2;
extern vex::vision::signature SIG_3;
extern vex::vision::signature SIG_4;
extern vex::vision::signature SIG_5;
extern vex::vision::signature SIG_6;
extern vex::vision::signature SIG_7;

extern vision VisionSensor;

// Other sensors
extern optical ballSensor;
// extern optical ballSensor2;

// Other motors
extern motor DescoreMotor;

// Config constants
namespace config {
    extern const double TRACK_WIDTH_M;
    extern const double TRACKING_WHEEL_CIRCUMFERENCE_M;
    extern const double ARCADE_DEADBAND;
    extern const double SIDE_OFFSET_M;
    extern const double VERT_OFFSET_M;
}

#endif