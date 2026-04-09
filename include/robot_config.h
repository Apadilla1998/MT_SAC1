#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "vex.h"

// Core
extern vex::brain Brain;
extern vex::controller Controller1;

// Drive
extern vex::motor LeftA;
extern vex::motor LeftB;
extern vex::motor LeftC;
extern vex::motor LeftD;
extern vex::motor_group LeftMotorGroup;

extern vex::motor RightA;
extern vex::motor RightB;
extern vex::motor RightC;
extern vex::motor RightD;
extern vex::motor_group RightMotorGroup;

// Intake / Outtake
extern vex::motor Intake;
extern vex::motor LeverArm;
extern vex::motor colorSortMotor;

// Descore
extern vex::motor DescoreMotor;

// Sensors
extern vex::inertial inertial_sensor;
extern vex::rotation verticalRot;
extern vex::optical ballSensor;

extern vex::vision::signature GOAL;
extern vex::vision VisionSensor;

// Pneumatics / 3-wire
extern vex::digital_out wingsPiston;
extern vex::digital_out loader;
extern vex::digital_out DescorePiston;

extern vex::pot ColorSort;
extern vex::pot DescorePot;
extern vex::pot LeverArmPot;

// Config constants
namespace config {
    extern const double TRACK_WIDTH_M;
    extern const double TRACKING_WHEEL_CIRCUMFERENCE_M;
    extern const double ARCADE_DEADBAND;
    extern const double SIDE_OFFSET_M;
    extern const double VERT_OFFSET_M;
}

#endif