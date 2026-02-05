#ifndef SENSORS_H
#define SENSORS_H

#include "vex.h"
#include "robot_config.h"
#include <cmath>

using namespace vex;

static inline double norm360(double a) {
    a = std::fmod(a, 360.0);
    if (a < 0.0) a += 360.0;
    return a;
}

inline void initSensors() {
    inertial_sensor.calibrate();
    while (inertial_sensor.isCalibrating()) wait(20, msec);

    inertial_sensor.setHeading(0, deg);
    inertial_sensor.setRotation(0, deg);

    verticalRot.resetPosition();
    horizontalRot.resetPosition();
}

inline double headingDeg() {
    return norm360(360.0 - inertial_sensor.heading(deg));
}

inline double rotationDeg() {
    return -inertial_sensor.rotation(deg);
}

inline double yawRateDps() {
    return -inertial_sensor.gyroRate(zaxis, dps);
}

inline double verticalDeg()   { return verticalRot.position(deg); }
inline double horizontalDeg() { return horizontalRot.position(deg); }

inline double ax_g() { return  inertial_sensor.acceleration(xaxis); }
inline double ay_g() { return -inertial_sensor.acceleration(yaxis); } //was negative
inline double az_g() { return -inertial_sensor.acceleration(zaxis); } //was negative

#endif
