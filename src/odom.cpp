#include "odom.h"
#include "robot_config.h"
#include "utils.h"
#include "sensors.h"
#include "vex.h"
#include <cmath>

using namespace vex;

Pose robotPose{0.0, 0.0, 0.0};

struct OdomState {
    double prevS_m = 0.0;
    double prevRotDeg = 0.0;
    double theta = 0.0;
};

static OdomState s;

static inline double degToMeters(double deg) {
    return (deg * config::TRACKING_WHEEL_CIRCUMFERENCE_M) / 360.0;
}

void resetOdometry() {
    robotPose = {0.0, 0.0, 0.0};

    inertial_sensor.setHeading(0, deg);
    inertial_sensor.setRotation(0, deg);

    verticalRot.resetPosition();

    s.prevS_m = degToMeters(verticalDeg());
    s.prevRotDeg = rotationDeg();
    s.theta = degToRad(s.prevRotDeg);

    robotPose.theta = s.theta;
}

static int odomLoop() {
    s.prevS_m = degToMeters(verticalDeg());
    s.prevRotDeg = rotationDeg();
    s.theta = degToRad(s.prevRotDeg);
    robotPose.theta = s.theta;

    while (true) {
        const double s_m = degToMeters(verticalDeg());
        const double rotDeg = rotationDeg();

        const double dS = s_m - s.prevS_m;
        const double dRotDeg = rotDeg - s.prevRotDeg;
        const double dT = degToRad(dRotDeg);
        const double avgT = s.theta + 0.5 * dT;

        robotPose.x += dS * std::sin(avgT);
        robotPose.y += dS * std::cos(avgT);

        s.theta += dT;
        robotPose.theta = s.theta;

        s.prevS_m = s_m;
        s.prevRotDeg = rotDeg;

        wait(10, msec);
    }

    return 0;
}

int odomTaskFn() {
    return odomLoop();
}