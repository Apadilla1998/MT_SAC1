#include "drive.h"
#include "robot_config.h"
#include "utils.h"

using namespace vex;

static inline void normalizeArcade(double& leftPct, double& rightPct) {
    const double maxMag = std::max(std::fabs(leftPct), std::fabs(rightPct));
    if (maxMag > 100.0) {
        const double scale = 100.0 / maxMag;
        leftPct *= scale;
        rightPct *= scale;
    }
}

void tankDrive(double leftPct, double rightPct) {
    LeftMotorGroup.spin(fwd, clampPct(leftPct), percent);
    RightMotorGroup.spin(fwd, clampPct(rightPct), percent);
}

void arcadeDrive(double fwdPct, double turnPct) {
    double leftPct  = fwdPct + turnPct;
    double rightPct = fwdPct - turnPct;

    normalizeArcade(leftPct, rightPct);
    tankDrive(leftPct, rightPct);
}

void stopDrive(brakeType mode) {
    LeftMotorGroup.stop(mode);
    RightMotorGroup.stop(mode);
}