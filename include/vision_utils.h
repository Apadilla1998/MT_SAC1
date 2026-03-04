#ifndef VISION_UTILS_H
#define VISION_UTILS_H

#include "vex.h"
#include "robot_config.h"
#include "drive.h"
#include "utils.h"
#include <cmath>

static inline bool visionTurnToCenterId(
    uint32_t sigId,
    int timeoutMs = 1500,
    double kP = 0.35,
    double kD = 0.10,
    int deadbandPx = 4,
    double maxTurnPct = 35,
    int lostFramesToFail = 10,
    double centerX = 158.0,
    double offsetX = 0.0
) {
    vex::timer t; t.reset();
    double prevErr = 0.0;
    int stableMs = 0;
    int lost = 0;

    const int dtMs = 20;
    const double dt = dtMs / 1000.0;

    while (t.time(vex::msec) < timeoutMs) {
        VisionSensor.takeSnapshot(sigId);

        if (VisionSensor.objectCount <= 0) {
            lost++;
            tankDrive(+10, -10); // slow scan

            if (lost >= lostFramesToFail) {
                stopDrive(vex::brake);
                return false;
            }
            vex::wait(dtMs, vex::msec);
            continue;
        }

        lost = 0;
        vex::vision::object obj = VisionSensor.largestObject;

        double err = (double)obj.centerX - (centerX + offsetX);

        if (std::fabs(err) <= deadbandPx) {
            stableMs += dtMs;
            tankDrive(0, 0);
            if (stableMs >= 120) {
                stopDrive(vex::brake);
                return true;
            }
        } else {
            stableMs = 0;

            double derr = (err - prevErr) / dt;
            prevErr = err;

            double turn = kP * err + kD * derr;
            turn = clampD(turn, -maxTurnPct, +maxTurnPct);

            // stiction floor
            if (std::fabs(turn) < 8.0) turn = (turn > 0) ? 8.0 : -8.0;

            tankDrive(turn, -turn);
        }

        vex::wait(dtMs, vex::msec);
    }

    stopDrive(vex::brake);
    return false;
}

#endif