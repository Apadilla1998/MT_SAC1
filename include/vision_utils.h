#ifndef VISION_UTILS_H
#define VISION_UTILS_H

#include "vex.h"
#include "robot_config.h"
#include "drive.h"
#include "utils.h"
#include <cmath>

static inline bool visionAlignOnlyToCenterId(
    uint32_t sigId,
    int timeoutMs = 1500,
    double kP = 0.35,
    double kD = 0.00,
    int deadbandPx = 5,
    double maxTurnPct = 12,
    int lostFramesToFail = 8,
    double centerX = 158.0,
    double offsetX = 0.0
) {
    vex::timer t; t.reset();

    const int dtMs = 20;
    const double dt = dtMs / 1000.0;

    double prevErr = 0.0;
    double lastTurn = 0.0;
    int stableMs = 0;
    int lost = 0;

    const double slowBandPx = 70.0;   // bigger = gentler near center
    const double minTurnPct = 2.5;    // tiny stiction help
    const double maxStepPct = 1.5;    // slew limit per loop

    while (t.time(vex::msec) < timeoutMs) {
        VisionSensor.takeSnapshot(sigId);

        if (VisionSensor.objectCount <= 0) {
            lost++;
            stableMs = 0;
            lastTurn = 0.0;

            stopDrive(vex::brake);

            if (lost >= lostFramesToFail) {
                return false;
            }

            vex::wait(dtMs, vex::msec);
            continue;
        }

        lost = 0;
        vex::vision::object obj = VisionSensor.largestObject;

        const double targetX = centerX + offsetX;
        double err = (double)obj.centerX - targetX;

        if (std::fabs(err) <= deadbandPx) {
            stableMs += dtMs;
            stopDrive(vex::brake);

            if (stableMs >= 140) {
                return true;
            }

            vex::wait(dtMs, vex::msec);
            continue;
        }

        stableMs = 0;

        double derr = (err - prevErr) / dt;
        prevErr = err;

        double turn = kP * err + kD * derr;
        turn = clampD(turn, -maxTurnPct, +maxTurnPct);

        double scale = std::fabs(err) / slowBandPx; 
        scale = clampD(scale, 0.0, 1.0);
        turn *= scale;

        if (std::fabs(turn) < minTurnPct) {
            turn = (err > 0.0) ? +minTurnPct : -minTurnPct;
        }

        turn = clampD(turn, lastTurn - maxStepPct, lastTurn + maxStepPct);
        lastTurn = turn;

        tankDrive(-turn, turn);

        vex::wait(dtMs, vex::msec);
    }

    stopDrive(vex::brake);
    return false;
}

#endif