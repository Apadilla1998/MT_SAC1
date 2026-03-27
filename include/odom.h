#ifndef ODOM_H
#define ODOM_H

struct Pose {
    double x;
    double y;
    double theta;
};

extern Pose robotPose;

void resetOdometry();
int odomTaskFn();

#endif