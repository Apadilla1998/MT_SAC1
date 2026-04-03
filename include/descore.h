#ifndef DESCORE_H
#define DESCORE_H

// Initialize the lever arm zero position
void initLeverArm();

// Update lever arm target
// deployed = true  -> move to 120 deg
// deployed = false -> move back to 0 deg
void updateLeverArm(bool deployed);

// Print lever angle to controller screen
void printArmAngleControllerUpdate();

#endif