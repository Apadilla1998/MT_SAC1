#pragma once

// ============================================================
//  Descore arm (DescoreMotor + DescorePot on G)
// ============================================================

// Print the descore arm pot angle to controller line 3.
void printArmAngleControllerUpdate();

// ============================================================
//  Lever Arm (LeverArm motor + LeverArmPot on H)
// ============================================================

// Call once at startup to initialize the arm to hold position.
void initLeverArm();

// Call every loop tick.
//   deployed = true  → move to deployed angle
//   deployed = false → return to stored / origin angle
void updateLeverArm(bool deployed);