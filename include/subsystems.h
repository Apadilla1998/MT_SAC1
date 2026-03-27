#pragma once
#include "robot_config.h"

// ============================================================
//  Alliance
// ============================================================
enum class Alliance { RED, BLUE };
extern Alliance myAlliance;

// ============================================================
//  Pneumatic subsystems
// ============================================================
class Wings {
public:
    void toggle();
    void set(bool s);
    bool isExtended() const { return state; }
private:
    bool state = false;
};

class BallLoader {
public:
    void toggles();
    void sets(bool s);
    bool isExtended() const { return state; }
private:
    bool state = false;
};

extern Wings     wings;
extern BallLoader ballLoader;

// ============================================================
//  Sorter
// ============================================================
extern volatile bool g_sorterEnabled;

void setSorterEnabled(bool enabled);

// Background task — register with vex::task in main.cpp
int intakeTaskFn();

// ============================================================
//  Intake
// ============================================================
void runIntake    (double speedPct);
void reverseIntake(double speedPct);
void stopIntake   ();

// ============================================================
//  Descore arm (DescoreMotor)
// ============================================================
void moveArmRight(double speedPct);
void moveArmLeft (double speedPct);
void stopArm     ();

// ============================================================
//  Autonomous helpers
// ============================================================
void runIntakeAuto    (double speedPct);
void reverseIntakeAuto(double speedPct);
void stopIntakeAuto   ();