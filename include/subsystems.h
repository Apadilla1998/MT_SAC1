#ifndef SUBSYSTEMS_H
#define SUBSYSTEMS_H

#include "vex.h"

enum class SortTargetColor {
    OFF,
    RED,
    BLUE
};

enum class Alliance {
    RED,
    BLUE
};

extern SortTargetColor g_sortTargetColor;
extern Alliance myAlliance;

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

class Descore{
public:
    void toggle();
    void set(bool s);
    bool isExtended() const { return state; }
private:
    bool state = false;
};

extern Wings wings;
extern BallLoader ballLoader;
extern Descore descore;

void setSorterEnabled(bool enabled);
void setSortTargetColor(SortTargetColor color);

// add this
void homeColorSort();

void runIntake(double speedPct);
void reverseIntake(double speedPct);
void stopIntake();

void moveArmRight(double speedPct);
void moveArmLeft(double speedPct);
void stopArm();

void runIntakeAuto(double speedPct);
void reverseIntakeAuto(double speedPct);
void stopIntakeAuto();

int intakeTaskFn();

#endif