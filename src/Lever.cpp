#include "Lever.h"
#include "robot_config.h"
#include "utils.h"
#include "subsystems.h"
#include <cmath>

using namespace vex;

// Lever arm positions using LeverArmPot
static constexpr double kLeverOriginDeg   = 1.2;
static constexpr double kLeverDeployedDeg = 248.0;

// Normal control tuning
static constexpr double kLeverKpNormal      = 0.4;
static constexpr double kLeverKdNormal      = 0.20;
static constexpr double kLeverMaxPctNormal  = 30.0;

// Stronger tuning when wings are extended
static constexpr double kLeverKpWings       = 1.0;
static constexpr double kLeverKdWings       = 0.3;
static constexpr double kLeverMaxPctWings   = 65.0;

static constexpr double kLeverDeadbandDeg   = 1.0;
static constexpr double kLeverMinPct        = 8.0;

// Slower behavior near the low/home position
static constexpr double kLowZoneTopDeg        = 20.0;
static constexpr double kLeverLowMaxPctNormal = 22.0;
static constexpr double kLeverLowMaxPctWings  = 35.0;
static constexpr double kLeverLowMinPct       = 5.0;

// When far from target, run at nearly constant velocity.
// Once inside this zone, switch to PD slowdown.
static constexpr double kLeverSlowZoneDeg   = 18.0;

// Hold briefly after releasing L2
static constexpr int kReleaseHoldMs = 500;

// Reverse-intake oscillation to help push balls out
static constexpr double kReverseOscLowDeg   = 1.2;
static constexpr double kReverseOscHighDeg  = 10.0;
static constexpr int    kReverseOscPeriodMs = 50;

// Separate speed just for reverse oscillation
static constexpr double kReverseOscMinPct   = 65.0;
static constexpr double kReverseOscMaxPct   = 100.0;

// State
static bool  gPrevDeployed      = false;
static bool  gReleaseHoldActive = false;
static timer gReleaseTimer;

static bool  gPrevReverseActive = false;
static timer gReverseOscTimer;

static double leverAngleDeg() {
    return LeverArmPot.angle(deg);
}

static double leverKp() {
    return wings.isExtended() ? kLeverKpWings : kLeverKpNormal;
}

static double leverKd() {
    return wings.isExtended() ? kLeverKdWings : kLeverKdNormal;
}

static double leverMaxPct(bool lowZone) {
    if (wings.isExtended()) {
        return lowZone ? kLeverLowMaxPctWings : kLeverMaxPctWings;
    }
    return lowZone ? kLeverLowMaxPctNormal : kLeverMaxPctNormal;
}

static double reverseOscillationTargetDeg() {
    const double tMs = std::fmod(
        static_cast<double>(gReverseOscTimer.time(msec)),
        static_cast<double>(kReverseOscPeriodMs)
    );

    const double halfPeriodMs = kReverseOscPeriodMs * 0.5;

    // Triangle wave: low -> high -> low
    double blend;
    if (tMs <= halfPeriodMs) {
        blend = tMs / halfPeriodMs;
    } else {
        blend = 2.0 - (tMs / halfPeriodMs);
    }

    blend = clampD(blend, 0.0, 1.0);

    return kReverseOscLowDeg +
           (kReverseOscHighDeg - kReverseOscLowDeg) * blend;
}

static bool isInLowZone(double degValue) {
    return degValue <= kLowZoneTopDeg;
}

// Signed current motor velocity in pct
static double leverVelocityPctSigned() {
    double v = LeverArm.velocity(pct);

    if (LeverArm.direction() == reverse) {
        v = -v;
    }

    return v;
}

// Position -> target velocity controller
// Far away: constant cruise speed
// Near target: PD slowdown using measured motor velocity
static void moveLeverTo_PDVelocity(double targetDeg,
                                   double minPctOverride = -1.0,
                                   double maxPctOverride = -1.0) {
    const double currentDeg = leverAngleDeg();
    const double error      = targetDeg - currentDeg;

    if (std::fabs(error) <= kLeverDeadbandDeg) {
        LeverArm.stop(hold);
        return;
    }

    const bool lowZoneMove = isInLowZone(currentDeg) || isInLowZone(targetDeg);

    double minPct = lowZoneMove ? kLeverLowMinPct : kLeverMinPct;
    double maxPct = leverMaxPct(lowZoneMove);

    if (minPctOverride >= 0.0) minPct = minPctOverride;
    if (maxPctOverride >= 0.0) maxPct = maxPctOverride;

    const double currentVelPct = leverVelocityPctSigned();
    double targetVelPct = 0.0;

    // Far from target: try to maintain a nearly constant velocity
    if (std::fabs(error) > kLeverSlowZoneDeg) {
        targetVelPct = (error > 0.0) ? maxPct : -maxPct;
    }
    // Near target: PD control for smooth slowdown
    else {
        targetVelPct = (leverKp() * error) - (leverKd() * currentVelPct);
        targetVelPct = clampD(targetVelPct, -maxPct, maxPct);

        if (std::fabs(targetVelPct) < minPct) {
            targetVelPct = (targetVelPct >= 0.0) ? minPct : -minPct;
        }
    }

    LeverArm.setVelocity(std::fabs(targetVelPct), pct);

    if (targetVelPct > 0.0) {
        LeverArm.spin(fwd);
    } else {
        LeverArm.spin(reverse);
    }
}

void updateLeverArm(bool deployed) {
    const bool reverseActive = Controller1.ButtonR2.pressing();

    // Detect L2 release and begin hold
    if (gPrevDeployed && !deployed) {
        gReleaseHoldActive = true;
        gReleaseTimer.reset();
    }
    gPrevDeployed = deployed;

    // Reset oscillation timer when reverse first starts
    if (reverseActive && !gPrevReverseActive) {
        gReverseOscTimer.reset();
    }

    // Priority 1: L2 held -> go up
    if (deployed) {
        gReleaseHoldActive = false;
        moveLeverTo_PDVelocity(kLeverDeployedDeg);
        gPrevReverseActive = reverseActive;
        return;
    }

    // Priority 2: after releasing L2, hold exactly where it is for a short time
    if (gReleaseHoldActive && gReleaseTimer.time(msec) < kReleaseHoldMs) {
        LeverArm.stop(hold);
        gPrevReverseActive = reverseActive;
        return;
    }

    // Done holding
    if (gReleaseHoldActive && gReleaseTimer.time(msec) >= kReleaseHoldMs) {
        gReleaseHoldActive = false;
    }

    // Priority 3: while reversing intake, oscillate
    if (reverseActive) {
        const double targetDeg = reverseOscillationTargetDeg();
        moveLeverTo_PDVelocity(targetDeg, kReverseOscMinPct, kReverseOscMaxPct);
        gPrevReverseActive = true;
        return;
    }

    // Priority 4: return back down to home
    moveLeverTo_PDVelocity(kLeverOriginDeg);
    gPrevReverseActive = false;
}

void initLeverArm() {
    LeverArm.setStopping(hold);
    gPrevDeployed      = false;
    gReleaseHoldActive = false;
    gReleaseTimer.reset();

    gPrevReverseActive = false;
    gReverseOscTimer.reset();
}

void printArmAngleControllerUpdate() {
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("ARM: %.1f deg", leverAngleDeg());
}