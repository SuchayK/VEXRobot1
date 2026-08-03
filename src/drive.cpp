#include "vex.h"

#include "drive.h"

#include <cmath>

#include "odometry.h"

namespace Drive {

  const Gains DISTANCE = {0.6, 0.04, 0.3};
  const Gains TURN     = {0.3, 0.02, 0.005};
  const Gains LIFT     = {0.6, 0.04, 0.3};

  const double HEADING_KC = 0.5;

  namespace {

    constexpr int    TICK_MS       = 20;
    constexpr double DIST_EPSILON  = 0.1;    // wheel turns
    constexpr double TURN_EPSILON  = 1.0;    // degrees
    constexpr double MAX_POWER     = 100.0;  // percent
    constexpr double INTEGRAL_CAP  = 50.0;
    constexpr int    TIMEOUT_MS    = 4000;

    double clamp(double v, double lo, double hi) {
      return v < lo ? lo : (v > hi ? hi : v);
    }

    // Shared closed-loop straight move. `sign` is +1 forward, -1 reverse.
    void straight(double targetTurns, int sign) {
      const double startHeading = imu.rotation(degrees);

      left_drive.resetPosition();
      right_drive.resetPosition();

      double error = targetTurns;
      double previousError = 0;
      double integral = 0;
      int elapsed = 0;

      while (std::fabs(error) > DIST_EPSILON && elapsed < TIMEOUT_MS) {
        const double travelled = std::fabs(left_drive.position(turns));
        error = targetTurns - travelled;

        // Only accumulate near the target. Winding up across a long move made
        // the original overshoot every time.
        if (std::fabs(error) < 1.0) integral += error;
        integral = clamp(integral, -INTEGRAL_CAP, INTEGRAL_CAP);

        const double derivative = error - previousError;
        previousError = error;

        double power = DISTANCE.kP * error + DISTANCE.kI * integral + DISTANCE.kD * derivative;
        power = clamp(power, -MAX_POWER, MAX_POWER);

        // Heading correction. Splitting it differentially steers without
        // changing forward speed.
        const double headingError = startHeading - imu.rotation(degrees);
        const double correction = HEADING_KC * headingError;

        left_drive.spin(sign > 0 ? fwd : reverse, power + correction, pct);
        right_drive.spin(sign > 0 ? fwd : reverse, power - correction, pct);

        Odom::update();
        wait(TICK_MS, msec);
        elapsed += TICK_MS;
      }

      stop();
      Odom::update();
    }

  }  // namespace

  void init() {
    left_drive.setStopping(brake);
    right_drive.setStopping(brake);
    left_drive.setVelocity(40, pct);
    right_drive.setVelocity(40, pct);
    lift.setStopping(hold);
  }

  void forward(double turnsTarget) { straight(turnsTarget, +1); }
  void back(double turnsTarget)    { straight(turnsTarget, -1); }

  void turnBy(double targetDegrees) {
    imu.resetRotation();

    double error = targetDegrees;
    double previousError = 0;
    double integral = 0;
    int elapsed = 0;

    while (std::fabs(error) > TURN_EPSILON && elapsed < TIMEOUT_MS) {
      error = targetDegrees - imu.rotation(degrees);

      if (std::fabs(error) < 15.0) integral += error;
      integral = clamp(integral, -INTEGRAL_CAP, INTEGRAL_CAP);

      const double derivative = error - previousError;
      previousError = error;

      double power = TURN.kP * error + TURN.kI * integral + TURN.kD * derivative;
      power = clamp(power, -MAX_POWER, MAX_POWER);

      left_drive.spin(fwd, power, pct);
      right_drive.spin(fwd, -power, pct);

      Odom::update();
      wait(TICK_MS, msec);
      elapsed += TICK_MS;
    }

    stop();
    Odom::update();
  }

  void liftTo(double targetTurns) {
    lift.resetPosition();

    double error = targetTurns;
    double previousError = 0;
    double integral = 0;
    int elapsed = 0;

    while (std::fabs(error) > DIST_EPSILON && elapsed < TIMEOUT_MS) {
      error = targetTurns - lift.position(turns);

      if (std::fabs(error) < 1.0) integral += error;
      integral = clamp(integral, -INTEGRAL_CAP, INTEGRAL_CAP);

      const double derivative = error - previousError;
      previousError = error;

      double power = LIFT.kP * error + LIFT.kI * integral + LIFT.kD * derivative;
      lift.spin(fwd, clamp(power, -MAX_POWER, MAX_POWER), pct);

      wait(TICK_MS, msec);
      elapsed += TICK_MS;
    }

    lift.stop();
  }

  void forwardOpen(double t, bool blocking) {
    right_drive.spinFor(fwd, t, turns, false);
    left_drive.spinFor(fwd, t, turns, blocking);
    Odom::update();
  }

  void backOpen(double t, bool blocking) {
    right_drive.spinFor(reverse, t, turns, false);
    left_drive.spinFor(reverse, t, turns, blocking);
    Odom::update();
  }

  void turnLeftOpen(double t, bool blocking) {
    left_drive.spinFor(reverse, t, turns, false);
    right_drive.spinFor(fwd, t, turns, blocking);
    Odom::update();
  }

  void turnRightOpen(double t, bool blocking) {
    left_drive.spinFor(fwd, t, turns, false);
    right_drive.spinFor(reverse, t, turns, blocking);
    Odom::update();
  }

  void stop() {
    left_drive.stop();
    right_drive.stop();
  }

}  // namespace Drive
