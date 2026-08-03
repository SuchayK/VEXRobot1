// drive.h — motion primitives.
//
// Every autonomous routine is written against this file. Two families:
//
//   Drive::forward / back / turnBy   closed loop, heading-corrected. Use these.
//   Drive::forwardOpen / turnOpen    blocking spinFor calls, no feedback.
//                                    Kept for quick tuning and sanity checks.
//
// The old code had both families named moveForward(double) — two functions,
// same signature, same translation unit, which is a redefinition error. They're
// distinguished by name now.

#pragma once

namespace Drive {

  // PID gains, tuned on the competition robot.
  struct Gains {
    double kP, kI, kD;
  };

  extern const Gains DISTANCE;   // error in wheel turns
  extern const Gains TURN;       // error in degrees
  extern const Gains LIFT;

  // Proportional heading correction applied differentially during a straight
  // move: left = power + KC * error, right = power - KC * error.
  extern const double HEADING_KC;

  void init();

  // Closed loop, holds the heading it started on.
  void forward(double turns);
  void back(double turns);

  // Closed loop on the inertial sensor. Pivots BY `degrees` relative to the
  // current heading — the sensor's rotation is reset at the start of each call.
  // Positive is clockwise. (This was named turnTo, which reads as turning to an
  // absolute field heading; it never did that.)
  void turnBy(double degrees);

  void liftTo(double turns);

  // Open loop. No feedback, no heading correction.
  void forwardOpen(double turns, bool blocking = true);
  void backOpen(double turns, bool blocking = true);
  void turnLeftOpen(double turns, bool blocking = true);
  void turnRightOpen(double turns, bool blocking = true);

  void stop();

}
