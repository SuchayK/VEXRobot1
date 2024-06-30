#include "vex.h"
using namespace vex;

competition Competition;

void moveForward(double x) {
  right_drive.spinFor(fwd, x, turns, false);
  left_drive.spinFor(fwd, x, turns);
}

void moveForward(double x, bool z) {
  right_drive.spinFor(fwd, x, turns, false);
  left_drive.spinFor(fwd, x, turns, z);
}

void moveBack(double x) {
  right_drive.spinFor(reverse, x, turns, false);
  left_drive.spinFor(reverse, x, turns);
}

void moveBack(double x, bool z) {
  right_drive.spinFor(reverse, x, turns, false);
  left_drive.spinFor(reverse, x, turns, z);
}

void turnLeft(double x) {
  left_drive.spinFor(reverse, x, turns, false);
  right_drive.spinFor(fwd, x, turns);
}

void turnLeft(double x, bool z) {
  left_drive.spinFor(reverse, x, turns, false);
  right_drive.spinFor(fwd, x, turns, z);
}

void turnRight(double x) {
  left_drive.spinFor(fwd, x, turns, false);
  right_drive.spinFor(reverse, x, turns);
}

void turnRight(double x, bool z) {
  left_drive.spinFor(fwd, x, turns, false);
  right_drive.spinFor(reverse, x, turns, z);
}

void rightDegrees(double x) {
  left_drive.spinFor(fwd, x, degrees, false);
  right_drive.spinFor(reverse, x, degrees);
}

void leftDegrees(double x) {
  left_drive.spinFor(reverse, x, degrees, false);
  right_drive.spinFor(fwd, x, degrees);
}

void moveForwardPID(double targetDistance) {
  double kP = 0.5;
  double error = targetDistance;
  double motorPower;
  
  while (fabs(error) > 0.1) {
    error = targetDistance - left_drive.position(turns);
    motorPower = error * kP;
    left_drive.spin(fwd, motorPower, pct);
    right_drive.spin(fwd, motorPower, pct);
    wait(20, msec);
  }
  left_drive.stop();
  right_drive.stop();
}

void pre_auton(void) {
  vexcodeInit();

  right_drive.setStopping(brake);
  left_drive.setStopping(brake);
  right_drive.setVelocity(40, pct);
  left_drive.setVelocity(40, pct);
}

void autonomous(void) {
  moveForward(4.376, false);
  wait(2, sec);
  moveBack(3.891, false);
  wait(2, sec);
  turnRight(3.42);
  wait(2, sec);
  turnLeft(3.42);
  rightDegrees(85.1);
  leftDegrees(85.1);
  moveForwardPID(10.0);
}