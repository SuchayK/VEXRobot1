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
  double kP = 0.6;
  double kI = 0.04;
  double kD = 0.3;
  double error = targetDistance;
  double previousError = 0;
  double integral = 0;
  double derivative;
  double motorPower;

  left_drive.resetPosition();
  right_drive.resetPosition();

  while (fabs(error) > 0.1) {
    error = targetDistance - left_drive.position(turns);
    integral += error;
    derivative = error - previousError;
    motorPower = (kP * error) + (kI * integral) + (kD * derivative);

    left_drive.spin(fwd, motorPower, pct);
    right_drive.spin(fwd, motorPower, pct);

    previousError = error;
    wait(20, msec);
  }

  left_drive.stop();
  right_drive.stop();
}

void turnPID(double targetDegrees) {
  double kP = 0.3;
  double kI = 0.01;
  double kD = 0;
  double error = targetDegrees;
  double previousError = 0;
  double integral = 0;
  double derivative;
  double motorPower;

  inertial.resetRotation();

  while (fabs(error) > 1) {
    error = targetDegrees - inertial.rotation(degrees);
    integral += error;
    derivative = error - previousError;
    motorPower = (kP * error) + (kI * integral) + (kD * derivative);

    left_drive.spin(fwd, motorPower, pct);
    right_drive.spin(reverse, motorPower, pct);

    previousError = error;
    wait(20, msec);
  }

  left_drive.stop();
  right_drive.stop();
}

void liftToHeight(double targetHeight) {
  double kP = 0.24;
  double kI = 0.01;
  double kD = 0.005;
  double error = targetHeight;
  double previousError = 0;
  double integral = 0;
  double derivative;
  double motorPower;

  lift.resetPosition();

  while (fabs(error) > 0.1) {
    error = targetHeight - lift.position(turns);
    integral += error;
    derivative = error - previousError;
    motorPower = (kP * error) + (kI * integral) + (kD * derivative);

    lift.spin(fwd, motorPower, pct);

    previousError = error;
    wait(20, msec);
  }

  lift.stop();
}


void pre_auton(void) {
  vexcodeInit();

  right_drive.setStopping(brake);
  left_drive.setStopping(brake);
  right_drive.setVelocity(40, pct);
  left_drive.setVelocity(40, pct);
}

void autonomous(void) {
  leftDegrees(45);  // Turn left 45 degrees
  wait(1, sec);
  
  moveForward(1.0);  // Move forward 1 tile
  wait(1, sec);
  
  moveBack(1.0);  // Move backward 1 tile
  wait(1, sec);
  
  rightDegrees(135);  // Turn right 135 degrees
  wait(1, sec);
  
  moveForward(2.0);  // Move forward 2 tiles
  wait(1, sec);

  liftToHeight(5.0);  // Activate lift
  wait(1, sec);
  
  moveBack(2.0);  // Move backward 2 tiles
  wait(1, sec);
  
  leftDegrees(90);  // Turn left 90 degrees
  wait(1, sec);
  
  moveForward(2.0);  // Move forward 2 tiles
  wait(1, sec);
  
  leftDegrees(90);  // Turn left 90 degrees
  wait(1, sec);
  
  moveForward(0.5);  // Move forward 1/2 tile
  wait(1, sec);
  
  moveBack(1.5);  // Move backward 1 1/2 tiles
  wait(1, sec);
  
  leftDegrees(90);  // Turn left 90 degrees
  wait(1, sec);
  
  moveForward(2.0);  // Move forward 2 tiles
  wait(1, sec);
  
  moveBack(4.0);  // Move backward 4 tiles
  wait(1, sec);
  
  leftDegrees(90);  // Turn left 90 degrees
  wait(1, sec);
  
  moveForward(1.25);  // Move forward 1.25 tiles
  wait(1, sec);
  
  rightDegrees(60);  // Turn right 60 degrees
  wait(1, sec);
  
  moveForward(2.75);  // Move forward 2.75 tiles
  wait(1, sec);
}

void usercontrol(void) {
  while (true) {
    left_drive.spin(fwd,Controller1.Axis3.value()+Controller1.Axis1.value()*0.45,percent);
    right_drive.spin(fwd,Controller1.Axis3.value()-Controller1.Axis1.value()*0.45,percent);
    
    if (Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing()){
      lift.spin(fwd);
    } else if (Controller1.ButtonB.pressing()){
      lift.spin(reverse);
    }
    else{
      lift.stop();
    }
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  
  while (true) {
    wait(100, msec);
  }
}
