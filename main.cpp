#include "vex.h"
using namespace vex;

competition Competition;

motor lift(PORT1, gearSetting::ratio18_1, false);
digital_out wings(Brain.ThreeWirePort.A);
digital_out left_wing(Brain.ThreeWirePort.B);
digital_out right_wing(Brain.ThreeWirePort.C);

double robotX = 0.0; 
double robotY = 0.0; 
double robotTheta = 0.0; 

const double wheelDiameter = 4.0; 
const double tpr = 360; 


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

void moveForward(double targetDistance) {
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

void moveBack(double targetDistance) {
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

    left_drive.spin(reverse, motorPower, pct);
    right_drive.spin(reverse, motorPower, pct);

    previousError = error;
    wait(20, msec);
  }

  left_drive.stop();
  right_drive.stop();
}

void turnPID(double targetDegrees) {
  double kP = 0.6;
  double kI = 0.04;
  double kD = 0.3;
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
  double kP = 0.6;
  double kI = 0.04;
  double kD = 0.3;
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

void setLeftWing(bool state) {
  wings.left_wing.set(state);
}

void setRightWing(bool state) {
  wings.right_wing.set(state);
}

void updateOdometry() {

  static double lastLeftEncoder = 0.0;
  static double lastRightEncoder = 0.0;
  static double lastBackEncoder = 0.0;

  double currentLeftEncoder = leftEncoder.position(degrees);
  double currentRightEncoder = rightEncoder.position(degrees);
  double currentBackEncoder = backEncoder.position(degrees);

  double deltaLeft = currentLeftEncoder - lastLeftEncoder;
  double deltaRight = currentRightEncoder - lastRightEncoder;
  double deltaBack = currentBackEncoder - lastBackEncoder;

  lastLeftEncoder = currentLeftEncoder;
  lastRightEncoder = currentRightEncoder;
  lastBackEncoder = currentBackEncoder;

  double deltaLeftInches = (deltaLeft / tpr) * wheelDiameter * M_PI;
  double deltaRightInches = (deltaRight / tpr) * wheelDiameter * M_PI;
  double deltaBackInches = (deltaBack / tpr) * wheelDiameter * M_PI;

  double deltaTheta = (deltaRightInches - deltaLeftInches) / wheelBaseWidth;
  double deltaX = (deltaLeftInches + deltaRightInches) / 2.0;
  double deltaY = deltaBackInches - (deltaTheta * backWheelOffset);

  robotTheta += deltaTheta;
  robotX += deltaX * cos(robotTheta) - deltaY * sin(robotTheta);
  robotY += deltaX * sin(robotTheta) + deltaY * cos(robotTheta);

}


void pre_auton(void) {
  vexcodeInit();

  right_drive.setStopping(brake);
  left_drive.setStopping(brake);
  right_drive.setVelocity(40, pct);
  left_drive.setVelocity(40, pct);
}

void autonomous(void) {

  leftDegrees(45);  
  wait(1, sec);

  wings.set(true);  
  wait(1, sec)
  
  moveForward(1.0);  
  wait(1, sec);
  
  moveBack(1.0); 
  wait(1, sec);

  wings.set(false); 
  wait(1, sec)

  rightDegrees(135);  
  wait(1, sec);

  wings.set(true); 
  wait(1, sec)

  moveForward(2.0);  
  wait(1, sec);

  moveBack(2.0); 
  wait(1, sec);

  wings.set(false);  
  wait(1, sec)

  leftDegrees(90);  
  wait(1, sec);

  moveForward(2.0); 
  wait(1, sec);
  
  leftDegrees(90); 
  wait(1, sec);

  wings.set(true); 
  wait(1, sec)
  
  moveForward(0.5);  
  wait(1, sec);
  
  moveBack(1.5);  
  wait(1, sec);

  wings.set(false);
  wait(1, sec)
  
  leftDegrees(90); 
  wait(1, sec);

  moveForward(2.0); 
  wait(1, sec);

  wings.set(true);  
  wait(1, sec)

  wings.set(false); 
  wait(1, sec)

  moveBack(4.0);  
  wait(1, sec);

  leftDegrees(90);
  wait(1, sec);

  moveForward(1.25);  
  wait(1, sec);

  rightDegrees(60); 
  wait(1, sec);

  wings.set(true); 
  wait(1, sec)

  moveForward(2.75); 
  wait(1, sec);

  wings.set(false);
  wait(1, sec);

  moveBack(2.0);
  wait(1, sec);

  rightDegrees(30);
  wait(1, sec);

  moveForward(2.0);
  wait(1, sec);

  rightDegrees(90);
  wait(1, sec);

  wings.set(true);
  wait(1, sec);

  moveForward(1.3);
  wait(1, sec);

  wings.set(false);
  wait(1, sec);

  moveBack(1.0);
  wait(1, sec);

  leftDegrees(180);
  wait(1, sec);

  wings.set(true);
  wait(1, sec);

  moveForward(2.0);
  wait(1, sec);


//   moveForward(1.0);         // Forward 1
//   wings.set(true);          // Set wings to true
//   wait(1, sec);
  
//   leftDegrees(30);          // Left 30
//   wait(1, sec);

//   moveForward(0.6);         // Forward 0.6
//   wait(1, sec);

//   leftDegrees(60);          // Left 60
//   wait(1, sec);

//   moveForward(1.3);         // Forward 1.3
//   wait(1, sec);

//   moveBack(1.0);            // Back 1
//   wait(1, sec);

//   moveForward(1.0);         // Forward 1
//   wait(1, sec);

//   moveBack(1.0);            // Back 1
//   wait(1, sec);

//   leftDegrees(45);          // Left 45
//   wait(1, sec);

//   moveForward(4.3);         // Forward 4.3
//   wait(1, sec);

//   wings.set(false);         // Set wings to false
//   wait(1, sec);

//   leftDegrees(45);          // Left 45
//   wait(1, sec);

//   moveBack(2.0);            // Back 2
//   wait(1, sec);

//   rightDegrees(90);         // Right 90
//   wait(1, sec);

//   wings.set(true);          // Set wings to true
//   wait(1, sec);

//   moveForward(1.4);         // Forward 1.4
//   wait(1, sec);

//   wings.set(false);         // Set wings to false
//   wait(1, sec);

//   leftDegrees(90);          // Left 90
//   wait(1, sec);

//   moveForward(3.0);         // Forward 3
//   wait(1, sec);

//   moveBack(1.0);            // Back 1
//   wait(1, sec);

//   leftDegrees(90);          // Left 90
//   wait(1, sec);

//   moveForward(3.0);         // Forward 3
//   wait(1, sec);

//   rightDegrees(90);         // Right 90
//   wait(1, sec);

//   moveBack(2.0);            // Back 2
//   wait(1, sec);

//   moveForward(2.0);         // Forward 2
//   wait(1, sec);

//   wings.set(true);          // Set wings to true
//   wait(1, sec);

//   moveForward(2.0);         // Forward 2 tiles
//   wait(1, sec);
}

void usercontrol(void) {
  while (true) {

    left_drive.spin(fwd, Controller1.Axis3.position(pct), pct);
    right_drive.spin(fwd, Controller1.Axis2.position(pct), pct);
    wait(20, msec);

    updateOdometry();

    Brain.Screen.print(robotX);
    Brain.Screen.print(robotY);
    Brain.Screen.print(robotTheta);
    
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
