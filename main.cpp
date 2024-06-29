/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
//#include "image.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void moveForward(double x){
  right_drive.spinFor(fwd, x,turns,false);
  left_drive.spinFor(fwd, x,turns);
}
void moveForward(double x, bool z){
  right_drive.spinFor(fwd, x,turns,false);
  left_drive.spinFor(fwd, x,turns, z);
}
void moveBack(double x){
  right_drive.spinFor(reverse, x,turns,false);
  left_drive.spinFor(reverse, x,turns);
}
void moveBack(double x, bool z){
  right_drive.spinFor(reverse, x,turns,false);
  left_drive.spinFor(reverse, x,turns, z );
}
void turnLeft(double x){
  left_drive.spinFor(reverse,x,turns,false);
  right_drive.spinFor(fwd,x,turns);

}
void turnLeft(double x, bool z){
  left_drive.spinFor(reverse,x,turns,false);
  right_drive.spinFor(fwd,x,turns, z);
}
void turnRight(double x){
  left_drive.spinFor(fwd,x,turns,false);
  right_drive.spinFor(reverse,x, turns);
} 

void rightDegrees(double x){
  left_drive.spinFor(fwd,x,degrees,false);
  right_drive.spinFor(reverse,x, degrees);
} 

void turnRight(double x, bool z){
  left_drive.spinFor(fwd,x,turns,false);
  right_drive.spinFor(reverse,x, turns, z);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  right_drive.setStopping(brake);
  left_drive.setStopping(brake);
  right_drive.setVelocity(40, pct);
  left_drive.setVelocity(40, pct);  

}

void autonomous(void) {
  
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  
  //offside
  
  moveForward(4.376, false);
  wait(2, sec);
  moveBack(3.891,false);
  wait(2, sec);
  turnRight(3.42);
  wait(2, sec);
  turnLeft(3.42);

  rightDegrees(85.1);

}