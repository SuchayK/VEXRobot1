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

void moveBack(double x){
  right_drive.spinFor(reverse, x,turns,false);
  left_drive.spinFor(reverse, x,turns);
}

void turnLeft(double x){
  left_drive.spinFor(reverse,x,turns,false);
  right_drive.spinFor(fwd,x,turns);

}

void turnRight(double x){
  left_drive.spinFor(fwd,x,turns,false);
  right_drive.spinFor(reverse,x, turns);
} 

