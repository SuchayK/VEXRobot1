// main.cpp — competition entry point.
//
// VEXcode `competition` template: pre_auton runs once at startup, then the field
// control system calls autonomous() and usercontrol() at the right times.

#include "vex.h"

#include "autons.h"
#include "drive.h"
#include "odometry.h"
#include "robot-config.h"

using namespace vex;

competition Competition;

namespace {

  // Deadband keeps a resting stick from creeping the drive.
  constexpr int DEADBAND = 5;

  int applyDeadband(int value) {
    return (value > -DEADBAND && value < DEADBAND) ? 0 : value;
  }

  bool g_wingsOut = false;

  void onScreenTap() {
    Auton::cycleSelection();
  }

}  // namespace

void pre_auton(void) {
  vexcodeInit();     // blocks while the inertial sensor calibrates
  Drive::init();
  Odom::reset();

  Brain.Screen.pressed(onScreenTap);
  Auton::drawSelector();
}

void autonomous(void) {
  Auton::run(Auton::selected());
}

void usercontrol(void) {
  while (true) {
    const int leftStick  = applyDeadband(Controller1.Axis3.position());
    const int rightStick = applyDeadband(Controller1.Axis2.position());

    left_drive.spin(fwd, leftStick, pct);
    right_drive.spin(fwd, rightStick, pct);

    // Lift on the left shoulder buttons.
    if (Controller1.ButtonL1.pressing()) {
      lift.spin(fwd, 100, pct);
    } else if (Controller1.ButtonL2.pressing()) {
      lift.spin(reverse, 100, pct);
    } else {
      lift.stop();
    }

    // Wings toggle on a rising edge, so holding the button doesn't chatter the
    // solenoid.
    if (Controller1.ButtonR1.pressing()) {
      if (!g_wingsOut) {
        g_wingsOut = true;
        wings.set(true);
      }
    } else if (Controller1.ButtonR2.pressing()) {
      if (g_wingsOut) {
        g_wingsOut = false;
        wings.set(false);
      }
    }

    Odom::update();

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("x %.1f  y %.1f  h %.1f   ",
                       Odom::x(), Odom::y(), Odom::headingDeg());

    wait(20, msec);
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
