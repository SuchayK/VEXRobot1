#include "vex.h"

#include "autons.h"

#include "drive.h"
#include "odometry.h"

namespace Auton {

  namespace {

    Routine g_selected = NEAR_SIDE;

    // Deploy, score the preload, retreat to a legal position.
    void nearSide() {
      Drive::turnTo(-45);
      wings.set(true);
      wait(300, msec);
      Drive::forward(1.0);
      Drive::back(1.0);
      wings.set(false);
      wait(300, msec);
      Drive::turnTo(135);
      Drive::forward(2.0);
      Drive::back(2.0);
    }

    void farSide() {
      Drive::forward(2.0);
      Drive::turnTo(-90);
      wings.set(true);
      wait(300, msec);
      Drive::forward(1.5);
      Drive::back(1.0);
      wings.set(false);
      wait(300, msec);
      Drive::turnTo(90);
      Drive::forward(1.0);
    }

    // 60 second skills run. Longer, so it leans on odometry rather than on
    // dead-reckoned turns stacking up.
    void skills() {
      for (int cycle = 0; cycle < 3; cycle++) {
        wings.set(true);
        Drive::forward(2.5);
        wings.set(false);
        Drive::back(2.0);
        Drive::turnTo(90);
      }
      Drive::liftTo(1.5);
    }

  }  // namespace

  const char* name(Routine r) {
    switch (r) {
      case NEAR_SIDE: return "Near side";
      case FAR_SIDE:  return "Far side";
      case SKILLS:    return "Skills";
      default:        return "None";
    }
  }

  void run(Routine r) {
    Odom::reset();
    switch (r) {
      case NEAR_SIDE: nearSide(); break;
      case FAR_SIDE:  farSide();  break;
      case SKILLS:    skills();   break;
      default: break;
    }
    Drive::stop();
  }

  Routine selected() { return g_selected; }

  void cycleSelection() {
    g_selected = static_cast<Routine>((static_cast<int>(g_selected) + 1) % ROUTINE_COUNT);
    drawSelector();
  }

  void drawSelector() {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Auton: %s", name(g_selected));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Tap screen to change");
  }

}  // namespace Auton
