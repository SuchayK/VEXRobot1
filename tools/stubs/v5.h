// Minimal stand-in for the VEX V5 SDK headers, used only by tools/syntax-check.
//
// The real headers ship with VEXcode and cannot be redistributed, so this
// declares just enough of the API surface for a host compiler to type-check the
// project: every device class, enum and free function the code actually touches.
//
// This verifies declarations, types, overloads and syntax. It does NOT verify
// runtime behaviour, and it is not a substitute for building in VEXcode.

#pragma once

#include <cstdint>
#include <cstdio>

namespace vex {

enum directionType { fwd, reverse };
enum velocityUnits { pct, rpm, dps };
enum rotationUnits { degrees, turns, raw };
enum timeUnits { sec, msec };
enum brakeType { coast, brake, hold };
enum gearSetting { ratio36_1, ratio18_1, ratio6_1 };
enum controllerType { primary, partner };
enum axisType { xaxis, yaxis, zaxis };

typedef int32_t port;
const port PORT1 = 1, PORT2 = 2, PORT3 = 3, PORT6 = 6, PORT7 = 7;
const port PORT11 = 11, PORT12 = 12, PORT13 = 13;

inline void wait(double, timeUnits) {}

struct triport {
  struct pin {};
  pin A, B, C, D, E, F, G, H;
};

class motor {
 public:
  motor(port, gearSetting = ratio18_1, bool reversed = false) {}
  void spin(directionType, double, velocityUnits) {}
  void spin(directionType) {}
  void spinFor(directionType, double, rotationUnits, bool blocking = true) {}
  void stop() {}
  void setStopping(brakeType) {}
  void setVelocity(double, velocityUnits) {}
  void resetPosition() {}
  double position(rotationUnits) { return 0; }
};

class motor_group {
 public:
  motor_group(motor&, motor&, motor&) {}
  void spin(directionType, double, velocityUnits) {}
  void spinFor(directionType, double, rotationUnits, bool blocking = true) {}
  void stop() {}
  void setStopping(brakeType) {}
  void setVelocity(double, velocityUnits) {}
  void resetPosition() {}
  double position(rotationUnits) { return 0; }
};

class encoder {
 public:
  encoder(triport::pin) {}
  void resetPosition() {}
  double position(rotationUnits) { return 0; }
};

class digital_out {
 public:
  digital_out(triport::pin) {}
  void set(bool) {}
};

class inertial {
 public:
  inertial(port) {}
  void calibrate() {}
  bool isCalibrating() { return false; }
  void resetRotation() {}
  double rotation(rotationUnits) { return 0; }
  double heading(rotationUnits) { return 0; }
};

class brain {
 public:
  triport ThreeWirePort;
  class lcd {
   public:
    void clearScreen() {}
    void setCursor(int, int) {}
    void print(const char*, ...) {}
    void pressed(void (*)()) {}
  } Screen;
};

class controller {
 public:
  controller(controllerType = primary) {}
  class axis {
   public:
    int position() { return 0; }
  } Axis1, Axis2, Axis3, Axis4;
  class button {
   public:
    bool pressing() { return false; }
  } ButtonL1, ButtonL2, ButtonR1, ButtonR2, ButtonA, ButtonB, ButtonX, ButtonY;
};

class competition {
 public:
  void autonomous(void (*)()) {}
  void drivercontrol(void (*)()) {}
};

}  // namespace vex
