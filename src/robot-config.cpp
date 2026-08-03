#include "vex.h"

#include "robot-config.h"

using namespace vex;

brain Brain;
controller Controller1 = controller(primary);

// Left side is reversed so positive velocity drives the robot forward on both
// sides.
motor leftMotor1  = motor(PORT1,  ratio6_1, true);
motor leftMotor2  = motor(PORT2,  ratio6_1, true);
motor leftMotor3  = motor(PORT3,  ratio6_1, true);
motor rightMotor1 = motor(PORT11, ratio6_1, false);
motor rightMotor2 = motor(PORT12, ratio6_1, false);
motor rightMotor3 = motor(PORT13, ratio6_1, false);

motor_group left_drive  = motor_group(leftMotor1, leftMotor2, leftMotor3);
motor_group right_drive = motor_group(rightMotor1, rightMotor2, rightMotor3);

motor lift = motor(PORT7, ratio18_1, true);

// Pneumatics. Both wing pistons are plumbed to ONE solenoid through a tee, so
// one digital_out fires them together — which is also the only way the port
// math closes: the three quadrature encoders below each consume a PAIR of
// adjacent three-wire ports (A+B, C+D, E+F), leaving just G and H. The old
// per-side left_wing/right_wing declarations sat on B and D, inside the
// encoder pairs, and nothing ever actuated them; they're gone.
//
// (main.cpp also used to re-declare wings on three-wire port A while this file
// put it on H, so the two definitions disagreed about which solenoid fired.)
digital_out wings = digital_out(Brain.ThreeWirePort.H);

encoder leftEncoder  = encoder(Brain.ThreeWirePort.A);
encoder rightEncoder = encoder(Brain.ThreeWirePort.C);
encoder backEncoder  = encoder(Brain.ThreeWirePort.E);

inertial imu = inertial(PORT6);

void vexcodeInit(void) {
  // The heading loop is only as good as this calibration, and the robot must be
  // still while it runs. Blocking here is deliberate.
  imu.calibrate();
  while (imu.isCalibrating()) {
    wait(50, msec);
  }
}
