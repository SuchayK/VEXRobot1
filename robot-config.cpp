#include "vex.h"

using namespace vex;


// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftMotor1 = motor(PORT10, ratio6_1, true);
motor leftMotor2 = motor(PORT20, ratio6_1, true);
motor leftMotor3 = motor(PORT18, ratio6_1, true);
motor rightMotor1 = motor(PORT11, ratio6_1, false);
motor rightMotor2 = motor(PORT2, ratio6_1, false);
motor rightMotor3 = motor(PORT3, ratio6_1, false);
controller Controller1 = controller(primary);
// VEXcode generated functions
motor_group left_drive= motor_group(leftMotor1, leftMotor2,leftMotor3);
motor_group  right_drive= motor_group(rightMotor1, rightMotor2,rightMotor3);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}

