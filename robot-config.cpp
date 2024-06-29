#include "vex.h"

using namespace vex;


// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftMotor1 = motor(PORT10);
motor leftMotor2 = motor(PORT20);
motor leftMotor3 = motor(PORT18);
motor rightMotor1 = motor(PORT11);
motor rightMotor2 = motor(PORT2);
motor rightMotor3 = motor(PORT3);
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