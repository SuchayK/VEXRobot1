using namespace vex;

extern brain Brain;
extern motor leftMotor1;
extern motor leftMotor2;
extern motor leftMotor3;
extern motor rightMotor1;
extern motor rightMotor2;
extern motor rightMotor3;
extern motor otherMotor;
extern motor otherMotor1;
extern controller Controller1;
extern motor_group left_drive;
extern motor_group right_drive;
extern digital_out wings;
extern digital_out intake;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
