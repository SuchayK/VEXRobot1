// robot-config.h — every device on the robot, declared once.
//
// Definitions live in src/robot-config.cpp. Ports here must match the physical
// build; nothing else in the codebase hardcodes a port.

#pragma once

// Reached via vex.h, which includes this file after the V5 SDK headers.
using namespace vex;

extern brain Brain;
extern controller Controller1;

// Drivetrain — three motors per side, 6:1 (600 rpm) cartridges.
extern motor leftMotor1;
extern motor leftMotor2;
extern motor leftMotor3;
extern motor rightMotor1;
extern motor rightMotor2;
extern motor rightMotor3;
extern motor_group left_drive;
extern motor_group right_drive;

// Lift — 18:1 (200 rpm) cartridge for torque.
extern motor lift;

// Pneumatics — both wing pistons on one solenoid via a tee (three-wire H).
extern digital_out wings;

// Odometry: two parallel tracking wheels plus one perpendicular rear wheel.
extern encoder leftEncoder;
extern encoder rightEncoder;
extern encoder backEncoder;

// Heading reference.
//
// The old header declared every device except this one, and the line above it
// was missing its semicolon, so nothing that included it compiled.
extern inertial imu;

void vexcodeInit(void);
