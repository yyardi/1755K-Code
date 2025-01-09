#ifndef _SUBSYSTEMS_HPP_
#define _SUBSYSTEMS_HPP_

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"
#include "pros/optical.hpp"
#include <atomic>

extern std::atomic<bool> isRedTeam; // Atomic for thread safety

void selectRedTeam();
void selectBlueTeam();

extern pros::Controller controller;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern lemlib::Chassis chassis;
extern pros::Motor intakeLow;
extern pros::Motor intakeHigh;
extern pros::Optical colorsort;
extern pros::Motor ladybrown;
extern pros::ADIDigitalOut doinker;
extern pros::ADIDigitalOut mogoclamp;

void initializeSubsystems();


#endif  // _SUBSYSTEMS_HPP_
