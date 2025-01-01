#pragma once

#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor intakeLow(-11);
inline pros::Motor intakeHigh(-7);
inline pros::Motor ladybrown(5);
inline ez::Piston doinker('C');
inline ez::Piston mogoclamp('A');

// inline pros::adi::DigitalIn limit_switch('A');