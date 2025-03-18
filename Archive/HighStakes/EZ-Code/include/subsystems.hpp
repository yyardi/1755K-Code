#pragma once

#include "EZ-Template/piston.hpp"
#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor intakeLow(-4);
inline pros::Motor intakeHigh(-5);
inline pros::Motor ladybrown(16);
// inline ez::Piston doinker('A');
inline ez::Piston intakePiston('B');
inline ez::Piston mogoclamp('C');

// inline pros::adi::DigitalIn limit_switch('A');



