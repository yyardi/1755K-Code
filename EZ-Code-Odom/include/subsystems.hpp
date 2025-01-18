#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/optical.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intakeLow(-11);
inline pros::Motor intakeHigh(-7);
inline pros::Motor ladybrown(4);
// inline ez::Piston doinker('A');
inline pros::Optical colorsort(16);

inline ez::Piston intakePiston('B');
inline ez::Piston mogoclamp('C');


inline void set_lb(int input) {
  ladybrown.move(input);
}

inline ez::PID lbPID{0.45, 0, 0, 0, "ladybrown"};

inline void lb_wait() {
  while (lbPID.exit_condition({ladybrown}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}


// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');