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

// inline ez::Piston intakePiston('B');
inline ez::Piston mogoclamp('H');


inline void set_lb(int input) {
  ladybrown.move(input);
}

inline ez::PID lbPID{0.45, 0, 1.5, 0, "ladybrown"};

inline void lb_wait() {
  while (lbPID.exit_condition({ladybrown}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

//vars

inline bool isColorSortEnabled = false; // CHANGE AT EVERY MATCH SCHEDULE
inline bool isRedTeam = false; 

inline void selectRedTeam() {
    isRedTeam = true;
    
}

inline void selectBlueTeam() {
    isRedTeam = false; 
}

inline int intake_speed_high = 0;
inline int intake_speed_low = 0;
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');