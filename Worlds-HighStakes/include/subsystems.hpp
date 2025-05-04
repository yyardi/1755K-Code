#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor intake(-19);
inline pros::Motor intake2(14);
inline pros::Motor ladybrown1(-18);



inline ez::Piston doinkerR('F');
inline ez::Piston doinkerL('G');
inline ez::Piston mogoclamp('H');

inline pros::Optical colorsort(12);


inline pros::Rotation ladybrown_sensor(-11);

inline void set_LB(int input) {
    ladybrown1.move(input);
  }
  
  inline ez::PID lbPID{0.02, 0, 0.02, 0, "ladybrown"};
  
  inline void lb_wait() {
    while (lbPID.exit_condition({ladybrown1}, true) == ez::RUNNING) {
      pros::delay(ez::util::DELAY_TIME);
    }
  }


//vars

inline bool antijamOn = false; 

inline int isRedTeam = 1; //CHANGE AT EVERY MATCH

inline void selectRedTeam() {
    isRedTeam = 1;   
}

inline void selectBlueTeam() {
    isRedTeam = 0; 
}

inline void selectSkills() {
    isRedTeam = 2;
}

inline int intake_speed_low = 0;
inline int intake_speed_high = 0;
