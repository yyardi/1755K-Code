#include "main.h"
#include "api.h"


/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_drive_constants_set(20, 0, 100);
  chassis.pid_turn_constants_set(3, 0.05, 20, 15);
  chassis.pid_swing_constants_set(6, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .

void blue_negative_auton() {
    doinker.set(false);
    mogoclamp.set(false);
    ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
    //BLOCK 1 - ally stake
    chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
    chassis.pid_wait_quick();
    chassis.pid_turn_set(90_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-18.25_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_relative_set(-90_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-5_in, DRIVE_SPEED);
    chassis.pid_wait();
    intakeHigh.move(100); //dont need low intake for ally stake
    pros::delay(300);
    intakeHigh.move(0);
    
    //BLOCK 2 - get mogo
    chassis.pid_drive_set(5_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_turn_set(0_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_turn_relative_set(-145_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-30_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-11_in, DRIVE_SPEED*0.7);
    chassis.pid_wait_quick();
    mogoclamp.set(true);
    chassis.pid_drive_set(-3_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    
    //BLOCK 3 - get 3 rings 
    chassis.pid_turn_relative_set(-165_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    intakeLow.move(127);
    intakeHigh.move(100); // maybe change later
    chassis.pid_drive_set(19_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    pros::delay(500);
    chassis.pid_turn_relative_set(25_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(6_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    pros::delay(400);
    

    chassis.pid_drive_set(-10_in, DRIVE_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_turn_relative_set(60_deg, TURN_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_drive_set(13_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    pros::delay(1500);
    intakeLow.move(0);
    intakeHigh.move(0);

    //BLOCK 4 - rush to ladder
    chassis.pid_turn_relative_set(140_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(35_in, DRIVE_SPEED);
    chassis.pid_wait();

    ladybrown.move(127);
    pros::delay(200);
    ladybrown.move(0);


    
    // //BLOCK 4 - doinker on the negative corner 
    // chassis.pid_drive_set(-14_in, DRIVE_SPEED);
    // chassis.pid_wait_quick();
    // chassis.pid_turn_relative_set(165_deg, TURN_SPEED);
    // chassis.pid_wait_quick();
    // chassis.pid_drive_set(60_in, DRIVE_SPEED);
    // chassis.pid_wait_quick();
    // doinker.set(true);
    // chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
    // chassis.pid_wait_quick();
    // doinker.set(false);
    
    // //BLOCK 5 - intake last ring on negative blue corner 
    // chassis.pid_turn_relative_set(-90_deg, TURN_SPEED);
    // chassis.pid_wait_quick();
    // chassis.pid_drive_set(-10_in, DRIVE_SPEED);
    // chassis.pid_wait_quick();
    // intakeLow.move(127);
    // intakeHigh.move(100);
    // chassis.pid_drive_set(12_in, DRIVE_SPEED);
    // chassis.pid_wait_quick();
    // pros::delay(300);
    // chassis.pid_turn_relative_set(-180_deg, TURN_SPEED);
    // chassis.pid_wait_quick();

}


void red_negative_auton() {
    //degrees are flipped i think
    doinker.set(false);
    mogoclamp.set(false);
    ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //BLOCK 1 - ally stake
    chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
    chassis.pid_wait_quick();
    chassis.pid_turn_set(-90_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-18.25_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-5_in, DRIVE_SPEED);
    chassis.pid_wait();
    intakeHigh.move(100); //dont need low intake for ally stake
    pros::delay(300);
    intakeHigh.move(0);
    
    //BLOCK 2 - get mogo
    chassis.pid_drive_set(5_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_turn_set(0_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_turn_relative_set(145_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-30_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-11_in, DRIVE_SPEED*0.7);
    chassis.pid_wait_quick();
    mogoclamp.set(true);
    chassis.pid_drive_set(-3_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    
    //BLOCK 3 - get 3 rings 
    chassis.pid_turn_relative_set(165_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    intakeLow.move(127);
    intakeHigh.move(100);
    chassis.pid_drive_set(19_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    pros::delay(500);
    chassis.pid_turn_relative_set(-25_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(6_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    pros::delay(400);
    

    chassis.pid_drive_set(-10_in, DRIVE_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_turn_relative_set(-60_deg, TURN_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_drive_set(13_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    pros::delay(1500);
    intakeLow.move(0);
    intakeHigh.move(0);

    //BLOCK 4 - rush to ladder
    chassis.pid_turn_relative_set(-140_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(35_in, DRIVE_SPEED);
    chassis.pid_wait();

    ladybrown.move(127);
    pros::delay(200);
    ladybrown.move(0);


    
    // //BLOCK 4 - doinker on the negative corner 
    // chassis.pid_drive_set(-14_in, DRIVE_SPEED);
    // chassis.pid_wait_quick();
    // chassis.pid_turn_relative_set(165_deg, TURN_SPEED);
    // chassis.pid_wait_quick();
    // chassis.pid_drive_set(60_in, DRIVE_SPEED);
    // chassis.pid_wait_quick();
    // doinker.set(true);
    // chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
    // chassis.pid_wait_quick();
    // doinker.set(false);
    
    // //BLOCK 5 - intake last ring on negative blue corner 
    // chassis.pid_turn_relative_set(-90_deg, TURN_SPEED);
    // chassis.pid_wait_quick();
    // chassis.pid_drive_set(-10_in, DRIVE_SPEED);
    // chassis.pid_wait_quick();
    // intakeLow.move(127);
    // intakeHigh.move(100);
    // chassis.pid_drive_set(12_in, DRIVE_SPEED);
    // chassis.pid_wait_quick();
    // pros::delay(300);
    // chassis.pid_turn_relative_set(-180_deg, TURN_SPEED);
    // chassis.pid_wait_quick();



}

void red_positive_auton() {
    ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    doinker.set(false);
    mogoclamp.set(false);

    //BLOCK 1 - Shove Blue Ring to the side
    intakeLow.move(127);
    chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
    chassis.pid_wait_quick();
    chassis.pid_turn_set(-90_deg, TURN_SPEED);
    intakeLow.move(-127);
    chassis.pid_wait_quick();

    //BLOCK 2 - Go around the stack and go to Mogo
    chassis.pid_turn_relative_set(-45_deg, TURN_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_drive_set(-20_in, DRIVE_SPEED);
    chassis.pid_wait();

    chassis.pid_turn_relative_set(-45_deg, TURN_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_drive_set(-10_in, DRIVE_SPEED);
    chassis.pid_wait();

    chassis.pid_turn_relative_set(-30_deg, TURN_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_drive_set(-16.5_in, DRIVE_SPEED);
    chassis.pid_wait();
    mogoclamp.set(true);
    
    //BLOCK 3 - Face Stack and collect rings
    chassis.pid_turn_relative_set(20_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    intakeLow.move(100); //purposefully slower so the preload doesnt get dropped
    chassis.pid_drive_set(10_in, DRIVE_SPEED);
    mogoclamp.set(false); // drop mogo on our side to regrab it
    chassis.pid_drive_set(5_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    intakeLow.move(0);

    chassis.pid_drive_set(-7_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    mogoclamp.set(true); //regrab mogo

    intakeHigh.move(127);
    pros::delay(600); //intake both red rings here onto the mogo
    intakeHigh.move(0);

    //BLOCK 4 - go to corner and use doinker
    chassis.pid_turn_relative_set(-50_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(50_in, DRIVE_SPEED);
    chassis.pid_wait_until(37_in);
    doinker.set(true);
    mogoclamp.set(false); //let go of mogo on our side 
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_relative_set(90_deg, TURN_SPEED); //hit the stack in the corner with doinker at 90 deg, then back
    chassis.pid_wait();
    doinker.set(false);
    chassis.pid_turn_relative_set(-90_deg, TURN_SPEED);
    chassis.pid_drive_set(4_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    intakeLow.move(127); // collect bottom ring just enough to get it on the intake stage 2
    intakeHigh.move(100);
    pros::delay(50);
    intakeLow.move(0);
    intakeHigh.move(0);

    chassis.pid_turn_relative_set(-4_deg, TURN_SPEED); //align with the other mogo
    chassis.pid_wait_quick();
    
    chassis.pid_drive_set(-40_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    mogoclamp.set(true);
    chassis.pid_drive_set(-5_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    intakeHigh.move(127);
    pros::delay(600);
    intakeHigh.move(0);
    chassis.pid_turn_relative_set(180_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-8_in, DRIVE_SPEED);
    chassis.pid_wait();

    ladybrown.move(127);
    pros::delay(200);
    ladybrown.move(0);

}

void blue_positive_auton() {
    ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    doinker.set(false);
    mogoclamp.set(false);

    //BLOCK 1 - Shove Blue Ring to the side
    intakeLow.move(127);
    chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
    chassis.pid_wait_quick();
    chassis.pid_turn_set(90_deg, TURN_SPEED);
    intakeLow.move(-127);
    chassis.pid_wait_quick();

    //BLOCK 2 - Go around the stack and go to Mogo
    chassis.pid_turn_relative_set(45_deg, TURN_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_drive_set(-20_in, DRIVE_SPEED);
    chassis.pid_wait();

    chassis.pid_turn_relative_set(45_deg, TURN_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_drive_set(-10_in, DRIVE_SPEED);
    chassis.pid_wait();

    chassis.pid_turn_relative_set(30_deg, TURN_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_drive_set(-16.5_in, DRIVE_SPEED);
    chassis.pid_wait();
    mogoclamp.set(true);
    
    //BLOCK 3 - Face Stack and collect rings
    chassis.pid_turn_relative_set(-20_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    intakeLow.move(100); //purposefully slower so the preload doesnt get dropped
    chassis.pid_drive_set(10_in, DRIVE_SPEED);
    mogoclamp.set(false); // drop mogo on our side to regrab it
    chassis.pid_drive_set(5_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    intakeLow.move(0);

    chassis.pid_drive_set(-7_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    mogoclamp.set(true); //regrab mogo

    intakeHigh.move(127);
    pros::delay(600); //intake both red rings here onto the mogo
    intakeHigh.move(0);

    //BLOCK 4 - go to corner and use doinker
    chassis.pid_turn_relative_set(50_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(50_in, DRIVE_SPEED);
    chassis.pid_wait_until(37_in);
    doinker.set(true);
    mogoclamp.set(false); //let go of mogo on our side 
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_relative_set(-90_deg, TURN_SPEED); //hit the stack in the corner with doinker at 90 deg, then back
    chassis.pid_wait();
    doinker.set(false);
    chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
    chassis.pid_drive_set(4_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    intakeLow.move(127); // collect bottom ring just enough to get it on the intake stage 2
    intakeHigh.move(100);
    pros::delay(50);
    intakeLow.move(0);
    intakeHigh.move(0);

    chassis.pid_turn_relative_set(4_deg, TURN_SPEED); //align with the other mogo
    chassis.pid_wait_quick();
    
    chassis.pid_drive_set(-40_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    mogoclamp.set(true);
    chassis.pid_drive_set(-5_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    intakeHigh.move(127);
    pros::delay(600);
    intakeHigh.move(0);
    chassis.pid_turn_relative_set(-180_deg, TURN_SPEED);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-8_in, DRIVE_SPEED);
    chassis.pid_wait();

    ladybrown.move(127);
    pros::delay(200);
    ladybrown.move(0);

}

void skills_auton() {
  ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  doinker.set(false);
  mogoclamp.set(false);
  
  

}