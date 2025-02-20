#include "autons.hpp"
#include <sys/select.h>
#include "EZ-Template/drive/drive.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110; // 110
const int TURN_SPEED = 90; //90
const int SWING_SPEED = 110; // 110

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  // chassis.pid_drive_constants_set(23, 1.0, 220.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_drive_constants_forward_set(24.0, 0.05, 220.0);
  chassis.pid_drive_constants_backward_set(16.0, 0.05, 220.0);
  chassis.pid_heading_constants_set(18.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(4.0, 0.05, 15.0, 10.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(1.5, 0.0, 18.0);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(1.5, 0.0, 10.0);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.7);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(12_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.2);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches
  // selectBlueTeam();
  // chassis.pid_odom_set(-4_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // mogoclamp.set(true);
  // intake_speed_high = 127;
  // intake_speed_low = 127;
  // pros::delay(100000);

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
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

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
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

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED, 45);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::RIGHT_SWING, -90_deg, SWING_SPEED, 45);
  chassis.pid_wait_quick();

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

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED},
                        {{0_in, 0_in}, rev, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  intake_speed_high = 127;  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  intake_speed_high = 0; // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{24_in, 24_in, 90_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  // chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
  //                      true);
  // chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

void old_blue_negative_auton() {
    selectBlueTeam();
    // doinker.set(false);
    // intakePiston.set(false);
    //BLOCK 1 - Get Mogo + Preload
    chassis.pid_drive_set(-24.5_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(27_deg, TURN_SPEED);
    chassis.pid_wait();
    
    chassis.pid_drive_set(-21_in, DRIVE_SPEED*0.7);
    chassis.pid_wait_until(-20_in);
    mogoclamp.set(true);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-4_in, DRIVE_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_turn_relative_set(205_deg, TURN_SPEED);
    chassis.pid_wait();
    // BLOCK 2 - get 3 rings 
    intake_speed_high = 106;
    intake_speed_low = 127;
    pros::delay(100);
    chassis.pid_drive_set(12_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_relative_set(32.5_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(10_in, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(800);
    chassis.pid_turn_relative_set(86_deg, TURN_SPEED);
    chassis.pid_wait();
    lbPID.target_set(570);
    chassis.pid_drive_set(13_in, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(700);
    intake_speed_low = 0;
    // intake_speed_high = -10;
    // pros::delay(10);
    // intake_speed_high = 0;
    // chassis.pid_turn_relative_set(-180_deg, TURN_SPEED);
    // chassis.pid_wait();
    // chassis.pid_drive_set(-24_in, DRIVE_SPEED);
    // chassis.pid_wait();
    // mogoclamp.set(false);

    chassis.pid_odom_set({{13_in, -4.68_in, 28_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();
    lbPID.target_set(2550);
    intake_speed_high = 0;
    pros::delay(2000);
    chassis.pid_drive_set(-10_in, DRIVE_SPEED);
    chassis.pid_wait();
    lbPID.target_set(0);

    chassis.pid_odom_set({{1.21_in, -22.67_in, 133.9_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();
    lbPID.target_set(400);
    
}


void old_red_negative_auton() {
    selectRedTeam();
    // doinker.set(false);
    // intakePiston.set(false);
    //BLOCK 1 - Get Mogo + Preload
    chassis.pid_drive_set(-24.5_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-27_deg, TURN_SPEED);
    chassis.pid_wait();
    
    chassis.pid_drive_set(-21_in, DRIVE_SPEED*0.7);
    chassis.pid_wait_until(-20_in);
    mogoclamp.set(true);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-4_in, DRIVE_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_turn_relative_set(-205_deg, TURN_SPEED);
    chassis.pid_wait();
    // BLOCK 2 - get 3 rings 
    intake_speed_high = 106;
    intake_speed_low = 127;
    pros::delay(100);
    chassis.pid_drive_set(12_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_relative_set(-27.5_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(16_in, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(800);
    chassis.pid_turn_relative_set(-102_deg, TURN_SPEED);
    chassis.pid_wait();
    lbPID.target_set(200);
    chassis.pid_drive_set(13_in, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(700);
    intake_speed_low = 0;
    // intake_speed_high = -10;
    // pros::delay(10);
    // intake_speed_high = 0;
    // chassis.pid_turn_relative_set(-180_deg, TURN_SPEED);
    // chassis.pid_wait();
    // chassis.pid_drive_set(-24_in, DRIVE_SPEED);
    // chassis.pid_wait();
    // mogoclamp.set(false);

    chassis.pid_odom_set({{-13_in, -4.68_in, 1_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();
    lbPID.target_set(2550);
    intake_speed_high = 0;
    pros::delay(2000);
    chassis.pid_drive_set(-10_in, DRIVE_SPEED);
    chassis.pid_wait();
    lbPID.target_set(0);

    chassis.pid_odom_set({{-13_in, -52.7_in, -133.9_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();
    lbPID.target_set(400);
}

void old_skills_auton() {
    selectSkills();
    intake_speed_high = 127;
    pros::delay(500);
    intake_speed_high = 0;
    
    chassis.pid_odom_set(14_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-90_deg, TURN_SPEED);
    chassis.pid_wait();
    
    chassis.pid_odom_set(-24_in, DRIVE_SPEED);
    chassis.pid_wait_until(-23_in);
    mogoclamp.set(true);
    //grabs mogo
    chassis.pid_wait();
    chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
    chassis.pid_wait();
    intake_speed_high = 127;
    intake_speed_low = 106;
    chassis.pid_odom_set(24_in, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(200);
    //curve part start
    chassis.pid_odom_set({{{48.05_in, 54.82_in, -4.36_deg}, fwd, DRIVE_SPEED},
                      {{22.22_in, 84.6_in, -47.8_deg}, fwd, DRIVE_SPEED}},
                     true);
    chassis.pid_wait();
    pros::delay(200);
    chassis.pid_turn_relative_set(160_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(25_in, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(200);
    chassis.pid_drive_set(-7_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_relative_set(85_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_odom_set({{50.93_in, -14.06_in, 180_deg}, fwd, 90}, true);
    chassis.pid_wait();
    pros::delay(500);
    chassis.pid_turn_relative_set(125_deg, TURN_SPEED);
    chassis.pid_wait();
    intake_speed_high = -100;
    pros::delay(100);
    intake_speed_high = 0;
    chassis.pid_drive_set(-17_in, DRIVE_SPEED);
    chassis.pid_wait();
    mogoclamp.set(false);
    pros::delay(100);
    chassis.pid_drive_set(20_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_relative_set(145_deg,TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_odom_set({{-13.5_in, 1.83_in, 446.69_deg}, rev, 60}, true); // go to mogo 
    chassis.pid_wait();
    mogoclamp.set(true);  // clamped 2nd mogo
    chassis.pid_wait();
    chassis.pid_turn_set(360, TURN_SPEED);
    chassis.pid_wait();
    intake_speed_high = 127;
    intake_speed_low = 106;
    chassis.pid_drive_set(24_in, DRIVE_SPEED); //drives fwd 1 tile , first ring on mogo 2
    chassis.pid_wait();
    pros::delay(200);
   chassis.pid_odom_set({{{-48.05_in, 54.82_in, 324.73_deg}, fwd, DRIVE_SPEED},
                      {{-22.22_in, 84.6_in, 412.24_deg}, fwd, DRIVE_SPEED}},
                     true); //ring 2
    chassis.pid_wait();
    pros::delay(100);
    chassis.pid_turn_relative_set(-137_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(25_in, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(100);
    chassis.pid_turn_relative_set(84_deg, TURN_SPEED);
    chassis.pid_odom_set({{-44.4_in, -21.5_in, 180_deg}, fwd, 90}, true);  //last 3 rings on mogo 2
    chassis.pid_wait();
    pros::delay(500);
    chassis.pid_turn_relative_set(-125_deg, TURN_SPEED);
    chassis.pid_wait();
    intake_speed_high = -100;
    pros::delay(100);
    intake_speed_high = 0;
    chassis.pid_drive_set(-17_in, DRIVE_SPEED);
    chassis.pid_wait();
    mogoclamp.set(false);
    pros::delay(100);
    chassis.pid_drive_set(20_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_odom_set({{-52.3_in, 70.58_in, -6.88_deg}, fwd, DRIVE_SPEED}, true);  // third mogo pursuit ig idk
    chassis.pid_wait();
    pros::delay(100);
    chassis.pid_odom_set({{-13.22_in, 88.3_in, -129.69_deg}, rev, 80}, true);  // third mogo pursuit ig idk
    chassis.pid_wait();
    pros::delay(100);
    mogoclamp.set(true);

    /*chassis.pid_odom_set(28_in, DRIVE_SPEED); //drives fwd 1 tile
    chassis.pid_wait();
    chassis.pid_turn_relative_set(-88_deg, TURN_SPEED);//turns twards 4nd ring 
    chassis.pid_wait();
    chassis.pid_odom_set(79_in, DRIVE_SPEED); //drives fwd 4 tile
    chassis.pid_wait();
    chassis.pid_odom_set(-10_in, DRIVE_SPEED); //drives back a little 
    chassis.pid_wait();
    chassis.pid_turn_relative_set(-135_deg, TURN_SPEED);//turns twards corner 
    chassis.pid_wait();
    intake_speed_high = -50;
    pros::delay(100);
    intake_speed_high = 0;
    chassis.pid_drive_set(-10_in, DRIVE_SPEED); // puts in corner
    chassis.pid_wait();
    mogoclamp.set(false);
    chassis.pid_drive_set(-12_in, DRIVE_SPEED); // second push  
    chassis.pid_wait();
    chassis.pid_drive_set(8_in, DRIVE_SPEED); // second push  
    chassis.pid_wait();

    //push third mogo in corner
    chassis.pid_odom_set({{-10.84_in, 114.5_in, 15.89_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();

    chassis.pid_odom_set({{-24.16_in, 113.5_in, -77.17_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();

    chassis.pid_odom_set({{-58.42_in, 120.52_in, -74.95_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();

    chassis.pid_odom_set({{-34.3_in, 112.3_in, -71.63_deg}, rev, 60}, true);
    chassis.pid_wait();


    chassis.pid_odom_set({{-20.51_in, 105.65_in, -236.66_deg}, fwd, 60}, true);
    chassis.pid_wait();

    chassis.pid_odom_set({{4.02_in, 115.15_in, -285.93_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();

    chassis.pid_odom_set({{21.46_in, 115.35_in, -267.38_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();


      chassis.pid_odom_set({{56.95_in, 118.97_in, -302.26_deg}, fwd, DRIVE_SPEED}, true); 
      chassis.pid_wait();
*/


    //not ready for LB in Auto yet
    // // lbPID.target_set(200);
    // // chassis.pid_wait();
    // // chassis.pid_drive_set(-15_in, DRIVE_SPEED);
    // // chassis.pid_wait();
    // // chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
    // // chassis.pid_wait();
    // // chassis.pid_drive_set(25.5_in, DRIVE_SPEED);
    // // chassis.pid_wait();
    // // chassis.pid_turn_relative_set(-88_deg, TURN_SPEED);
    // // chassis.pid_wait();
    // // chassis.pid_odom_set({{70.64_in, 53.20_in, 90_deg}, fwd, 50}, 
    // // true);
    // // chassis.pid_wait();

    // chassis.pid_drive_set(-0.3_in, DRIVE_SPEED);
    // chassis.pid_wait();


    // lbPID.target_set(1900);
    // intake_speed_high = 0;
    // pros::delay(2000);


}



//States Negatives
void new_negative_blue() {
  //Starting Pose: Angled to Mogo (backwards)
  selectBlueTeam();

  //Grab Mogo, then usual ring rush with 4 in the mogo

  //Go to corner, and then intake 2 more rings and LB

  //Go to ally stake 
  
  // then touch ladder
}

void new_negative_red() {
  //Starting Pose: Angled to Mogo (backwards)
  // selectRedTeam();
  //Grab Mogo, then usual ring rush with 4 in the mogo
  chassis.pid_drive_set(-23_in, DRIVE_SPEED);
  chassis.pid_wait();
  mogoclamp.set(true);
  
  pros::delay(50);
  intake_speed_high = 127;
  pros::delay(300);
  
  chassis.pid_turn_set(120_deg, TURN_SPEED);
  chassis.pid_wait();
  intake_speed_low = 127;
  chassis.pid_odom_set(23_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(400);

  chassis.pid_odom_set(-6_in, DRIVE_SPEED);
  chassis.pid_wait();


  chassis.pid_turn_set(92_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  pros::delay(450);

  chassis.pid_turn_set(-27_deg, TURN_SPEED);
  chassis.pid_wait();

  intake_speed_low = -127;

  chassis.pid_drive_set(16_in, DRIVE_SPEED);
  chassis.pid_wait();

  intake_speed_low = 127;
  
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  intake_speed_low = 0;



  //Go to corner, and then intake 2 more rings and LB

  

  chassis.pid_odom_set({{11.85_in, 14.85_in, 13_deg}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  chassis.pid_drive_set(2_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  doinker.set(true);
  
  // chassis.pid_turn_relative_set(-80_deg, 127);
  // chassis.pid_wait();



  //Go to ally stake 
  
  // then touch ladder
  intake_speed_high = 0;
  intake_speed_low = 0;

}

void full_goal_negative_blue() {
  //Starting Pose: Angled to Mogo (backwards)
  selectBlueTeam();
  //Grab mogo, then usual ring rush with 4 in the mogo

  //Go to corner, and then intake 2 more rings, 


  // rush to positive corner
}

void full_goal_negative_red() {
  //Starting Pose: Angled to Mogo (backwards)
  selectRedTeam();


}


//States Positives
void goal_rush_positive_blue() {
  //Starting Pose: Angled to Mogo (forwards)
  selectBlueTeam();
  //Rush with doinker to goal 
  //(Back up and clamp properly, then drop preload)

  //get second mogo and then drop the second ring 

  //get corner and drop another 2 rings

  //get ally stake with intake piston

  //rush ladder

}

void goal_rush_positive_red() {
  //Starting Pose: Angled to Mogo (forwards)
  selectBlueTeam();



}

void carry_positive_blue() {
  //Starting Pose: Angled to Mogo (forwards)
  selectBlueTeam();
  //Rush with doinker to goal
  //(Back up and clamp properly, then drop preload)

  //get second mogo and then drop the second ring

  //get ally stake with intake piston

  //get third mogo (negative side) and go forward, then drop a ring from the stack

  //rush ladderx


}

void carry_positive_red() {
  //Starting Pose: Angled to Mogo (forwards)
  selectRedTeam();
}

//Skills
void fiftyone_skills() {
  selectSkills();
}










