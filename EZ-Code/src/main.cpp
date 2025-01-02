#include "main.h"
#include "okapi/api.hpp"
#include "api.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-18, -19, -20},     // Left Chassis Ports (negative port will reverse it!)
    {8, 9, 10},  // Right Chassis Ports (negative port will reverse it!)

    17,      // IMU Port
    2.75,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM

bool isClamp = false;
bool clampLatch = false;


int currentPositionIndex = 0;
bool lastCycleButtonState = false;

const float Kp = 0.75;
const float Ki = 0.01;
const float Kd = 0.2;

const int tolerance = 50;
const int max_output = 50;

const int lbDown = 0;
const int lbMid = 420;
const int lbScore = 2000;
const int positions[] = {lbDown, lbMid, lbScore};



void ladyBrownAngle(int target) {
    int error = 0; 
    int last_error = 0;
    int integral = 0;
    int derivative = 0;

    while(true) {
        int currentPosition = ladybrown.get_position();
        error = target - currentPosition;

        if (std::abs(error) <= tolerance) {
            ladybrown.move_velocity(0);
            break;
        }

        integral += error;
        derivative = error - last_error;
        last_error = error;

        int output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        output = std::clamp(output, -max_output, max_output);

        ladybrown.move_velocity(output);
        pros::delay(20);
    }
}




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(1.4);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)



  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      Auton("Aggressive Auton\n\nBlue - Side", blue_negative_auton),
      Auton("Aggressive Auton\n\nRed - Side", red_negative_auton),
      Auton("Aggressive Auton\n\nRed + Side", red_positive_auton),
      Auton("Aggressive Auton\n\nBlue + Side", blue_positive_auton),
      Auton("Skills Auton\n\nRed Ally Stake Start Pos", skills_auton),
      Auton("Example Drive\n\nDrive forward and come back.", drive_example),
      Auton("Example Turn\n\nTurn 3 times.", turn_example),
      Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
      Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
      Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
      Auton("Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining),
      Auton("Combine all 3 movements", combining_movements),
      Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
      
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */




void opcontrol() {
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;
  ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  ladybrown.tare_position();
  chassis.drive_brake_set(driver_preference_brake);
  
  doinker.set(false);
  mogoclamp.set(false);

  while (true) {


    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) {
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      if (master.get_digital_new_press(DIGITAL_X))
        chassis.pid_tuner_toggle();

      // Trigger the selected autonomous routine
      if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
        autonomous();
        chassis.drive_brake_set(driver_preference_brake);
      }

      chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    }

    //chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    //chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    
    if (master.get_digital(DIGITAL_R1)) {
      intakeLow.move(127);
      intakeHigh.move(127);
    } 
    else if (master.get_digital(DIGITAL_R2)) {
      intakeLow.move(-127);
      intakeHigh.move(-127);
    } 
    else {
      intakeLow.move(0);
      intakeHigh.move(0);
    }

    mogoclamp.button_toggle(master.get_digital(DIGITAL_L2)); 
    doinker.button_toggle(master.get_digital(DIGITAL_L1));


    if (master.get_digital(DIGITAL_DOWN)) {
        currentPositionIndex = (currentPositionIndex + 1) % 3;
        ladyBrownAngle(positions[currentPositionIndex]);
    }

    if (master.get_digital(DIGITAL_UP)) {
        currentPositionIndex = 0;
        ladyBrownAngle(positions[0]);
    }

    // bool currentCycleButtonState = master.get_digital(DIGITAL_DOWN);
    // if (currentCycleButtonState && !lastCycleButtonState) {
    //   const int positions[] = {lbDown, lbMid, lbScore};
    //   currentPositionIndex = (currentPositionIndex + 1) % 3;
    //   ladyBrownAngle(positions[currentPositionIndex]);
    // }
    // lastCycleButtonState = currentCycleButtonState;






    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
