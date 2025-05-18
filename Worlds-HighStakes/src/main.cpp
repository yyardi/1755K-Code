#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-8, -9, 10},     // Left Chassis Ports (negative port will reverse it!)
    {-15, 16, 17},  // Right Chassis Ports (negative port will reverse it!)

    13,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)




void intake_task() {
  pros::delay(2000);  // Set EZ-Template calibrate before this function starts running
  colorsort.set_led_pwm(100);
  bool isColorSortHappening = false;

  uint32_t intakeStartTime = 0, intake2StartTime = 0;
  uint32_t intakeJamStart = 0, intake2JamStart = 0;
  bool intakeRecentlyStarted = false, intake2RecentlyStarted = false;
  bool intakeJammed = false, intake2Jammed = false;

  const uint32_t antiJamDelay = 500; // Time before anti-jam can activate after starting
  const uint32_t highJamThreshold = 400;  // Time before intake high is considered jammed
  const uint32_t lowJamThreshold = 300;   // Lower threshold for intake low (more sensitive)


  while (true) {
    int threshold = 200; //check with proximity values printed
    if (isRedTeam != 2) {
      
      int hue = colorsort.get_hue();
      if (colorsort.get_proximity() > threshold) {
        if (hue > 180 && hue < 240 && (isRedTeam == 1)) { //blue is 240, red is 0, but our hooks are purple which is ~300
          isColorSortHappening = true; //Update boolean 
          pros::delay(110);
          intake.move(0);
          pros::delay(470);
          intake.move(0);
          printf("Hue: %d\n", hue);
          printf("Proximity: %d\n", colorsort.get_proximity());
        }
        else if (hue < 50 && (isRedTeam == 0)) { //blue is 240, red is 0
          isColorSortHappening = true; //Update boolean 
          pros::delay(110); 
          intake.move(0);
          pros::delay(470);
          intake.move(0);
          printf("Hue: %d\n", hue);
          printf("Proximity: %d\n", colorsort.get_proximity());
        }
      }
      else {
        isColorSortHappening = false; //Update boolean 
      }
    }

    uint32_t currentTime = pros::millis();



    // Detect if intake was just started
    if (intake_speed_high > 0 && !intakeRecentlyStarted) {
        intakeRecentlyStarted = true;
        intakeStartTime = pros::millis();
    }

    // Detect if intake2 was just started
    if (intake_speed_low > 0 && !intake2RecentlyStarted) {
        intake2RecentlyStarted = true;
        intake2StartTime = pros::millis();
    }

    // --- Anti-Jam Logic with Duration Check ---
    if (antijamOn) { 
      // Get Ladybrown Position
      int ladybrownPos = ladybrown_sensor.get_position();
      bool isLadybrownLoaded = (ladybrownPos > 200);
  
      // intake Jam Detection (Disabled When Ladybrown is Up)
      if (!isColorSortHappening && !isLadybrownLoaded && currentTime - intakeStartTime > antiJamDelay) {
          if (intake_speed_high > 0 && intake.get_actual_velocity() < intake_speed_high * 0.15) {
              if (!intakeJammed) {
                  intakeJamStart = currentTime;
                  intakeJammed = true;
              }
              else if (currentTime - intakeJamStart > highJamThreshold) {
                  intake.move(-127); // Reverse
                  pros::delay(500);     
                  intake.move(intake_speed_high); // Resume normal speed
                  intakeJammed = false; 
              }
          }
          else {
              intakeJammed = false; 
          }
      }
  
      // intake2 Jam Detection (Still Works Normally)
      if (!isColorSortHappening && currentTime - intake2StartTime > antiJamDelay) {
          if (intake_speed_low > 0 && intake2.get_actual_velocity() < intake_speed_low * 0.15) {
              if (!intake2Jammed) {
                  intake2JamStart = currentTime;
                  intake2Jammed = true;
              }
              else if (currentTime - intake2JamStart > lowJamThreshold) {
                  intake2.move(-127); // Reverse
                  pros::delay(500);    
                  intake2.move(intake_speed_low); // Resume normal speed
                  intake2Jammed = false; 
              }
          }
          else {
              intake2Jammed = false; 
          }
      }
    }  
    intake.move(intake_speed_high);
    intake2.move(intake_speed_low);

    // Reset flags when intake is turned off
    if (intake_speed_high == 0) {
      intakeRecentlyStarted = false;
      intakeJammed = false;
    }
    if (intake_speed_low == 0) {
        intake2RecentlyStarted = false;
        intake2Jammed = false;
    }
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task INTAKE_TASK(intake_task);

void lb_task() {
  pros::delay(2000);  // Set EZ-Template calibrate before this function starts running
  while (true) {
    set_LB(lbPID.compute(ladybrown_sensor.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task LB_TASK(lb_task);


    
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
  mogoclamp.set(false);
  ladybrown_sensor.reset_position();
  lbPID.exit_condition_set(80, 50, 300, 150, 500, 500);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Red Positive", redPositive},
      {"Blue Positive", bluePositive},
      {"Red Negative", redNegative},
      {"Blue Negative", blueNegative},
      {"Blue 2 Ring", blueCarried},
      {"Red 2 Ring", redCarried},
      {"Color Test", colorTest},
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
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
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  mogoclamp.set(false);
  doinkerR.set(false);
  doinkerL.set(false);

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
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
  // lv_image();
  // ez::as::shutdown(); //ez template green turns off and team image comes on
  // LEDmanager.flow(0x6414e3, 0x9828fa); //gradient between purple/blue ish
  // strand3.setColor(0x00FFFF); // strand stays on one color
  // strand4.flash(0xFF0000);    // flashes a color
  // strand5.pulse(0xFF0000);    // sends a pulse down the strand repeatedly


  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  ladybrown1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lbPID.target_set(0);
  isRedTeam = 2; //TURN OFF COLOR SORT FOR DRIVER 
  // antijamOn = false; //TURN OFF ANTI JAM FOR DRIVER
  
  doinkerR.set(false);
  doinkerL.set(false);
  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();
    

    //chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade


    if (master.get_digital(DIGITAL_R1)) {
        intake_speed_high=127;
        intake_speed_low=127;
    } 
    else if (master.get_digital(DIGITAL_R2)) {
        intake_speed_high=-127;
        intake_speed_low=-127;
    } 
    else {
        intake_speed_high=0;
        intake_speed_low=0;
    }

    mogoclamp.button_toggle(master.get_digital(DIGITAL_L2)); 
    doinkerR.button_toggle(master.get_digital(DIGITAL_L1));
    doinkerL.button_toggle(master.get_digital(DIGITAL_LEFT));

    if (master.get_digital(DIGITAL_DOWN)) { //reset LB with Down
      lbPID.target_set(0);
    }

    if (master.get_digital(DIGITAL_B)) { //Descore Angle
      lbPID.target_set(15000);

    }

    if (master.get_digital(DIGITAL_RIGHT)) { //First load stage LB With Right
      lbPID.target_set(4000);
    }

    if (master.get_digital(DIGITAL_UP)) { //Main scoring angle 
      
      lbPID.target_set(20000);
    }

    if (master.get_digital(DIGITAL_Y)) { //antitip goal on Y
      lbPID.target_set(24500);
    } 


    
    // if (abs(ladybrown_sensor.get_position() - 3850) < 500) {
    //   if (master.get_digital(DIGITAL_R1)) {
    //     intake_speed_high = 80;
    //     intake_speed_low = 127;
    //   } 
    // }


    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
