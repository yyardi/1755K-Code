#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/motor_group.hpp"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

//SETUP
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({-11,-12,-7}, pros::MotorGearset::blue); //need ports
pros::MotorGroup right_motors({1,2,3}, pros::MotorGearset::blue); 
pros::Motor intake(16, pros::MotorGearset::blue); //check everything here
pros::ADIDigitalOut clamp ('C', LOW); //port
bool isClamp = false;
bool clampLatch = false;    

lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              8.1,
                              lemlib::Omniwheel::NEW_275, 
                              300, // drivetrain rpm
                              3
);

pros::Imu imu(10); // need port (technically we never use this)

pros::adi::Encoder vertical_encoder('A', 'B');
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(0, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              00, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


/*
// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
*/

lemlib::ControllerSettings angular_controller(0, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);



void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    		pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    		pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "ADI Encoder: %i", vertical_encoder.get_value());
			//pros::lcd::register_btn1_cb(on_center_button);
			pros::delay(20);
            
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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

ASSET(example_txt);
ASSET(path_txt);

void autonomous() {
    chassis.setPose(0, 0, 0);
    // chassis.follow(path_txt, 15, 2000); //need to get rid of the oscillation
    chassis.moveToPoint(10, 10, 4000);
    //chassis.moveToPoint(0, 20, 4000, {.forwards = false}, true);
    //chassis.turnToHeading(-90, 2000);
}
 
void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, -1 * rightX);

        if (controller.get_digital(DIGITAL_R1)) {
            intake.move_velocity(100); // This is 100 because it's a 100rpm motor
        }
        else if (controller.get_digital(DIGITAL_R2)) {
            intake.move_velocity(-100);
        }
        else {
            intake.move_velocity(0);
        }

        if (isClamp){
            clamp.set_value(HIGH);
        } 
        else {
            clamp.set_value(LOW);
        }
        if (controller.get_digital(DIGITAL_L2)) {
            if (!clampLatch) {
                isClamp = !isClamp;
                clampLatch = true;
            } 
        } 
        else {
            clampLatch = false;
        }



        // delay to save resources
        pros::delay(25);
    }
}