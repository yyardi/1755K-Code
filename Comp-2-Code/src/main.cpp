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
pros::MotorGroup left_motors({10,9,8}, pros::MotorGearset::blue); //need ports
pros::MotorGroup right_motors({20,19,18}, pros::MotorGearset::blue); //find which ones are reversed
pros::Motor intake(11, pros::MotorGearset::blue); //check everything here
pros::ADIDigitalOut clamp ('C', LOW); //port
bool isClamp = false;
bool clampLatch = false;

lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10, // need inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              300, // drivetrain rpm
                              2 // horizontal drift is 2 (for now)
);

pros::Imu imu(10); // need port (technically we never use this)

pros::adi::Encoder vertical_encoder('A', 'B');
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
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
	
			//pros::lcd::register_btn1_cb(on_center_button);
			pros::lcd::print(0, "ADI Encoder: %i", vertical_encoder.get_value());
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
void autonomous() {}

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
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(-1 * rightX, -1 * leftY);

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