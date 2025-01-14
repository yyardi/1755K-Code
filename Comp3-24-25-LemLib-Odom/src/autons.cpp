#include "autons.hpp"
#include "liblvgl/llemu.h"
#include "main.h"
#include "subsystems.hpp"


// get a path used for pure pursuit
// in the static folder 
ASSET(example_txt); // '.' replaced with "_" to make c++ happy 
void example_drive(){
    selectBlueTeam();
    chassis.moveToPoint(0, 10, 4000);
    
}

void blue_negative_auton() {
    selectBlueTeam(); // Set team color for sorting
    // ... rest of the routine
}

void red_negative_auton() {
    selectRedTeam(); // Set team color for sorting
    // ... rest of the routine
}

void blue_positive_auton() {
    selectBlueTeam(); // Set team color for sorting
    // ... rest of the routine
}

void red_positive_auton() {
    selectRedTeam(); // Set team color for sorting
    // ... rest of the routine
}

void skills_auton() {
    selectRedTeam(); //red always in skills
    // ... rest of the routine
}

void auton_example() {
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}

// Initialize autonomous selection
void AutonSelector::init() {
    currentAuton = AutonRoutine::NONE;
    pros::lcd::clear();
    showingCoords = false;
    displayAutonSelection();
}

// Update display or selection logic
void AutonSelector::update() {
    if (showingCoords) {
        displayCoordinates();
    } else {
        displayAutonSelection();
    }

    // Handle screen input for changing routines
    if (LCD_BTN_RIGHT) {
        currentAuton = static_cast<AutonRoutine>((static_cast<int>(currentAuton) + 1) % 6);
    }
    if (LCD_BTN_LEFT) {
        currentAuton = static_cast<AutonRoutine>((static_cast<int>(currentAuton) - 1 + 6) % 6);
    }
    
}

// Run the selected autonomous routine
void AutonSelector::runSelectedAuton() {
    switch (currentAuton) {
        case AutonRoutine::BLUE_NEGATIVE: blue_negative_auton(); break;
        case AutonRoutine::RED_NEGATIVE: red_negative_auton(); break;
        case AutonRoutine::RED_POSITIVE: red_positive_auton(); break;
        case AutonRoutine::BLUE_POSITIVE: blue_positive_auton(); break;
        case AutonRoutine::SKILLS: skills_auton(); break;
        case AutonRoutine::AUTON_EXAMPLE: auton_example(); break;
        case AutonRoutine::NONE: default: break;
    }
}

// Toggle display mode
void AutonSelector::toggleDisplay() {
    showingCoords = !showingCoords;
    pros::lcd::clear();
}

// Display available routines
void AutonSelector::displayAutonSelection() {
    pros::lcd::print(0, "Auton: %s", getAutonName());
}

// Display robot coordinates
void AutonSelector::displayCoordinates() {
    auto pose = chassis.getPose();
    pros::lcd::print(0, "Auton: %s", getAutonName());
    pros::lcd::print(0, "X: %.2f", pose.x);
    pros::lcd::print(1, "Y: %.2f", pose.y);
    pros::lcd::print(2, "Theta: %.2f", pose.theta);
}

// Get auton routine name
const char* AutonSelector::getAutonName() {
    switch (currentAuton) {
        case AutonRoutine::BLUE_NEGATIVE: return "Blue Negative";
        case AutonRoutine::RED_NEGATIVE: return "Red Negative";
        case AutonRoutine::RED_POSITIVE: return "Red Positive";
        case AutonRoutine::BLUE_POSITIVE: return "Blue Positive";
        case AutonRoutine::SKILLS: return "Skills";
        case AutonRoutine::AUTON_EXAMPLE: return "Autons Example";
        case AutonRoutine::NONE: default: return "None";
    }
}