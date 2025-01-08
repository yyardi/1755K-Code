/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

motor_group lDrive = motor_group(LB, LM, LF);
motor_group rDrive = motor_group(RB, RM, RF);

drivetrain MyDrive = drivetrain(lDrive, rDrive, 7.38274274, 13, 12, inches, 0.6);

bool auto_started = false;
int current_auton_selection = 0; // Auton vars

bool isWingsOut = false; // Wings Pneumatics 
bool wingsLatch = false;

bool isFourBar = false; //Intake Ext Pneumatics
bool fourBarLatch = false;

bool isCata = false; //Cata toggle
bool cataLatch = false;

bool isIntake = false; //Intake toggle
bool intakeLatch = false;

bool isPark = false;
bool parkLatch = false;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Wings.set(false);
  FourBar.set(false);
  Park.set(false);
  task::sleep(100);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  Park.set(false);
  MyDrive.setTimeout(700, msec);
  Wings.set(false);
  FourBar.set(false);
  MyDrive.setDriveVelocity(40, pct);
  MyDrive.setTurnVelocity(30, pct);
  Cata.setVelocity(65, pct);

  //Start in the middle of the tile. parallel to the red corner bar. 
  MyDrive.driveFor(-20, inches);
  //Angle correctly for cata launch
  MyDrive.turnFor(left, 52/2.9, degrees);
  //Wings touch and 4Bar on, Cata goes
  Wings.set(true);
  FourBar.set(true);
  Cata.spinFor(fwd, 27, sec);
  Wings.set(false);
  FourBar.set(false);
  //Do a 180 cuz wings side gotta be on the front
  //Then add 30 because we need it to be parallel to the bar with wings at front
  MyDrive.turnFor(left, 120/2.96, degrees);
  //Go under red pole and pop wings, first sweep from the right
  MyDrive.driveFor(-28, inches);
  MyDrive.turnFor(right, 152/2.9, degrees);
  
  MyDrive.driveFor(35, inches);
  MyDrive.driveFor(35, inches);
  MyDrive.driveFor(20, inches);
  MyDrive.turnFor(left, 200/2.9, degrees);
  Wings.set(true);
  //Score in side
  MyDrive.turnFor(left, 54/2.9, degrees);
  MyDrive.turnFor(left, 40/2.9, degrees);
  MyDrive.driveFor(-33, inches);
  //Go out and score from first third
  MyDrive.driveFor(10, inches);
  MyDrive.turnFor(left, 45/2.9, degrees);
  MyDrive.driveFor(-30, inches);
  Wings.set(true);
  MyDrive.driveFor(-20, inches);
  MyDrive.turnFor(right, 115/2.9, degrees);
  MyDrive.driveFor(-26, inches);
  //Go out and sweep one more time 
  Wings.set(false);
  MyDrive.driveFor(5, inches);
  MyDrive.turnFor(right, 26/2.9, degrees);
  MyDrive.driveFor(25, inches);
  Wings.set(true);
  MyDrive.turnFor(left, 26/2.9, degrees);
  MyDrive.driveFor(-30, inches);
  
  MyDrive.driveFor(10, inches);

  
}




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    Park.set(false);
    MyDrive.setDriveVelocity(100, pct);

    
    int leftY = Controller1.Axis3.position(pct);
    int rightX = Controller1.Axis1.position(pct);

    lDrive.spin(vex::directionType::fwd, leftY + rightX, pct);
    rDrive.spin(vex::directionType::fwd, leftY - rightX, pct);

    //Wings
    if (isWingsOut){
      Wings.set(true);
    } 
    else {
      Wings.set(false);
    }
    if (Controller1.ButtonL2.pressing()) {
      if (!wingsLatch) {
        isWingsOut = !isWingsOut;
        wingsLatch = true;
      } 
    } 
    else {
        wingsLatch = false;
    }


    //Park Pneumatics
    if (isPark){
      Park.set(true);
    } 
    else {
      Park.set(false);
    }
    if (Controller1.ButtonX.pressing()) {
      if (!parkLatch) {
        isPark = !isPark;
        parkLatch = true;
      } 
    } 
    else {
        parkLatch = false;
    }


    //Four Bar Pneumatics
    if (isFourBar){
      FourBar.set(true);
    } 
    else {
      FourBar.set(false);
    }
    if (Controller1.ButtonUp.pressing()) {
      if (!fourBarLatch) {
        isFourBar = !isFourBar;
        fourBarLatch = true;
      } 
    } 
    else {
        fourBarLatch = false;
    }

    //Intake Code
    /*if (isIntake){
      if (Controller1.ButtonR1.pressing())
        Intake.spin(fwd, 100, pct);
      else if (Controller1.ButtonR2.pressing()) {
        Intake.spin(reverse, 100, pct);
      }
    } 
    else {
      Intake.stop(brake);
    }
    if (Controller1.ButtonR1.pressing()) {
      if (!intakeLatch) {
        isIntake = !isIntake;
        intakeLatch = true;
      } 
    } 
    else {
        intakeLatch = false;
    }
    */
    if (Controller1.ButtonR2.pressing()){
      Intake.spin(reverse, 95,pct);
    } else if (Controller1.ButtonR1.pressing()){
      Intake.spin(fwd, 95, pct);
    } else{
      Intake.stop();
    }
    //Cata
    if (isCata){
      Cata.spin(fwd, 65, pct);
    } 
    else {
      Cata.stop();
    }
    if (Controller1.ButtonL1.pressing()) {
      if (!cataLatch) {
        isCata = !isCata;
        cataLatch = true;
      } 
    } 
    else {
        cataLatch = false;
    }
  
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
