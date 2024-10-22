/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Intake               motor_group   5, 6            
// Clamp                digital_out   A               
// LF                   motor         1               
// RF                   motor         2               
// LB                   motor         3               
// RB                   motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
motor_group lDrive = motor_group(LB, LF);
motor_group rDrive = motor_group(RB, RF);

drivetrain MyDrive = drivetrain(lDrive, rDrive, 10.2101761, 12.5, 8, inches, 0.6); //make sure the second param (circumference) is right

bool isClamp = false;
bool clampLatch = false;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Clamp.set(false);
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
  //Default is RIGHT
  double turnconst = 1.95;
  //Intake facing wall, clamp towards field
  //BLOCK 1
  MyDrive.setTimeout(300, msec);
  MyDrive.setDriveVelocity(40, pct); //we only got 15 sec so need as fast as possible
  MyDrive.setTurnVelocity(30, pct); //modifiable depending on trial and error performance
  MyDrive.driveFor(reverse, 22.2, inches); //check direction
  MyDrive.turnFor(right, 30/turnconst, degrees); //the clamp should face the Mogo
  
  //BLOCK 2
  MyDrive.driveFor(reverse, 7, inches);
  Clamp.set(true);
  Intake.spin(forward, 100, pct); //Check Intake direction here and make sure its dropping the ring into the Mogo
  wait(200, msec);
  Intake.stop();
  
  //BLOCK 3
  MyDrive.turnFor(right, 70/turnconst, degrees); //The intake should face the ring stack
  MyDrive.driveFor(forward, 24, inches);
  Intake.spin(forward, 100, pct); //Check Intake direction here and make sure its dropping the LOWER ring into the Mogo
  wait(200, msec);
  Intake.stop();

  //DONE

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
    Clamp.set(false);
    MyDrive.setDriveVelocity(100, pct);
    
    
    int leftY = Controller1.Axis3.position(pct);
    int rightX = Controller1.Axis1.position(pct);

    lDrive.spin(vex::directionType::fwd, leftY + rightX, pct);
    rDrive.spin(vex::directionType::fwd, leftY - rightX, pct);

    if (isClamp){
      Clamp.set(true);
    } 
    else {
      Clamp.set(false);
    }
    if (Controller1.ButtonL2.pressing()) {
      if (!clampLatch) {
        isClamp = !isClamp;
        clampLatch = true;
      } 
    } 
    else {
        clampLatch = false;
    }

    if (Controller1.ButtonR2.pressing()){ //Make sure intake and outtake are binded correctly
      Intake.spin(reverse, 95,pct); //R2 should bring in the rings
    } else if (Controller1.ButtonR1.pressing()){
      Intake.spin(fwd, 95, pct); //R1 should spit out the rings
    } else{
      Intake.stop();
    }

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
