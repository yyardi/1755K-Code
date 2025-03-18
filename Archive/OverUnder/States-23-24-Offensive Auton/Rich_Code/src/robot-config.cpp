#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LB = motor(PORT18, ratio6_1, true);
motor LM = motor(PORT19, ratio6_1, true);
motor LF = motor(PORT20, ratio6_1, true);
motor RB = motor(PORT13, ratio6_1, false);
motor RM = motor(PORT12, ratio6_1, false);
motor RF = motor(PORT11, ratio6_1, false);
inertial Inertial1 = inertial(PORT5);


controller Controller1 = controller(primary);
motor Intake = motor(PORT16, ratio6_1, false);
motor Cata = motor(PORT8, ratio36_1, false);
digital_out Wings = digital_out(Brain.ThreeWirePort.H);
digital_out FourBar = digital_out(Brain.ThreeWirePort.G);
digital_out Park = digital_out(Brain.ThreeWirePort.F);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}