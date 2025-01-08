using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LB;
extern motor LM;
extern motor LF;
extern motor RB;
extern motor RM;
extern motor RF;
extern controller Controller1;
extern motor Intake;
extern motor Cata;
extern digital_out Wings;
extern digital_out FourBar;
extern inertial Inertial1;
extern digital_out Park;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );