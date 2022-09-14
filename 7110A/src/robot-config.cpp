#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller = controller();
encoder leftEncoder = encoder(Brain.ThreeWirePort.C);
encoder rightEncoder = encoder(Brain.ThreeWirePort.A);
motor BackLeft = motor(PORT18, ratio18_1, true);
motor BackRight = motor(PORT16, ratio18_1, false);
motor FrontLeft = motor(PORT9, ratio18_1, true);
motor FrontRight = motor(PORT19, ratio18_1, false);
motor MiddleLeft = motor(PORT10, ratio18_1, true);
motor MiddleRight = motor(PORT15, ratio18_1, false);
motor flywheel = motor(PORT20, ratio6_1,false);
inertial Inertial = inertial(PORT9);
motor_group leftDrive = motor_group(BackLeft, FrontLeft, MiddleLeft);
motor_group rightDrive = motor_group(BackRight, FrontRight, MiddleRight);
motor intake = motor(PORT17, ratio18_1, false);


// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}