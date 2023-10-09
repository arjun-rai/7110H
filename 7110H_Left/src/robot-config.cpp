#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller = controller();
motor BackLeft = motor(PORT13, ratio6_1, true);
motor BackRight = motor(PORT16, ratio6_1, false);
motor FrontLeft = motor(PORT14, ratio6_1, true);
motor FrontRight = motor(PORT17, ratio6_1, false);
motor MiddleLeft = motor(PORT15, ratio6_1, true);
motor MiddleRight = motor(PORT18, ratio6_1, false);
motor catapult = motor(PORT11, ratio36_1,false);
motor intake = motor(PORT12, ratio36_1, false);
inertial Inertial = inertial(PORT20);
motor_group leftDrive = motor_group(BackLeft, FrontLeft, MiddleLeft);
motor_group rightDrive = motor_group(BackRight, FrontRight, MiddleRight);
rotation cataSense = rotation(PORT19);
digital_out intakeLifter = digital_out(Brain.ThreeWirePort.B);
digital_out wings = digital_out(Brain.ThreeWirePort.A);
digital_out blooper = digital_out(Brain.ThreeWirePort.C);
// digital_out auton_grabber = digital_out(Brain.ThreeWirePort.F);



// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}