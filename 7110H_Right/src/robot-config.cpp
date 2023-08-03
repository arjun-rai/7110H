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
motor cata = motor(PORT11, ratio36_1,false);
motor cata2 = motor(PORT12, ratio36_1,true);
motor_group catapult = motor_group(cata, cata2);
inertial Inertial = inertial(PORT19);
motor_group leftDrive = motor_group(BackLeft, FrontLeft, MiddleLeft);
motor_group rightDrive = motor_group(BackRight, FrontRight, MiddleRight);
rotation cataSense = rotation(PORT20);
digital_out lifter = digital_out(Brain.ThreeWirePort.G);
digital_out auton_grabber = digital_out(Brain.ThreeWirePort.F);
digital_out lifter2 = digital_out(Brain.ThreeWirePort.E);



// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}