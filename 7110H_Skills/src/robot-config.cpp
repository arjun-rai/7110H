#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller = controller();
motor BackLeft = motor(PORT18, ratio6_1, true);
motor BackRight = motor(PORT11, ratio6_1, false);
motor FrontLeft = motor(PORT19, ratio6_1, true);
motor FrontRight = motor(PORT12, ratio6_1, false);
motor MiddleLeft = motor(PORT20, ratio6_1, true);
motor MiddleRight = motor(PORT13, ratio6_1, false);
motor motor1 = motor(PORT15, ratio36_1,true);
motor motor2 = motor(PORT16, ratio36_1, false);
rotation parallelEncoder = rotation(PORT14);
rotation perpendicularEncoder = rotation(PORT17);
inertial Inertial = inertial(PORT9);
motor_group leftDrive = motor_group(BackLeft, FrontLeft, MiddleLeft);
motor_group rightDrive = motor_group(BackRight, FrontRight, MiddleRight);
digital_out wings = digital_out(Brain.ThreeWirePort.A);
digital_out wingsBackLeft = digital_out(Brain.ThreeWirePort.D);
digital_out wingsBackRight = digital_out(Brain.ThreeWirePort.C);
digital_out pto = digital_out(Brain.ThreeWirePort.B);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}