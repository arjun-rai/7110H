#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller = controller();
motor BackLeft = motor(PORT10, ratio6_1, true);
motor BackRight = motor(PORT1, ratio6_1, false);
motor FrontLeft = motor(PORT8, ratio6_1, true);
motor FrontRight = motor(PORT2, ratio6_1, false);
motor MiddleLeft = motor(PORT9, ratio6_1, true);
motor MiddleRight = motor(PORT3, ratio6_1, false);
motor GearboxRight = motor(PORT4, ratio18_1,false);
motor GearboxLeft = motor(PORT7, ratio18_1, true);
rotation parallelEncoder = rotation(PORT4);
rotation perpendicularEncoder = rotation(PORT21);
inertial Inertial = inertial(PORT5);
motor_group leftDrive = motor_group(BackLeft, FrontLeft, MiddleLeft, GearboxLeft);
motor_group rightDrive = motor_group(BackRight, FrontRight, MiddleRight, GearboxRight);
digital_out wings = digital_out(Brain.ThreeWirePort.A);
digital_out pto = digital_out(Brain.ThreeWirePort.C);
digital_out elevation = digital_out(Brain.ThreeWirePort.B);
motor intake = motor(PORT6, ratio18_1, true);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}