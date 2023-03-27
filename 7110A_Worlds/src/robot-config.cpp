#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller = controller();
// encoder leftEncoder = encoder(Brain.ThreeWirePort.C);
// encoder rightEncoder = encoder(Brain.ThreeWirePort.A);
motor BackLeft = motor(PORT1, ratio6_1, true);
motor BackRight = motor(PORT4, ratio6_1, false);
motor FrontLeft = motor(PORT3, ratio6_1, true);
motor FrontRight = motor(PORT5, ratio6_1, false);
motor MiddleLeft = motor(PORT2, ratio6_1, false);
motor MiddleRight = motor(PORT6, ratio6_1, true);
motor catapult = motor(PORT7, ratio36_1,false);
inertial Inertial = inertial(PORT20);  
inertial Inertial2 = inertial(PORT19);
motor_group leftDrive = motor_group(BackLeft, FrontLeft, MiddleLeft);
motor_group rightDrive = motor_group(BackRight, FrontRight, MiddleRight);
motor intake = motor(PORT11, ratio6_1, true);
digital_out expansion = digital_out(Brain.ThreeWirePort.A);
digital_out cataBoost = digital_out(Brain.ThreeWirePort.B);
rotation cataSense = rotation(PORT9);
digital_out blocker = digital_out(Brain.ThreeWirePort.C);
distance intakeSense = distance(PORT18);



// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}