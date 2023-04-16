#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller = controller();
rotation fEncoder = rotation(PORT13);
rotation hEncoder = rotation(PORT16);
motor BackLeft = motor(PORT1, ratio6_1, false);
motor BackRight = motor(PORT12, ratio6_1, true);
motor FrontLeft = motor(PORT3, ratio6_1, false);
motor FrontRight = motor(PORT19, ratio6_1, true);
motor MiddleLeft = motor(PORT2, ratio6_1, true);
motor MiddleRight = motor(PORT6, ratio6_1, false);
motor catapult = motor(PORT7, ratio36_1,false);
inertial Inertial = inertial(PORT14);  
motor_group leftDrive = motor_group(BackLeft, FrontLeft, MiddleLeft);
motor_group rightDrive = motor_group(BackRight, FrontRight, MiddleRight);
motor intake = motor(PORT11, ratio6_1, true);
digital_out expansion = digital_out(Brain.ThreeWirePort.B);
digital_out cataBoost = digital_out(Brain.ThreeWirePort.A);
digital_out cataBoost2 = digital_out(Brain.ThreeWirePort.D);
digital_out cataReduce = digital_out(Brain.ThreeWirePort.C);
rotation cataSense = rotation(PORT9);
digital_out intakeLifter = digital_out(Brain.ThreeWirePort.H);
distance intakeSense = distance(PORT21);
// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}