#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
motor BackLeft = motor(PORT5, ratio18_1, true);
motor BackRight = motor(PORT16, ratio18_1, false);
motor FrontLeft = motor(PORT9, ratio18_1, true);
motor FrontRight = motor(PORT19, ratio18_1, false);
motor MiddleLeft = motor(PORT10, ratio18_1, true);
motor MiddleRight = motor(PORT15, ratio18_1, false);
// motor BackLeft = motor(PORT2, ratio18_1, true);
// motor BackRight = motor(PORT14, ratio18_1, false);
// motor FrontLeft = motor(PORT7, ratio18_1, true);
// motor FrontRight = motor(PORT19, ratio18_1, false);
controller Controller = controller(primary);
controller Controller2 = controller(partner);
motor_group LeftDriveSmart = motor_group(BackLeft, FrontLeft); 
motor_group RightDriveSmart = motor_group(BackRight, FrontRight);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 320, 130, mm, 1);
motor Roller = motor(PORT12, ratio18_1, true);
motor Moby = motor(PORT19, ratio18_1, false);
motor Wheel = motor(PORT18, ratio18_1, true);
digital_out Clamp1 = digital_out(Brain.ThreeWirePort.A);
digital_out Clamp2 = digital_out(Brain.ThreeWirePort.G);
digital_out Lift1 = digital_out(Brain.ThreeWirePort.C);
digital_out Lift2 = digital_out(Brain.ThreeWirePort.B);
digital_out Lift3 = digital_out(Brain.ThreeWirePort.F);
inertial Inert = inertial(PORT13);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}