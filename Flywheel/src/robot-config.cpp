#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller = controller();
motor flywheel1 = motor(PORT9, ratio6_1, true);
motor flywheel2 = motor(PORT10, ratio6_1, false);
motor right1 = motor (PORT18, ratio6_1, false);
motor right2 = motor (PORT17, ratio6_1, false);
motor right3 = motor (PORT19, ratio6_1, false);
motor left1 = motor (PORT10, ratio6_1, true);
motor left2 = motor (PORT15, ratio6_1, true);
motor left3 = motor (PORT16, ratio6_1, true);
motor_group leftDrive = motor_group(left1, left2, left3);
motor_group rightDrive = motor_group(right1, right2, right3);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}