using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller;
extern motor LeftFront;
extern motor LeftBack;
extern motor RightFront;
extern motor RightBack;
extern inertial Inertial;
extern motor_group leftDrive;
extern motor_group rightDrive;
extern encoder leftEncoder;
extern encoder rightEncoder;
extern motor flywheel;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );