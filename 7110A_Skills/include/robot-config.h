using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller;
extern motor BackLeft;
extern motor BackRight;
extern motor FrontLeft;
extern motor FrontRight;
extern motor MiddleLeft;
extern motor MiddleRight;
extern inertial Inertial;
extern inertial Inertial2;
extern motor_group leftDrive;
extern motor_group rightDrive;
// extern encoder leftEncoder;
// extern encoder rightEncoder;
extern motor catapult;
extern motor intake;
extern digital_out expansion;
extern rotation cataSense;
extern digital_out cataBoost;
extern distance intakeSense;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );