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
extern motor_group leftDrive;
extern motor_group rightDrive;
extern motor cata;
extern motor cata2;
extern motor_group catapult;
extern digital_out claw;
extern digital_out claw2;
extern digital_out lifter;
extern digital_out balance;
extern digital_out wedge;
extern rotation cataSense;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );