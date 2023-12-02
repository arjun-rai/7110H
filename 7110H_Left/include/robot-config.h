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
extern motor catapult;
extern digital_out intakeLifter;
extern digital_out wings;
extern motor intake;
extern digital_out blooper;
//extern digital_out auton_grabber;
extern rotation cataSense;
extern digital_out elevation;
extern digital_out elevationLifter;
extern distance distSensor;
extern digital_out blockerLifter;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );