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
extern motor motor1;
extern digital_out wings;
extern motor motor2;
extern digital_out wingsBackLeft;
extern digital_out wingsBackRight;
extern digital_out ratchet;
extern rotation parallelEncoder;
extern rotation perpendicularEncoder;
extern motor intake;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );