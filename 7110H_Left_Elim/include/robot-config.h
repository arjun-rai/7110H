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
extern motor GearboxLeft;
extern digital_out wings;
extern motor GearboxRight;
extern digital_out backWing;
extern digital_out release;
extern rotation parallelEncoder;
extern rotation perpendicularEncoder;
extern motor intake;
extern distance dist_sens;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );