using namespace vex;

extern brain Brain;

extern motor BackLeft;
extern motor BackRight;
extern motor FrontLeft;
extern motor FrontRight;
extern motor MiddleLeft;
extern motor MiddleRight;
extern controller Controller;
extern controller Controller2;
extern drivetrain Drivetrain;
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern motor Roller;
extern motor Moby;
extern motor Wheel;
extern digital_out Clamp1;
extern digital_out Clamp2;
extern digital_out Lift1;
extern digital_out Lift2;
extern digital_out Lift3;
extern inertial Inert;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);