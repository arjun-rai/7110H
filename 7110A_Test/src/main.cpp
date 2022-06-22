/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    8, 10, 3, 20    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "pure-pursuit.cpp"
#include "auton-selector.cpp"
using namespace vex;

// A global instance of competition
competition Competition;
// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

//Button Class for auton selector buttons, with hex codes for color

//auton buttons with the text, and hex codes for colors

int autonNum =-1;
std::vector<pathPoint> path = {point(0, 0), point(1, 1), point(4, 4)};
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  path = inject(path);
  path = smooth(path);
  curv_func(path);
  speed_func(path);
  

  //shows button, allows user to select button and then stops once submit is pressed
  //autonNum = autonSelector();
  Brain.Screen.clearScreen(vex::black);
  //Inertial.calibrate(2000);
  Inertial.resetRotation();
  Inertial.setHeading(0, degrees);
  Inertial.calibrate();
  while (Inertial.isCalibrating()) {
  wait(100, msec);
  }
  leftDrive.resetRotation();
  rightDrive.resetRotation();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

double kP = 0.02; //steady minor oscillations, should stop close to the correct point
double kI = 0; //compensate for undershoot
double kD = 0.0001; //until steady 0.0001

double turnkP = 0.1; //0.1
double turnkI = 0;
double turnkD = 0.06; //0.0001
//Autonomous Settings
double desiredValue = 0;
double desiredTurnValue = 0;

int error; //SensorValue - DesiredValue : Position 
int prevError = 0; //Position 2- milleseconds ago
int derivative; // error - prevError : Speed
int totalError=0; //totalError = totalError + error;

int turnError; //SensorValue - DesiredValue : Position 
int turnPrevError = 0; //Position 2- milleseconds ago
int turnDerivative; // error - prevError : Speed
int turnTotalError=0; //totalError = totalError + error;

int integralBound =25;
int averagePosition;
bool resetDriveSensors = false;
double maxLateralPower = 11.8;
double maxTurningPower = 10;

//Variables modified for use
bool enableDrivePID = true;

int drivePID(){

  while(enableDrivePID)
  {
    if (resetDriveSensors)
    {
      resetDriveSensors = false;
     // Inertial.setRotation(0, degrees);
      leftDrive.setPosition(0, degrees);
      rightDrive.setPosition(0, degrees);
    }
    //Get the position of both motors
    int leftMotorPosition = leftDrive.position(degrees);
    int rightMotorPosition = rightDrive.position(degrees);


    //////////////////////////////////////////////////////////
    //Lateral Movement PID
    /////////////////////////////////////////////////////////
    //Get average of the two motors
    averagePosition = (leftMotorPosition + rightMotorPosition)/2;

    //Potential
    error = desiredValue-averagePosition;

    //Derivative
    derivative = error - prevError;

    //Integral (Highly suggested do not use it)
    //Velocity -> Postion -> Absement (Position and Time)
    totalError += error;

    //Maybe /12.0 ?
    double lateralMotorPower = error*kP + derivative*kD+totalError*kI;
    //

    //////////////////////////////////////////////////////////
    //Turning Movement PID
    /////////////////////////////////////////////////////////
    //int turnDifference = leftMotorPosition - rightMotorPosition;

    //Potential
    turnError =desiredTurnValue-Inertial.rotation();
    


    //Derivative
    turnDerivative = turnError - turnPrevError;

    //Integral (Highly suggested do not use it)
    //Velocity -> Postion -> Absement (Position and Time)
    if (abs(turnError)<integralBound)
    {
      turnTotalError += turnError;
    }
    else {
      turnTotalError=0;
    }
    

    //Maybe /12.0 ?
    double turnMotorPower = turnError*turnkP + turnDerivative*turnkD+turnTotalError*turnkI;
    // x
    
    if(lateralMotorPower>maxLateralPower)
    {
      lateralMotorPower=maxLateralPower;
    }
    if(lateralMotorPower<-maxLateralPower)
    {
      lateralMotorPower=-maxLateralPower;
    }
     
    if(turnMotorPower>maxTurningPower)
    {
      turnMotorPower=maxTurningPower;
    }
    if(turnMotorPower<-maxTurningPower)
    {
      turnMotorPower=-maxTurningPower;
    }
    leftDrive.spin(fwd, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    rightDrive.spin(fwd, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(5);
  }


  return 1;
}

void PID(double move, double angle)
{
  resetDriveSensors=true;
  desiredValue=((move)/(M_PI*3.25))*360.0;
  desiredTurnValue=angle;
}
void PID(double move)
{
  resetDriveSensors=true;
  desiredValue=((move)/(M_PI*3.25))*360.0;
}



void driveBrake(vex::brakeType b)
{
  RightBack.setBrake(b);
  RightFront.setBrake(b);
  LeftBack.setBrake(b);
  LeftFront.setBrake(b);
}

double loc =44; //inches  dist+2
bool enableProfile = true;
int profile()
{
  driveBrake(coast);
  if (loc>=32)
  {
    double maxRpm = 200;
    double distForAccel = 16; //inches

    double maxV = maxRpm*((M_PI*3.25)/60.0); //inches per second
    double a = (maxV*maxV)/(2.0*distForAccel); //inch/s^2  
    double timeForConst = (loc-2*distForAccel)/maxV;
    double timeForAccel = ((loc - (timeForConst*maxV))/(0.5*maxV))/2.0;
    int size = (timeForAccel*1000)/10.0;
    double vel[size];
    vel[0] = 0;
    for (int i =1;i<size;i++)
    {
      vel[i] = vel[i-1]+(a/100.0);
    }

    for (int i =0;i<size; i++)
    {
      double rpm = ((vel[i])/(M_PI*3.25))*60;
      rightDrive.spin(fwd, rpm, vex::velocityUnits::rpm);
      leftDrive.spin(fwd, rpm, vex::velocityUnits::rpm);
      wait(10, msec);
    }

    wait(timeForConst, sec);

    for (int i =size-1;i>=0; i--)
    {
      double rpm = ((vel[i])/(M_PI*3.25))*60;
      rightDrive.spin(fwd, rpm, vex::velocityUnits::rpm);
      leftDrive.spin(fwd, rpm, vex::velocityUnits::rpm);
      wait(10, msec);
    }
  }
  else 
  {
    double maxRpm = 50;
    double distForAccel = 6;

    double maxV = maxRpm*((M_PI*3.25)/60.0); //inches per second
    double a = (maxV*maxV)/(2.0*distForAccel); //inch/s^2  
    double timeForConst = (loc-2*distForAccel)/maxV;
    double timeForAccel = ((loc - (timeForConst*maxV))/(0.5*maxV))/2.0;
    int size = (timeForAccel*1000)/10.0;
    double vel[size];
    vel[0] = 0;
    for (int i =1;i<size;i++)
    {
      vel[i] = vel[i-1]+(a/100.0);
    }
    for (int i =0;i<size; i++)
    {
      double rpmVal = ((vel[i])/(M_PI*3.25))*60;
      rightDrive.spin(fwd, rpmVal, vex::velocityUnits::rpm);
      leftDrive.spin(fwd, rpmVal, vex::velocityUnits::rpm);
      wait(10, msec);
    }
    wait(timeForConst, sec);
    for (int i =size-1;i>=0; i--)
    {
      double rpmVal = ((vel[i])/(M_PI*3.25))*60;
      rightDrive.spin(fwd, rpmVal, vex::velocityUnits::rpm);
      leftDrive.spin(fwd, rpmVal, vex::velocityUnits::rpm);
      wait(10, msec);
    }
  }
  return 1;
}

double pos[] = {0,0};
double lastLeft = 0;
double lastRight =0;
void getCurrLoc()
{
  double dist = ((leftDrive.rotation(rev)*M_PI*3.25)-lastLeft + (rightDrive.rotation(rev)*M_PI*3.25)-lastRight)/2.0;
  pos[0] += dist*cos(radians(Inertial.rotation()));
  pos[1] += dist*sin(radians(Inertial.rotation()));
  lastLeft = leftDrive.rotation(rev)*M_PI*3.25;
  lastRight = rightDrive.rotation(rev)*M_PI*3.25;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
double drive_width = 10;
double dt = 0.005;
double maxVelChange=500;
void autonomous(void) {
  while (closest(pos, path)!=path.size()-1)
  {
    getCurrLoc();
    double look[] = {};
    lookahead(pos, path, look);
    int close = closest(pos, path);
    double curv;
    if (look[2]>close)
      curv = curvature(path, pos, look);
    else
      curv = 0.00001;
    double vel = path[close].finVel;
    double wheels[] = {};
    turn(curv, vel, drive_width, wheels);
    rightDrive.spin(fwd, wheels[1], vex::velocityUnits::rpm);
    leftDrive.spin(fwd, wheels[0], vex::velocityUnits::rpm);
    wait(20, msec);
  }
  //vex::task prof(profile);
  // vex::task PID1(drivePID);
  // PID(48);
    // wait(1000, msec);
    // PID(0, 90);
    // wait(1000, msec);
    // PID(12);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  enableDrivePID=false;
  //Controller.Screen.clearLine();
  //Controller.Screen.print(averagePosition);
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    // Controller.Screen.clearLine();
    // Controller.Screen.print(frontLift.rotation(degrees));
    leftDrive.spin(vex::directionType::fwd, (Controller.Axis3.value() + (Controller.Axis1.value())), vex::velocityUnits::pct);
    rightDrive.spin(vex::directionType::fwd,  (Controller.Axis3.value() - (Controller.Axis1.value())), vex::velocityUnits::pct);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
