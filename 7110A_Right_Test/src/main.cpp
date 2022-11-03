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
void driveBrake(vex::brakeType b)
{
  BackLeft.setBrake(b);
  BackRight.setBrake(b);
  FrontLeft.setBrake(b);
  FrontRight.setBrake(b);
  MiddleLeft.setBrake(b);
  MiddleRight.setBrake(b);
}
int autonNum =-1;
//point(10, 20), point(15, 30), point(10, 50),
std::vector<std::vector<pathPoint>> pathMain = {
  {point(0, 0), point(0, -10)},
  {point(0, -10), point(-10,-10)}
  };
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  flywheel.setBrake(coast);
  for (int i =0;i<pathMain.size(); i++)
  {
    pathMain[i] = inject(pathMain[i]);
    pathMain[i] = smooth(pathMain[i]);
    curv_func(pathMain[i]);
    speed_func(pathMain[i]);
  }
  //double test[] = {0,0};
  
  // for (int i =0;i<path.size(); i++)
  // {
  //   printf("%d\t%f\n", i, path[i].finVel);
  //   wait(50 ,msec);
  // }

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
  // leftEncoder.resetRotation();
  // rightEncoder.resetRotation();
  driveBrake(brake);
  flywheel.setBrake(coast);
  intake.setBrake(coast);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

double pos[] = {0,0};
double lastLeft = 0;
double lastRight =0;
void getCurrLoc()
{
  double dist = ((leftDrive.rotation(rev)*(3.0/5)*M_PI*3.25)-lastLeft + (rightDrive.rotation(rev)*(3.0/5)*M_PI*3.25)-lastRight)/2.0;
  // if (dist*cos(radians(Inertial.rotation()))>100 || dist*sin(radians(Inertial.rotation()))>100)
  //   return;
  // Controller.Screen.setCursor(0,0);
  // Controller.Screen.clearLine();
  // Controller.Screen.print(pos[0]);
  printf("%f\t%f\t%f\n", pos[0], pos[1], Inertial.rotation());
  //printf("%f\t%f\n", wheels[0], wheels[1]);
  pos[0] += dist*sin(radians(Inertial.rotation()));
  pos[1] += dist*cos(radians(Inertial.rotation()));
  lastLeft = leftDrive.rotation(rev)*M_PI*3.25*(3.0/5);
  lastRight = rightDrive.rotation(rev)*M_PI*3.25*(3.0/5);
}

double kP = 0.01; //steady minor oscillations, should stop close to the correct point
double kI = 0; //compensate for undershoot
double kD = 0; //until steady 0.0001

double turnkP = 0.068; //0.0664
double turnkI = 0;
double turnkD = 0.0001; //0.006
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
double maxTurningPower = 6;
double maxLateralChange=1;
double lastLateralVoltage = 0;

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
      leftDrive.resetPosition();
      rightDrive.resetPosition();
      leftDrive.resetRotation();
      rightDrive.resetRotation();
      
    }
    //Get the position of both motors
    int leftMotorPosition = leftDrive.position(degrees);
    int rightMotorPosition = rightDrive.position(degrees);


    //////////////////////////////////////////////////////////
    //Lateral Movement PID
    /////////////////////////////////////////////////////////
    //Get average of the two motors
    averagePosition = (leftMotorPosition + rightMotorPosition)/2;
    //printf("%d\n", averagePosition);
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
    //getCurrLoc();
    
    // printf("%f\n", Inertial.rotation());
    // Controller.Screen.setCursor(0,0);
    // Controller.Screen.clearLine();
    // Controller.Screen.print(Inertial.rotation());

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
    Controller.Screen.setCursor(0,0);
    Controller.Screen.clearLine();
    Controller.Screen.print(Inertial.rotation());
    if (lateralMotorPower>0 && lateralMotorPower-lastLateralVoltage>maxLateralChange)
    {
      lateralMotorPower = lastLateralVoltage+maxLateralChange;
    }
    if (lateralMotorPower<0 && lateralMotorPower-lastLateralVoltage<-maxLateralChange)
    {
      lateralMotorPower = lastLateralVoltage-maxLateralChange;
    }

    lastLateralVoltage=lateralMotorPower;
    
    leftDrive.spin(fwd, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    rightDrive.spin(fwd, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(10);
  }


  return 1;
}

double flykP = 0; //steady minor oscillations, should stop close to the correct point 0.0003 good     0.0015
double flykI = 0; //compensate for undershoot
double flykD = 0; //until steady 0.0001     0.048
double flykF=0.0038888;
//Autonomous Settings

int flyError; //SensorValue - DesiredValue : Position 
int flyPrevError = 0; //Position 2- milleseconds ago
int flyDerivative; // error - prevError : Speed
int flyTotalError=0; //totalError = totalError + error;

double flyVolt=0;

int desiredFly=0;
int flyIntegralBound =25;
bool resetFlySensors = false;

//Variables modified for use
bool enableFlyPID = true;
int tempTime =0;
std::vector<double> vals1 = {};
std::vector<double> vals2 = {};
int FlyPID(){

  while(enableFlyPID)
  {
    if (resetFlySensors)
    {
      resetFlySensors = false;
     // Inertial.setRotation(0, degrees);
      flywheel.setPosition(0, degrees);
    }
    //Get the position of both motors
    int flywheelRPM = flywheel.velocity(vex::velocityUnits::rpm)*6.0;
    //////////////////////////////////////////////////////////
    //Lateral Movement PID
    /////////////////////////////////////////////////////////
    //Get average of the two motors
    
    //Potential
    flyError = desiredFly-flywheelRPM;

    //Derivative
    flyDerivative = flyError - flyPrevError;

    //Integral (Highly suggested do not use it)
    //Velocity -> Postion -> Absement (Position and Time)
    flyTotalError += flyError;

    //Maybe /12.0 ?
    double flyMotorPower = (flyError*flykP) + (flyDerivative*flykD)+(flyTotalError*flykI)+(desiredFly*flykF);
    //
    // if (flyMotorPower>3)
    // {
    //   flyMotorPower=3;
    // }
    double newVolt = flyMotorPower;
    // if (newVolt>=10)
    // {
    //   newVolt=10;
    // }
    if(desiredFly==0.0)
    {
      continue;
    }
    flywheel.spin(fwd, newVolt, voltageUnits::volt);
    flyVolt = newVolt;
    // Controller.Screen.setCursor(1, 0);
    // Controller.Screen.clearLine();
    // Controller.Screen.print(flywheelRPM);
    // printf("%f\t%f\n", tempTime*0.020, flywheel.velocity(rpm)*6);
    vals1.push_back(tempTime*0.02);
    vals2.push_back(flywheelRPM);
    tempTime+=1;

    flyPrevError = flyError;
    vex::task::sleep(20);
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





// bool enableOdom=false;
// int odom()
// {
//   while (enableOdom)
//   {
//     getCurrLoc();
//   }
//   return 1;
// }

double track_width = 15;
//double dt = 0.005;
double maxVelChange=3;
bool pathing(std::vector<pathPoint> path, bool backwards)
{
  double lastVel = 0;
  while (closest(pos, path)!=path.size()-1)
  {
    getCurrLoc();
    double look[] = {};
    lookahead(pos, path, look);
    int close = closest(pos, path);
    double curv;
    if (look[2]>close)
    {
      curv = curvature(path, pos, look, radians(Inertial.rotation()));
      //curv = curvature(path, pos, look, radians(0));
    }
    else
      curv = 0.00001;
    double vel = path[close].finVel;
    vel = lastVel+constrain(vel, lastVel, -maxVelChange, maxVelChange);
    lastVel = vel;
    double wheels[] = {};
    turn(curv, vel, track_width, wheels);
    //can add coefficients and tune for better velocity accuracy if 
    if (backwards)
    {
      // printf("%f\t%f\n", wheels[0], wheels[1]);
      //printf("%f\t%f\n", pos[0], pos[1]);
      rightDrive.spin(fwd, -wheels[1]/(3.25*M_PI*(3.0/5))*60, vex::velocityUnits::rpm);
      leftDrive.spin(fwd, -wheels[0]/(3.25*M_PI*(3.0/5))*60, vex::velocityUnits::rpm);
    }
    else
    {
      rightDrive.spin(fwd, wheels[1]/(3.25*M_PI*(3.0/5))*60, vex::velocityUnits::rpm);
      leftDrive.spin(fwd, wheels[0]/(3.25*M_PI*(3.0/5))*60, vex::velocityUnits::rpm);
    }
    //double avg = (rightDrive.velocity(vex::velocityUnits::rpm)+leftDrive.velocity(vex::velocityUnits::rpm))/2.0;
    //printf("%d\t%f\n", close, vel);
    //printf("%f\n", look[2]);
    wait(10, msec);
  }
  return true;
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
//Due to turning scrub, use a track width a couple inches larger than the real one

void autonomous(void) {
  // enableOdom=true;
  // vex::task Odom(odom);
  //pathing(pathMain[0], true);
  // vex::task flyPID1(FlyPID);
  // enableFlyPID=true;
  // desiredFly=450;
  // wait(2000, msec);
  // intake.spin(reverse, 150, vex::velocityUnits::rpm);
  
  // vex::task flyPID1(FlyPID);
  // desiredFly=400;
  //vex::task prof(profile);
  
  vex::task PID1(drivePID);
  vex::task PID2(FlyPID);
  desiredFly=600*6; //422
  resetDriveSensors=true;
  intake.spin(fwd, 100, vex::velocityUnits::pct);
  desiredValue=1600;
  wait(1000, msec);
  resetDriveSensors=true;
  desiredValue=0;
  desiredTurnValue=-45;
  wait(2000, msec);
  resetDriveSensors=true;
  desiredValue=800;
  wait(2000, msec);
  intake.stop();
  resetDriveSensors=true;
  desiredValue=0;
  desiredTurnValue=30;
  wait(1000,msec);
  resetDriveSensors=true;
  desiredValue=200;

  // wait(1430, msec); //1230
//   wait(500, msec); //1200
  
//   //printf("%f %f\n", flywheel.temperature(), intake.temperature());
  while (flywheel.velocity(rpm)*6<3300)
  {
    wait(20, msec);
  }
  //waitUntil(flywheel.velocity(rpm)>desiredFly-5&&flywheel.velocity(rpm)<desiredFly+5);

  intake.spinFor(reverse, 0.75, rev, 40, vex::velocityUnits::pct);
  //waitUntil(flywheel.velocity(rpm)>desiredFly-10&&flywheel.velocity(rpm)<desiredFly+10)
 while (flywheel.velocity(rpm)*6<3300)
  {
    wait(20, msec);
  }
  //waitUntil(flywheel.velocity(rpm)>desiredFly-5&&flywheel.velocity(rpm)<desiredFly+5);
  intake.spinFor(reverse, 0.6, rev, 30, vex::velocityUnits::pct);
  //waitUntil(flywheel.velocity(rpm)>desiredFly-10&&flywheel.velocity(rpm)<desiredFly+10)
  wait(250, msec);
  intake.spinFor(fwd, 2, rev, 50, vex::velocityUnits::pct);
  // wait(1000, msec);
  while (flywheel.velocity(rpm)*6<3300)
  {
    wait(20, msec);
  }
  //waitUntil(flywheel.velocity(rpm)>desiredFly-10&&flywheel.velocity(rpm)<desiredFly+10);
  intake.spinFor(reverse, 2.65, rev, 50, vex::velocityUnits::pct);
//   wait(1000, msec);
//   desiredFly=0;
//   flyVolt=0;
//   flywheel.stop();
  // for (int i =0;i<vals1.size(); i++)
  // {
  //   printf("%f\t%f\n", vals1[i], vals2[i]);
  //   wait(20, msec);
  // }
  // resetDriveSensors=true;
  // desiredValue=0;
  // desiredTurnValue=-70;
  // wait(750, msec);
  // resetDriveSensors=true;
  // desiredValue=-1800;
  // wait(2000, msec);
  // resetDriveSensors=true;
  // desiredValue=0;
  // desiredTurnValue=0;
  // wait(500, msec);
  // resetDriveSensors=true;
  // desiredValue=-620;
  // wait(1000, msec);
  // intake.spinFor(fwd, 80, deg, 100, vex::velocityUnits::pct);


  // wait(700, msec); //715
  // intake.spin(reverse, 100, vex::velocityUnits::pct);
  // wait(250,msec);
  // intake.stop();

  // wait(700, msec);
  // intake.spin(reverse, 100, vex::velocityUnits::pct);
  // wait(750,msec);
  // intake.stop();

  

  // desiredValue=-300;
  // desiredTurnValue=0;
  // wait(1000, msec);
  // resetDriveSensors=true;
  // desiredValue=0;
  // desiredTurnValue=90;
  // wait(1500, msec);
  // resetDriveSensors=true;
  // desiredValue=-1300;
  // wait(1500, msec);
  // resetDriveSensors=true;
  // desiredValue=0;
  // desiredTurnValue=180;
  // wait(1000, msec);
  // resetDriveSensors=true;
  // desiredValue=-400;
  // wait(1000, msec);
  // intake.spinFor(reverse, 1000, deg, 100, vex::velocityUnits::pct);
  // resetDriveSensors=true;
  // desiredValue=200;
  // wait(1000,msec);
  // resetDriveSensors=true;
  // desiredTurnValue=115;
  

  
  // lastLeft=0;
  // lastRight=0;
  // pos[0]=0;
  // pos[1]=0;
  
  // leftDrive.resetRotation();
  // rightDrive.resetRotation();
  
  // Inertial.resetRotation();
  
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
bool intakeOn = false;
bool intakeToggle =false;
bool indexerOn = false;
bool indexerToggle =false;
bool flyOn = false;
bool flyToggle =false;
float initTurnSpeed = 0.6;
float altTurnSpeed = 0.3;
float turnSpeed = initTurnSpeed;
bool turnOn = false;
float initDriveSpeed = 0.5;
float altDriveSpeed = 1.0;
float driveSpeed = initDriveSpeed;
bool driveOn = false;
float initFwdIntakeSpeed = 200;
// float altFwdIntakeSpeed = 200;
float initBackIntakeSpeed = 125;
float altBackIntakeSpeed = 200;
float fwdIntakeSpeed = initFwdIntakeSpeed;
float backIntakeSpeed = initBackIntakeSpeed;
// bool fwdIntakeSpdIncOn = false;
bool backIntakeSpdIncOn = false;
void usercontrol(void) {
  vex::task flyPID1(FlyPID);
  desiredFly=0;
  enableDrivePID=false;
  //Controller.Screen.clearLine();
  //Controller.Screen.print(averagePosition);
  // User control code here, inside the loop
  while (1) {
    // Controller.Screen.setCursor(0, 0);
    // Controller.Screen.clearLine();
    // Controller.Screen.print(flywheel.voltage());

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    // Controller.Screen.clearLine();
    // Controller.Screen.setCursor(0, 0);
    // Controller.Screen.print(leftEncoder.position(degrees));

    driveBrake(brake);
    leftDrive.spin(vex::directionType::fwd, driveSpeed*(Controller.Axis3.value() + turnSpeed*(Controller.Axis1.value())), vex::velocityUnits::pct);
    rightDrive.spin(vex::directionType::fwd,  driveSpeed*(Controller.Axis3.value() - turnSpeed*(Controller.Axis1.value())), vex::velocityUnits::pct);
    // if (Controller.ButtonL2.pressing())
    // {
    //   if (!indexerOn)
    //   {
    //     indexerToggle = !indexerToggle;
    //     indexerOn=true;
    //   }
    // }
    // else
    // {
    //   indexerOn=false;
    // }
    if (Controller.ButtonL2.pressing())
    {
      if (!indexerOn)
      {
        indexerToggle = !indexerToggle;
        indexerOn=true;
      }
    }
    else
    {
      indexerOn=false;
    }
    if (Controller.ButtonL1.pressing())
    {
      if (!intakeOn)
      {
        intakeToggle = !intakeToggle;
        intakeOn=true;
      }
    }
    else
    {
      intakeOn=false;
    }
    if (intakeToggle&&indexerToggle)
    {
      // 10/5: 150->125
      intake.spin(reverse, backIntakeSpeed, vex::velocityUnits::rpm);
    }
    else if (intakeToggle)
    {
      intake.spin(fwd, fwdIntakeSpeed, vex::velocityUnits::rpm);
    }
    else {
      intake.stop();
    }
    if (Controller.ButtonR1.pressing()||Controller.ButtonR2.pressing())
    {
      if (!flyOn)
      {
        flyToggle = !flyToggle;
        flyOn=true;
      }
    }
    else
    {
      flyOn=false;
    }
    if (Controller.ButtonUp.pressing()||Controller.ButtonLeft.pressing()||Controller.ButtonRight.pressing()||Controller.ButtonA.pressing()||Controller.ButtonY.pressing()||Controller.ButtonX.pressing())
    {
      if (!driveOn)
      {
        if(driveSpeed == initDriveSpeed)
          driveSpeed = altDriveSpeed;
        else
          driveSpeed = initDriveSpeed;
        driveOn = true;
      }
    }
    else
    {
      driveOn = false;
    }
    // if (Controller.ButtonUp.pressing()||Controller.ButtonLeft.pressing()||Controller.ButtonRight.pressing())
    // {
    //   if (!turnOn)
    //   {
    //     if(turnSpeed == initTurnSpeed)
    //       turnSpeed = altTurnSpeed;
    //     else
    //       turnSpeed = initTurnSpeed;
    //     turnOn = true;
    //   }
    // }
    // else
    // {
    //   turnOn = false;
    // }
    // if (Controller.ButtonX.pressing()||Controller.ButtonB.pressing()||Controller.ButtonY.pressing()||Controller.ButtonA.pressing())
    // {
    //   if (!backIntakeSpdIncOn)
    //   {
    //     if(backIntakeSpeed == initBackIntakeSpeed)
    //       backIntakeSpeed = altBackIntakeSpeed;
    //     else
    //       backIntakeSpeed = initBackIntakeSpeed;
    //     backIntakeSpdIncOn = true;
    //   }
    // }
    // else
    // {
    //   backIntakeSpdIncOn = false;
    // }
    // if (Controller.ButtonX.pressing()||Controller.ButtonY.pressing()||Controller.ButtonA.pressing())
    // {
    //   backIntakeSpeed = altBackIntakeSpeed;
    // }
    // else
    // {
    //   backIntakeSpeed = initBackIntakeSpeed;
    // }
    if (Controller.ButtonDown.pressing()||Controller.ButtonB.pressing())
    {
      expansion.set(true);
    }
    else
    {
      expansion.set(false);
    }
    if (flyToggle)
    {
      // flywheel.spin(fwd, 340, rpm);
      enableFlyPID=true;
      // 10/5: 365->390
      desiredFly=530*6;
    }
    else 
    {
      flywheel.stop();
      desiredFly=0;
      flyVolt=0;
      //enableFlyPID=false;
      
    }
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
