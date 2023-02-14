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
  cataSense.setPosition(0, deg);
  cataSense.resetPosition();
  while (Inertial.isCalibrating()) {
  wait(100, msec);
  }
  leftDrive.resetRotation();
  rightDrive.resetRotation();
  // leftEncoder.resetRotation();
  // rightEncoder.resetRotation();
  driveBrake(brake);
  catapult.setBrake(hold);
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
bool load=true;
bool fire = false;
bool autonCata=true;
int loadCata()
{
  while (autonCata)
  {
    if (load)
    {
      catapult.spin(reverse, 80, vex::velocityUnits::pct);
    }
    if (cataSense.angle(deg)<93&&load)
    {
      catapult.stop(hold);
      load=!load;
    }
    if (fire)
    {
      catapult.spin(reverse, 70, vex::velocityUnits::pct);
      wait(20, msec);
      cataBoost.set(true);
    }
    if (cataSense.angle(deg)>172&&fire)
    {
      catapult.stop(coast);
      cataBoost.set(false);
      fire=!fire;
    }
    vex::task::sleep(20);
  }
  return 0;
}

void autonomous(void) {
  autonCata=true;
  vex::task PID1(drivePID);
  vex::task cata(loadCata);
  resetDriveSensors=true;
  // intake.spin(fwd, 100, vex::velocityUnits::pct);
  desiredValue=-300;
  wait(1000, msec);
  intake.spinFor(fwd, 180, deg, 100, vex::velocityUnits::pct);
  load=true;
  resetDriveSensors=true;
  desiredValue=300;
  desiredTurnValue=48;
  wait(1000, msec);
  resetDriveSensors=true;
  desiredValue=1200;
  wait(2000, msec);
  resetDriveSensors=true;
  desiredValue=0;
  desiredTurnValue=-25; 
  wait(800, msec);
  resetDriveSensors=true;
  desiredValue=140;
  wait(1000, msec);
  fire=true;
  wait(500, msec);
  load=true;
  resetDriveSensors=true;
  desiredValue=0;
  desiredTurnValue=-115; //-110
  wait(800, msec);
  resetDriveSensors=true;
  desiredValue=-400;
  wait(800, msec);
  // resetDriveSensors=true;
  // desiredValue=200;
  // wait(800, msec);
  intake.spin(reverse, 600, vex::velocityUnits::rpm);
  resetDriveSensors=true;
  desiredValue=-100;
  wait(300, msec);
  resetDriveSensors=true;
  desiredValue=-100;
  wait(300, msec);
  resetDriveSensors=true;
  desiredValue=-200;
  wait(600, msec);
  resetDriveSensors=true;
  desiredValue=-200;
  wait(600, msec);
  resetDriveSensors=true;
  desiredValue=-200;
  wait(600, msec);
  // resetDriveSensors=true;
  // desiredValue=200;
  // wait(600, msec);
  resetDriveSensors=true;
  desiredValue=-400;
  wait(1000, msec);
  resetDriveSensors=true;
  desiredValue=-400;
  wait(800, msec);
  resetDriveSensors=true;
  desiredValue=0;
  desiredTurnValue=-25; 
  wait(800, msec);
  resetDriveSensors=true;
  desiredValue=300; //400
  wait(500, msec);
  intake.spin(fwd, 600, vex::velocityUnits::rpm);
  wait(500, msec);
  if (intakeSense.objectDistance(mm)>40)
  {
    fire=true;
    wait(400, msec);
    load=true;
  }


  // wait(500, msec);
  // load=true;
  // wait(3000, msec);
  // intake.spin(reverse, 500, vex::velocityUnits::rpm);
  // wait(2000, msec);
  // fire=true;
  // intake.stop();

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
  
  
  // wait(1430, msec); //1230
//   wait(500, msec); //1200
  
//   //printf("%f %f\n", flywheel.temperature(), intake.temperature());
 
}

void leftExpo (vex::directionType type, int percentage){
  if(percentage >= 0){
    percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
  }else{
    percentage = -percentage;
    percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
    percentage = -percentage;
  }
  leftDrive.spin (type, percentage, vex::velocityUnits::pct);
}

void rightExpo (vex::directionType type, int percentage){
  if(percentage >= 0){
    percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
  }else{
    percentage = -percentage;
    percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
    percentage = -percentage;
  }
  rightDrive.spin (type, percentage, vex::velocityUnits::pct);
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
bool indexerToggle =true;
bool CataOn = false;
bool CataToggle =false;
float initTurnSpeed = 0.6;
float altTurnSpeed = 1;
float turnSpeed = initTurnSpeed;
bool turnOn = false;
float initDriveSpeed = 0.5;
float altDriveSpeed = 1.0;
float driveSpeed = initDriveSpeed;
bool driveOn = false;
bool driveToggle = true;
bool reload = true;
bool expand = false;
bool boostOn = false;
bool boostToggle = false;

int discCount =0;

void usercontrol(void) {
  cataBoost.set(false);
  enableDrivePID=false;
  autonCata=false;
  //Controller.Screen.clearLine();
  //Controller.Screen.print(averagePosition);
  // User control code here, inside the loop
  while (1) {
    if (Controller.ButtonDown.pressing()&&Controller.ButtonB.pressing())
    {
      intakeToggle=false;
      expand = true;
      reload=false;
      catapult.spin(reverse, 70, vex::velocityUnits::pct);
      wait(450, msec);
      expansion.set(true);
    }
    else
    {
      expansion.set(false);
    }
    driveBrake(brake);
    // leftDrive.spin(vex::directionType::fwd, driveSpeed*(Controller.Axis3.value() + turnSpeed*(Controller.Axis1.value())), vex::velocityUnits::pct);
    // rightDrive.spin(vex::directionType::fwd,  driveSpeed*(Controller.Axis3.value() - turnSpeed*(Controller.Axis1.value())), vex::velocityUnits::pct);
    leftExpo(vex::directionType::fwd, (Controller.Axis3.value() + Controller.Axis1.value()));
    rightExpo(vex::directionType::fwd, (Controller.Axis3.value() - Controller.Axis1.value()));

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
        indexerToggle=true;
      }
    }
    else
    {
      intakeOn=false;
    }
    if (intakeToggle&&indexerToggle)
    {
      // 10/5: 150->125
      intake.spin(reverse, 600, vex::velocityUnits::rpm); //intake speed <----- this one!
    }
    else if (intakeToggle)
    {
      intake.spin(fwd, 400, vex::velocityUnits::rpm);
    }
    else {
      intake.stop();
    }

    if (Controller.ButtonUp.pressing()&&Controller.ButtonX.pressing())
    {
      blocker.set(true);
    }

    if (Controller.ButtonLeft.pressing()||Controller.ButtonRight.pressing()||Controller.ButtonY.pressing())
    {
      if (!driveOn)
      {
        driveToggle = !driveToggle;
        driveOn = true;
      }
    }
    else
    {
      driveOn = false;
    }
    if (driveToggle)
    {
      driveSpeed = altDriveSpeed;
      turnSpeed = altTurnSpeed;
    }
    else
    {
      driveSpeed = initDriveSpeed;
      turnSpeed=initTurnSpeed;
    }

    
     if (Controller.ButtonA.pressing())
    {
      if (!boostOn)
      {
        boostToggle = false;//!boostToggle
        boostOn=true;
      }
    }
    else
    {
      boostOn=false;
    }

    if (Controller.ButtonR1.pressing())
    {
      intakeToggle=false;
      reload=true;
      catapult.spin(reverse, 70, vex::velocityUnits::pct);
    }
    if (Controller.ButtonR2.pressing()&&intakeSense.objectDistance(mm)>40)
    {
      intakeToggle=false;
      reload=false;
      catapult.spin(reverse, 70, vex::velocityUnits::pct);
      wait(20, msec);
      if (boostToggle)
        cataBoost.set(true);
    }
    if (reload && cataSense.angle(deg)<93)
    {
      catapult.stop(hold);
      
    }
    else if (!reload && cataSense.angle(deg)>172)
    {
      intakeToggle=false;
      catapult.stop(coast);
      cataBoost.set(false);
      if (!expand)
      {
        intakeToggle=false;
        reload=true;
        catapult.spin(reverse, 80, vex::velocityUnits::pct);
      }
    }

    // if (intakeSense.objectDistance(mm)<=23)
    // {
    //   // Controller.Screen.setCursor(0,0);
    //   // Controller.Screen.clearLine();
    //   // Controller.Screen.print(1);
    //   discCount+=1;
    // }
    // else {
    //   //  Controller.Screen.setCursor(0,0);
    //   // Controller.Screen.clearLine();
    //   // Controller.Screen.print(0);
    // }

    // if (discCount>=6 && intakeSense.objectDistance(mm)>30)
    // {
    //   intakeToggle=false;
    // }




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
