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
  Inertial2.resetRotation();
  Inertial2.setHeading(0, degrees);
  Inertial2.calibrate();
  cataSense.setPosition(0, deg);
  cataSense.resetPosition();
  while (Inertial.isCalibrating()||Inertial2.isCalibrating()) {
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
  // printf("%f\t%f\t%f\n", pos[0], pos[1], (Inertial.rotation()+Inertial2.rotation())/2.0);
  //printf("%f\t%f\n", wheels[0], wheels[1]);
  pos[0] += dist*sin(radians((Inertial.rotation()+Inertial2.rotation())/2.0));
  pos[1] += dist*cos(radians((Inertial.rotation()+Inertial2.rotation())/2.0));
  lastLeft = leftDrive.rotation(rev)*M_PI*3.25*(3.0/5);
  lastRight = rightDrive.rotation(rev)*M_PI*3.25*(3.0/5);
}

double kP = 0.07; //steady minor oscillations, should stop close to the correct point
double kI = 0; //compensate for undershoot
double kD = 0; //until steady 0.0001

double turnkP = 0.404; //0.068 0.0517 0.063 0.23 0.138     0.45  0.446  0.404
double turnkI = 0;//.0044     0.001
double turnkD = 0.1; //0.0001  0.5 0.1  0.000001
double turnkF = 0.001;
//Autonomous Settings
double desiredValue = 0;
double desiredTurnValue = 0;
double startingTurnValue =0;

int error; //SensorValue - DesiredValue : Position 
int prevError = 0; //Position 2- milleseconds ago
int derivative; // error - prevError : Speed
int totalError=0; //totalError = totalError + error;

int turnError; //SensorValue - DesiredValue : Position 
int turnPrevError = 0; //Position 2- milleseconds ago
int turnDerivative; // error - prevError : Speed
int turnTotalError=0; //totalError = totalError + error;

double tuckyLeftVar= 1;
double tuckyRightVar  =1;

int integralBound =90;
int averagePosition;
bool resetDriveSensors = false;
double maxLateralPower = 100;
double maxTurningPower = 24;
double maxLateralChange=15;
double lastLateralVoltage = 0;

double timeMax = 1000000;

//Variables modified for use
bool enableDrivePID = true;

int drivePID(){
  vex::timer check = vex::timer();
  while(enableDrivePID)
  {
    if (check.time()>timeMax)
    {
      return 0;
    }
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
      startingTurnValue=(Inertial.rotation()+Inertial2.rotation())/2.0;
      
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
    turnError =desiredTurnValue-((Inertial.rotation()+Inertial2.rotation())/2.0);
    if (abs(turnError)<3&&abs(error)<50)
    {
      break;
    }

    //getCurrLoc();
    
    // printf("%f\n", Inertial.rotation());
    // Controller.Screen.setCursor(0,0);
    // Controller.Screen.clearLine();
    // Controller.Screen.print(Inertial.rotation());

    //Derivative
    turnDerivative = turnError - turnPrevError;

    //Integral (Highly suggested do not use it)
    //Velocity -> Postion -> Absement (Position and Time)
    if (turnError == 0)
    {
      turnTotalError = 0;
    }
    if ( abs(turnError) > 45)
    {
      turnTotalError = 0;
    }

    // if (turnDerivative<0.1)
    // {
      turnTotalError += turnError;
    // }
    // else {
    //   turnTotalError=0;
    // }

   
    

    //Maybe /12.0 ?
    double turnMotorPower = turnError*turnkP + turnDerivative*turnkD+turnTotalError*turnkI + turnkF*(desiredTurnValue-startingTurnValue);
    // x
    
    if(lateralMotorPower>maxLateralPower)
    {
      lateralMotorPower=maxLateralPower;
    }
    if(lateralMotorPower<-maxLateralPower)
    {
      lateralMotorPower=-maxLateralPower;
    }
     
    // if(turnMotorPower>maxTurningPower)
    // {
    //   turnMotorPower=maxTurningPower;
    // }
    // if(turnMotorPower<-maxTurningPower)
    // {
    //   turnMotorPower=-maxTurningPower;
    // }
    Controller.Screen.setCursor(0,0);
    Controller.Screen.clearLine();
    Controller.Screen.print((Inertial.rotation()+Inertial2.rotation())/2.0);//(Inertial.rotation()+Inertial2.rotation())/2.0
    if (lateralMotorPower>0 && lateralMotorPower-lastLateralVoltage>maxLateralChange)
    {
      lateralMotorPower = lastLateralVoltage+maxLateralChange;
    }
    if (lateralMotorPower<0 && lateralMotorPower-lastLateralVoltage<-maxLateralChange)
    {
      lateralMotorPower = lastLateralVoltage-maxLateralChange;
    }

    lastLateralVoltage=lateralMotorPower;
    
    leftDrive.spin(fwd, tuckyLeftVar*(lateralMotorPower + turnMotorPower), pct);
    rightDrive.spin(fwd, tuckyRightVar*(lateralMotorPower - turnMotorPower), pct);
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(10);
  }

  leftDrive.stop();
  rightDrive.stop();
  return 1;
}



void PID(double x, double y)
{
  resetDriveSensors=true;
  double ang = degree(atan2(x-pos[0], y-pos[1]));
  desiredTurnValue=ang;
  drivePID();
  resetDriveSensors=true;
  desiredValue=((distanceP(pos[0], pos[1], x, y))/(M_PI*3.25))*360.0;
  drivePID();  
}
void PIDMove(double move, double ang, double time = 1000000)
{
  timeMax=time;
  resetDriveSensors=true;
  desiredTurnValue=ang;
  desiredValue=0;
  drivePID();
  resetDriveSensors=true;
  desiredValue=move;
  drivePID();
}





bool enableOdom=true;
int odom()
{
  while (enableOdom)
  {
    // Controller.Screen.setCursor(0,0);
    // Controller.Screen.clearLine();
    // Controller.Screen.print(pos[1]);
    getCurrLoc();
    vex::task::sleep(20);
  }
  return 1;
}

double track_width = 12;
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
      curv = curvature(path, pos, look, radians((Inertial.rotation()+Inertial2.rotation())/2.0));
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
  rightDrive.stop();
  leftDrive.stop();
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
bool load=false;
bool fire = false;
bool autonCata=true;
bool loader=false;
int loadCata()
{
  while (autonCata)
  {
    if (load)
    {
      catapult.spin(reverse, 80, vex::velocityUnits::pct);
    }
    if (cataSense.angle(deg)<103&&load&&loader)
    {
      catapult.stop(hold);
      load=!load;
    }
    if (cataSense.angle(deg)<93&&load)
    {
      catapult.stop(hold);
      load=!load;
    }
    if (fire)
    {
      catapult.spin(reverse, 80, vex::velocityUnits::pct);
    }
    if (cataSense.angle(deg)>168&&fire)
    {
      catapult.stop(coast);
      fire=!fire;
    }
    vex::task::sleep(20);
  }
  return 0;
}

void autonomous(void) {
  autonCata=true;
  // vex::task PID1(drivePID);
  vex::task cata(loadCata);
  loader=true;
  for (int i =0;i<3;i++)
  {
  intake.spin(fwd, 400, rpm);
  if (i!=0)
  {
    wait(2200, msec);
  }
  else {
    wait(900, msec);
  }
  
  intake.spin(reverse, 400, rpm);
  wait(200,msec);
  intake.spin(fwd, 500, rpm);
  PIDMove(200, 0);
  PIDMove(0, 75);//-290
  fire=true;
  wait(500, msec);
  load=true;
  if (i!=2)
  {
    PIDMove(0, 0);
    PIDMove(-210, 0);
  }
  if (i==2)
  {
    loader=false;
  }
  }
  load=true;
  intake.stop();
  PIDMove(0, 95);
  PIDMove(-2150, 95);
  PIDMove(0, 0);
  PIDMove(-440, 0, 1000);
  intake.spinFor(reverse, 540, deg, 100, vex::velocityUnits::pct, false);
  wait(1000, msec);
  intake.stop();
  PIDMove(220, 0);
  PIDMove(0, -225);
  intake.spin(reverse, 500, rpm);
  PIDMove(-1500,-220);
  PIDMove(150,-220);
  // desiredTurnValue=180;
  PIDMove(0, -260);
    // intake.spin(reverse, 500, vex::velocityUnits::rpm);
  intake.stop();
  PIDMove(-590, -260,1000);

  
  // wait(500, msec);
  // resetDriveSensors=true;
  // desiredValue=0;
  intake.spinFor(reverse, 600, deg, 100, vex::velocityUnits::pct,false);
  wait(1000, msec);
  intake.stop();
  wait(100,msec);
  fire=true;
  wait(500, msec);
  load=true;
  PIDMove(100, -260);
  intake.spin(reverse, 600, rpm);
  PIDMove(0, -135);
  PIDMove(-2000, -135);
  PIDMove(-1000, -135);
  PIDMove(-1000, -135);
  PIDMove(0, -45);
  PIDMove(200, -45);
  fire=true;
  wait(500, msec);
  load=true;
  // PIDMove(-100, -45);
  PIDMove(0, -196);
  PIDMove(-500, -196);
  PIDMove(-500, -196);
  PIDMove(-500, -196);
  PIDMove(-500, -196);
  PIDMove(0, -90);
  intake.spin(fwd, 600, rpm);
  fire=true;
  wait(500,msec);
  load=true;
  PIDMove(-900, -90);
  PIDMove(0, -45);
  intake.spin(reverse, 600, rpm);
  PIDMove(-800, -45);
  PIDMove(-200, -45);
  PIDMove(-200, -45);
  PIDMove(-400, -45);
  PIDMove(-400, -45);
  intake.spin(fwd, 600, rpm);
  PIDMove(2200, -45);
  PIDMove(0, -90);
  PIDMove(700, -90);
  fire=true;
  wait(500, msec);
  // load=true;
  intake.stop();
  PIDMove(-3000, -90);
  PIDMove(0, -180);
  PIDMove(-860, -180);
  intake.spinFor(reverse, 600, deg, 100, vex::velocityUnits::pct,false);
  wait(1000, msec);
  intake.stop();
  PIDMove(300, -180);
  intake.spin(reverse, 600, rpm);
  PIDMove(0, -45);
  PIDMove(-950, -45);
  PIDMove(0, -90);
  intake.stop();
  PIDMove(-900, -90);
  intake.spinFor(reverse, 600, deg, 100, vex::velocityUnits::pct,false);
  wait(1000, msec);
  intake.stop();
  PIDMove(550, -90);
  PIDMove(0, -132);
  expansion.set(true);
  // wait(1000000,msec);

  // PIDMove(360, -260);
  // PIDMove(0, -230);
  // PIDMove(1400, -230);
  // PIDMove(0, -270);
  // PIDMove(2800, -270);
  // fire=true;
  // wait(500, msec);
  // loader=true;
  // load=true;
  // PIDMove(-870, -270);//-850
  // PIDMove(0,-360);
  // PIDMove(-650, -360);

 
  
  
  // PIDMove(100, -360);
  // PIDMove(0, -356);//-356
  // PIDMove(6000, -356); //-356.5
  // PIDMove(0, -180);//-185
  // PIDMove(-1230, -180);//-950
  // // intake.spin(reverse, 400, rpm);
  // intake.spin(fwd, 500, rpm);
  // wait(500, msec);

  // for (int i =0;i<3;i++)
  // {
  // intake.spin(fwd, 500, rpm);
  // wait(2200, msec);
  // intake.spin(reverse, 400, rpm);
  // wait(200,msec);
  // intake.spin(fwd, 300, rpm);
  // PIDMove(200, -180);
  // PIDMove(0, -103);
  // fire=true;
  // wait(400, msec);
  // if (i==2)
  // {
  //   loader=false;
  // }
  // load=true;
  // if (i!=2)
  // {
  //   PIDMove(0, -180);
  //   PIDMove(-270, -180);
  // }
  // }
  // intake.stop();
  // PIDMove(0, -86);
  // PIDMove(-2100, -86);
  // PIDMove(0, -180);
  // PIDMove(-440, -180);
  // intake.spinFor(reverse, 540, deg, 100, vex::velocityUnits::pct);
  // PIDMove(900, -180);
  // PIDMove(0, -140);
  //  wait(100000,msec);
  // wait(10000, msec);



  // resetDriveSensors=true;
  // desiredValue=0;
  // desiredTurnValue=-458;
  // wait(800, msec);
  // resetDriveSensors=true;
  // desiredValue=-640;
  // wait(1000, msec);
  //  for (int i =0;i<4;i++)
  // {
  // wait(3500, msec);
  // fire=true;
  // wait(200, msec);
  // load=true;
  // wait(300, msec);
  // }
  // resetDriveSensors=true;
  // intake.stop();
  // desiredTurnValue=-69;
  // desiredValue=0;
  // wait(900,msec);
  // resetDriveSensors=true;
  // desiredValue=-2000;
  // wait(1900, msec);
  // resetDriveSensors=true;
  // desiredTurnValue=-182;
  // desiredValue=0;
  // wait(700, msec);
  // resetDriveSensors=true;
  // desiredValue=-400;
  // wait(700, msec);
  // intake.spinFor(reverse, 540, deg, 100, vex::velocityUnits::pct);
  

  // desiredValue=50;
  // wait(200, msec);
  // resetDriveSensors=true;
  // desiredTurnValue=-220-182;
  // wait(1000, msec);
  // intake.spin(reverse, 600, rpm);
  // resetDriveSensors=true;
  // desiredValue=-1500;

  // wait(1000, msec);
  // resetDriveSensors=true;
  // desiredValue=250;
  // // desiredTurnValue=180;
  // wait(500, msec);
  // resetDriveSensors=true;
  // desiredValue=0;
  // desiredTurnValue=-280-182;
  // wait(800, msec);
  // // intake.spin(reverse, 500, vex::velocityUnits::rpm);
  // desiredValue=-550;
  // intake.stop();
  // wait(730, msec);
  
  // // wait(500, msec);
  // resetDriveSensors=true;
  // desiredValue=0;
  // intake.spinFor(reverse, 540, deg, 100, vex::velocityUnits::pct);
  // resetDriveSensors=true;
  // desiredValue=640;
  // wait(500, msec);
  // resetDriveSensors=true;
  // desiredTurnValue=-302-182;
  // wait(500, msec);
  // // wait(800, msec);
  // fire=true;
  // wait(50,msec);
  // expansion=true;
  // resetDriveSensors=true;
  // desiredValue=-400;
 
 
  
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

bool loaderOn = false;
bool loaderToggle = false;

int discCount =0;
int loaderCount=0;

void usercontrol(void) {
  
  autonCata=true;
  // vex::task PID1(drivePID);
  vex::task cata(loadCata);
  loader=true;
  for (int i =0;i<3;i++)
  {
  intake.spin(fwd, 400, rpm);
  if (i!=0)
  {
    wait(2200, msec);
  }
  else {
    wait(900, msec);
  }
  
  intake.spin(reverse, 400, rpm);
  wait(200,msec);
  intake.spin(fwd, 500, rpm);
  PIDMove(200, 0);
  PIDMove(0, 75);//-290
  fire=true;
  wait(500, msec);
  load=true;
  if (i!=2)
  {
    PIDMove(0, 0);
    PIDMove(-210, 0);
  }
  if (i==2)
  {
    loader=false;
  }
  }
  autonCata=false;
  enableDrivePID=false;
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

    // if (Controller.ButtonUp.pressing()&&Controller.ButtonX.pressing())
    // {
    //   blocker.set(true);
    // }

    // if (Controller.ButtonLeft.pressing()||Controller.ButtonRight.pressing()||Controller.ButtonY.pressing())
    // {
    //   if (!driveOn)
    //   {
    //     driveToggle = !driveToggle;
    //     driveOn = true;
    //   }
    // }
    // else
    // {
    //   driveOn = false;
    // }
    // if (driveToggle)
    // {
    //   driveSpeed = altDriveSpeed;
    //   turnSpeed = altTurnSpeed;
    // }
    // else
    // {
    //   driveSpeed = initDriveSpeed;
    //   turnSpeed=initTurnSpeed;
    // }
    if (Controller.ButtonY.pressing())
    {
      if (!loaderOn)
      {
        loaderToggle = !loaderToggle;
        loaderOn=true;
        loaderCount=0;
      }
    }
    else
    {
      if (loaderCount==5)
      {
        loaderToggle=false;
      }
      loaderOn=false;
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
    if (reload&&loaderToggle&&cataSense.angle(deg)<103)
    {
      catapult.stop(hold);
      
    }
    if (reload && cataSense.angle(deg)<93)//93
    {
      catapult.stop(hold);
      
    }
    else if (!reload && cataSense.angle(deg)>168)
    {
      intakeToggle=false;
      catapult.stop(coast);
      cataBoost.set(false);
      if (loaderToggle)
      {
        loaderCount+=1;
      }
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
