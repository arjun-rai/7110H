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
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //double test[] = {0,0};
  

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
  hEncoder.resetPosition();
  fEncoder.resetPosition();
  while (Inertial.isCalibrating()||Inertial2.isCalibrating()) {
  wait(100, msec);
  }
  
  leftDrive.resetRotation();
  rightDrive.resetRotation();
  // leftEncoder.resetRotation();
  // rightEncoder.resetRotation();
  driveBrake(coast);
  catapult.setBrake(hold);
  intake.setBrake(coast);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

double pos[] = {0,0};
double lastLeft = 0;
double lastRight =0;
double last_orientation_rad = 0;
double last_hEncoder =0;
double last_fEncoder =0;
void getCurrLoc()
{

  double hEncoder_angle = hEncoder.position(deg);
  double fEncoder_angle = fEncoder.position(deg);
  double hEncoder_delta = hEncoder_angle-last_hEncoder;
  double fEncoder_delta = fEncoder_angle-last_fEncoder;





  double orientation_rad = radians(Inertial.rotation());
  double orientation_delta = orientation_rad-last_orientation_rad;
  

  double local_X;
  double local_Y;

  if (orientation_delta==0)
  {
    local_X=hEncoder_delta;
    local_Y=fEncoder_delta;
  }
  else{
    local_X=(2*sin(orientation_delta/2.0))*((hEncoder_delta/orientation_delta)+2);
    local_Y=(2*sin(orientation_delta/2.0))*((fEncoder_delta/orientation_delta)+3.3);
  }

  double local_polar_angle;
  double local_polar_length;
  if (local_X==0&local_Y==0)
  {
    local_polar_angle=0;
    local_polar_length=0;
  }
  else 
  {
    local_polar_angle=atan2(local_Y, local_X);
    local_polar_length=distanceP(local_X, 0, local_Y, 0);
  }

  double global_polar_angle = local_polar_angle-last_orientation_rad-(orientation_delta/2.0);

  double X_position_delta = local_polar_length*cos(global_polar_angle);
  double Y_position_delta = local_polar_length*sin(global_polar_angle);

  X_position_delta = (X_position_delta/360.0)*M_PI*2.75;
  Y_position_delta = (Y_position_delta/360.0)*M_PI*2.75;

  pos[0]+=X_position_delta;
  pos[1]+=Y_position_delta;
  last_orientation_rad=orientation_rad;
  last_fEncoder=fEncoder_angle;
  last_hEncoder=hEncoder_angle;
}

double kP = 0; //steady minor oscillations, should stop close to the correct point 0.07
double kI = 0; //compensate for undershoot
double kD = 0; //until steady 0.0001

double turnkP = 0.254; //0.068 0.0517 0.063 0.23 0.138     0.45  0.446  0.404
double turnkI = 0;//.0044     0.001
double turnkD = 0; //0.0001  0.5 0.1  0.000001
double turnkF = 0.001; //0.001
//Autonomous Settings
double desiredValue = 0;
double desiredTurnValue = 0;
double startingTurnValue =0;

double error; //SensorValue - DesiredValue : Position 
double prevError = 0; //Position 2- milleseconds ago
double derivative; // error - prevError : Speed
double totalError=0; //totalError = totalError + error;

double turnError; //SensorValue - DesiredValue : Position 
double turnPrevError = 0; //Position 2- milleseconds ago
double turnDerivative; // error - prevError : Speed
double turnTotalError=0; //totalError = totalError + error;

double tuckyLeftVar= 1;
double tuckyRightVar  =1;

int integralBound =90;
int averagePosition;
bool resetDriveSensors = false;
double maxLateralPower = 100;
double maxTurningPower = 24;
double maxLateralChange=15;
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
      startingTurnValue=(Inertial.rotation());
      
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
    turnError =desiredTurnValue-((Inertial.rotation()));
    if (fabs(turnError)<1)
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
    if ( fabs(turnError) > 45)
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
    Controller.Screen.print((Inertial.rotation()));//(Inertial.rotation()+Inertial2.rotation())/2.0
    if (lateralMotorPower>0 && lateralMotorPower-lastLateralVoltage>maxLateralChange)
    {
      lateralMotorPower = lastLateralVoltage+maxLateralChange;
    }
    if (lateralMotorPower<0 && lateralMotorPower-lastLateralVoltage<-maxLateralChange)
    {
      lateralMotorPower = lastLateralVoltage-maxLateralChange;
    }

    lastLateralVoltage=lateralMotorPower;
    
    leftDrive.spin(fwd, tuckyLeftVar*(lateralMotorPower - turnMotorPower), pct);
    rightDrive.spin(fwd, tuckyRightVar*(lateralMotorPower + turnMotorPower), pct);
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(10);
  }

  leftDrive.stop(vex::brakeType::brake);
  rightDrive.stop(vex::brakeType::brake);
  return 1;
}


double distError;
double distPrevError;
double distDerivative;

double curveError;
double curvePrevError;
double curveDerivative;

double pointKpMove=1.5;
double pointKdMove=0;

double pointKpCurve=0.002;
double pointKdCurve=0;

double lastStraightPct =0;

double desiredPos[] = {0,0};

bool enablePointToPoint = true;
int pointToPoint()
{
  while (enablePointToPoint)
  {
    curveError = degree(atan2((desiredPos[1]-pos[1]), (desiredPos[0]-pos[0])))-90;
    distError = distanceP(pos[0], pos[1], desiredPos[0], desiredPos[1]);
    if (fmod(curveError,360)>90||fmod(curveError,-360)<-90)
    {
      distError=-distError;
    }
    distDerivative = distError-distPrevError;
    curveDerivative=curveError-curvePrevError;

    double straightPct = (distError*pointKpMove+distDerivative*pointKdMove);
    double curvePct = (curveError*pointKpCurve);

    if(curvePct<1 && curvePct>0)
    {
      curvePct=1;
    }

    if(curvePct>-1 && curvePct<0)
    {
      curvePct=-1;
    }

    // if(straightPct-lastStraightPct>5)
    // {
    //   straightPct=lastStraightPct+5;
    // }

    if (curvePct>0)
    {
      leftDrive.spin(fwd, straightPct/curvePct, pct);
      rightDrive.spin(fwd, straightPct, pct);
    }
    else {
      leftDrive.spin(fwd, straightPct, pct);
      rightDrive.spin(fwd, straightPct/curvePct, pct);
    }

    if (fabs(pos[0]-desiredPos[0])<0.5&&fabs(pos[1]-desiredPos[1])<0.5)
    {
      break;
    }
    // if (fabs(curveError)<1)
    // {
    //   pointKpCurve=0;
    //   // pointKpMove=0;
    // }

    distPrevError=distError;
    curvePrevError=curveError;
    lastStraightPct=straightPct;
    wait(20, msec);
  }
  leftDrive.stop(coast);
  rightDrive.stop(coast);
  return 1;
}



void PID(double x, double y)
{
  resetDriveSensors=true;
  double ang = degree(atan2(x-pos[0], y-pos[1]));
  desiredTurnValue=ang;
  drivePID();
  resetDriveSensors=true;
  desiredValue=((distanceP(pos[0], pos[1], x, y)*4*360)/(M_PI*3.25*5));
  drivePID();  
}
void PIDMove(double move, double ang)
{
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
    Controller.Screen.setCursor(0, 0);
    Controller.Screen.clearLine();
    Controller.Screen.print("%d %d %d", (int)pos[0], (int)pos[1], (int)curveError);
    vex::task::sleep(10);
  }
  return 1;
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
      wait(20, msec);
      cataBoost.set(true);
    }
    if (cataSense.angle(deg)>168&&fire)
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
  desiredPos[1]=48;desiredPos[0]=48;
  vex::task odometry(odom);
  pointToPoint();
  // PIDMove(0, 180);
  
}


double LeftPercent = 0;
double RightPercent =0;
double lastLeftPercent = 0;
double lastRightPercent = 0;
void leftExpo (vex::directionType type, double percentage){
  if(percentage >= 0){
    percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
  }else{
    percentage = -percentage;
    percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
    percentage = -percentage;
  }
  // if (percentage-lastLeftPercent>0 && percentage!=0 && lastLeftPercent<0)
  // {
  //   percentage=lastLeftPercent+5;
  // }
  LeftPercent=percentage;
  if (LeftPercent>=70&&RightPercent>=70)
  {
    percentage=70;
  }
  if (percentage<-4&&rightDrive.velocity(pct)>0&&leftDrive.velocity(pct)>0&&fabs(LeftPercent-RightPercent)<5)
  {
    percentage=lastLeftPercent-0.8;
  }
  leftDrive.spin (type, percentage, vex::velocityUnits::pct);
  lastLeftPercent=percentage;
}

void rightExpo (vex::directionType type, double percentage){
  if(percentage >= 0){
    percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
  }else{
    percentage = -percentage;
    percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
    percentage = -percentage;
  }
  RightPercent=percentage;
  if (LeftPercent>=70&&RightPercent>=70)
  {
    percentage=70;
  }
  if (percentage<-4&&rightDrive.velocity(pct)>0&&leftDrive.velocity(pct)>0&&fabs(LeftPercent-RightPercent)<5)
  {
    percentage=lastRightPercent-0.8;
  }
  rightDrive.spin (type, percentage, vex::velocityUnits::pct);
  lastRightPercent=percentage;
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

bool modeOn = false;
bool modeToggle = false;

bool loaderOn = false;
bool loaderToggle = false;

int discCount =0;
int loaderCount=0;

bool intakeLift = false;
bool liftToggle = false;

void usercontrol(void) {
  // intake.stop();

  cataBoost.set(false);
  autonCata=false;
  enableDrivePID=false;
  //Controller.Screen.clearLine();
  //Controller.Screen.print(averagePosition);
  // User control code here, inside the loop
  // vex::task odometry(odom);
  while (1) {

    if (Controller.ButtonDown.pressing()&&Controller.ButtonB.pressing())
    {
      intakeToggle=false;
      // expand = true;
      // reload=false;
      // catapult.spin(reverse, 70, vex::velocityUnits::pct);
      // wait(450, msec);
      expansion.set(true);
    }
    else
    {
      expansion.set(false);
    }
    driveBrake(coast);
    // leftDrive.spin(vex::directionType::fwd, driveSpeed*(Controller.Axis3.value() + turnSpeed*(Controller.Axis1.value())), vex::velocityUnits::pct);
    // rightDrive.spin(vex::directionType::fwd,  driveSpeed*(Controller.Axis3.value() - turnSpeed*(Controller.Axis1.value())), vex::velocityUnits::pct);
    leftExpo(vex::directionType::fwd, (Controller.Axis3.value() - Controller.Axis1.value()));
    rightExpo(vex::directionType::fwd, (Controller.Axis3.value() + Controller.Axis1.value()));

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
      intake.spin(fwd, 600, vex::velocityUnits::rpm);
    }
    else {
      intake.stop();
    }

    if (Controller.ButtonX.pressing())
    {
      if (!intakeLift)
      {
        liftToggle = !liftToggle;
        intakeLift=true;
      }
    }
    else
    {
      intakeLift=false;
    }
    if (liftToggle)
    {
      intakeLifter.set(true);
    }
    else
    {
      intakeLifter.set(false);
    }

     if (Controller.ButtonUp.pressing())
    {
      if (!modeOn)
      {
        modeToggle = !modeToggle;
        modeOn=true;
      }
    }
    else  
    {
      modeOn=false;
    }
    if (modeToggle)
    {
      cataReduce.set(true);
    }
    else {
      cataReduce.set(false);
    }

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
    // if (Controller.ButtonY.pressing())
    // {
    //   if (!loaderOn)
    //   {
    //     loaderToggle = !loaderToggle;
    //     loaderOn=true;
    //     loaderCount=0;
    //   }
    // }
    // else
    // {
    //   if (loaderCount==5)
    //   {
    //     loaderToggle=false;
    //   }
    //   loaderOn=false;
    // }

  


    
    if (Controller.ButtonRight.pressing())
    {
      if (!boostOn)
      {
        boostToggle = !boostToggle;//!boostToggle
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
      catapult.spin(reverse, 100, vex::velocityUnits::pct);
    }
    if (Controller.ButtonR2.pressing() && intakeSense.objectDistance(mm)>170)
    {
      intakeToggle=false;
      reload=false;
      catapult.spin(reverse, 100, vex::velocityUnits::pct);
      wait(50, msec);
      if (boostToggle)
        cataBoost.set(true);
      // if (true)
      // {
      //   wait(140, msec);
      //   cataBoost.set(false);
      // }
    }
    // if (reload&&loaderToggle&&cataSense.angle(deg)>116.5)
    // {
    //   catapult.stop(hold);
      
    // }
    if (reload && cataSense.angle(deg)>100)
    {
       catapult.spin(reverse, 60, vex::velocityUnits::pct);
    }
    if (reload && cataSense.angle(deg)>120)//93
    {
      catapult.stop(hold);
      
    }
    else if (!reload && cataSense.angle(deg)<50)
    {
      intakeToggle=false;
      catapult.stop(coast);
      cataBoost.set(false);

      reload=true;
      catapult.spin(reverse, 100, vex::velocityUnits::pct);
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




    wait(10, msec); // Sleep the task for a short amount of time to
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
