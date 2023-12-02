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
  // for (int i =0;i<pathMain.size(); i++)
  // {
  //   pathMain[i] = inject(pathMain[i]);
  //   pathMain[i] = smooth(pathMain[i]);
  //   curv_func(pathMain[i]);
  //   speed_func(pathMain[i]);
  // }

  // for (int i=0;i<pathMain[0].size(); i++)
  // {
  //   printf("%f\t%f\n", pathMain[0][i].x, pathMain[0][i].y);
  //   wait(20, msec);
  // }
  

  //shows button, allows user to select button and then stops once submit is pressed
  //autonNum = autonSelector();
  Brain.Screen.clearScreen(vex::black);
  //Inertial.calibrate(2000);
  Inertial.calibrate();
  while (Inertial.isCalibrating()) {
  wait(100, msec);
  }
  
  Inertial.resetRotation();
  Inertial.setHeading(0, degrees);
  
  cataSense.resetPosition();
  // leftEncoder.resetRotation();
  // rightEncoder.resetRotation();
  driveBrake(coast);
  catapult.setBrake(hold);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

double kP = 0.2; //steady minor oscillations, should stop close to the correct point
double kI = 0; //compensate for undershoot
double kD = 1; //until steady

double turnkP = 0.915; //0.057
double turnkI = 0; //0.0035
double turnkD = 4.3;
double turnkF = 0;
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

double curveLeftVar= 1;
double curveRightVar  =1;

int integralBound =90;
int averagePosition;
bool resetDriveSensors = false;
double maxLateralPower = 12;
double maxTurningPower = 12;
double maxLateralChange=1;
double lastLateralVoltage = 0;
timer t;
//Variables modified for use
bool enableDrivePID = true;
bool turning = false;

int drivePID(){
  t = timer();
  while(enableDrivePID)
  {
    if (t.time(seconds)>2)
    {
      break;
    }
    if (resetDriveSensors)
    {
      resetDriveSensors = false;
     // Inertial.setRotation(0, degrees);
      leftDrive.setPosition(0, degrees);
      rightDrive.setPosition(0, degrees);
      leftDrive.resetPosition();
      rightDrive.resetPosition();
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
    if (fabs(error)>fabs(totalError))
    {
      totalError=0;
    }
    if (error == 0)
    {
      totalError = 0;
    }

    //Maybe /12.0 ?
    double lateralMotorPower = error*kP + derivative*kD+totalError*kI;
    //

    //////////////////////////////////////////////////////////
    //Turning Movement PID
    /////////////////////////////////////////////////////////
    //int turnDifference = leftMotorPosition - rightMotorPosition;

    //Potential
    turnError =desiredTurnValue-((Inertial.rotation()));
    //printf("%f\t%d\n", t.time(seconds), averagePosition);
     if ((fabs(turnError)<1 && turning && fabs(turnDerivative)<1) || (fabs(error)<10 && !turning && fabs(derivative)<1))
    {
      break;
    }
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
    if ( fabs(turnError)<fabs(turnTotalError))
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

    if(turnMotorPower>maxTurningPower)
    {
      turnMotorPower=maxTurningPower;
    }
    if(turnMotorPower<-maxTurningPower)
    {
      turnMotorPower=-maxTurningPower;
    }
     
    // if(turnMotorPower>maxTurningPower)
    // {
    //   turnMotorPower=maxTurningPower;
    // }
    // if(turnMotorPower<-maxTurningPower)
    // {
    //   turnMotorPower=-maxTurningPower;
    // }
    // Controller.Screen.setCursor(0,0);
    // Controller.Screen.clearLine();
    // Controller.Screen.print((Inertial.rotation()));
    if (lateralMotorPower>0 && lateralMotorPower-lastLateralVoltage>maxLateralChange)
    {
      lateralMotorPower = lastLateralVoltage+maxLateralChange;
    }
    if (lateralMotorPower<0 && lateralMotorPower-lastLateralVoltage<-maxLateralChange)
    {
      lateralMotorPower = lastLateralVoltage-maxLateralChange;
    }

    lastLateralVoltage=lateralMotorPower;
    
    leftDrive.spin(fwd, curveLeftVar*(lateralMotorPower + turnMotorPower), volt);
    rightDrive.spin(fwd, curveRightVar*(lateralMotorPower - turnMotorPower), volt);
    prevError = error;
    turnPrevError = turnError;
    //printf("%f\n", Inertial.rotation());
    wait(10, msec);
  }
  leftDrive.stop(vex::brakeType::hold);
  rightDrive.stop(vex::brakeType::hold);
  return 1;
}


// double moveError;
// double movePrevError;
// double moveDerivative;

// double KpMove=1;
// double KdMove=10;


// double lastMovePct =0;

// double maxMoveVoltage =10; //8

// double desiredLength = 0;

// float clamp(float input, float min, float max){
//   if( input > max ){ return(max); }
//   if(input < min){ return(min); }
//   return(input);
// }

// bool enableDist = true;
// int dist(double timeout, brakeType chooseBrakeType)
// {
//   double timeout_loop = (timeout*1000.0);
//   double timeout_time =0;
//   double startingDist = (fEncoder.position(deg)/360.0)*M_PI*2.75;
//   while (enableDist&&timeout_time<timeout_loop)
//   {
//     moveError = desiredLength-(((fEncoder.position(deg)/360.0)*M_PI*2.75)-startingDist);
//     moveDerivative = moveError-movePrevError;

//     double moveVolt = (moveError*KpMove+moveDerivative*KdMove);

//     moveVolt = clamp(moveVolt, -maxMoveVoltage, maxMoveVoltage);
//     // if(straightPct-lastStraightPct>5)
//     // {
//     //   straightPct=lastStraightPct+5;
//     // }
//     //printf("%f\n", timeout_time);
    
//     leftDrive.spin(fwd, moveVolt, voltageUnits::volt);
//     rightDrive.spin(fwd, moveVolt, voltageUnits::volt);


//     if (fabs(moveError)<3)
//     {
//       break;
//     }

//     movePrevError=moveError;
//     lastMovePct=moveVolt;
//     timeout_time+=20;
//     wait(20, msec);
//   }
//   leftDrive.stop(chooseBrakeType);
//   rightDrive.stop(chooseBrakeType);
//   return 1;
// }


void PIDTurn(double angle)
{
  resetDriveSensors=true;
  desiredTurnValue=angle;
  desiredValue=0;
  turning=true;
  drivePID();
}
// void PIDMove (double length, double timeout=5, brakeType chooseBrakeType=hold)
// {
//   desiredLength=length;
//   dist(timeout, chooseBrakeType);
// }
void PIDTurnMove(double move, double ang)
{
  resetDriveSensors=true;
  desiredTurnValue=ang;
  desiredValue=0;
  drivePID();
  resetDriveSensors=true;
  desiredValue=move;
  drivePID();
}

void PIDMove(double move)
{
  resetDriveSensors=true;
  desiredValue=move;
  turning=false;
  drivePID();
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
// bool load = true;
// bool fire = false;
bool load = true;
bool fire = true;
bool autonCata=true;
int loadCata()
{
  while (autonCata)
  {
    if (load)
    {
      catapult.spin(reverse, 100, vex::velocityUnits::pct);
    }
    if (cataSense.angle(deg)>225&&load)
    {
      catapult.spin(reverse, 10, vex::velocityUnits::pct);
    }
    if (cataSense.angle(deg)>257&&load)
    {
      catapult.stop(hold);
      load=!load;
    }
    if (cataSense.angle(deg)<185&&fire)
    {
      catapult.stop(coast);
      fire=!fire;
      load=true;
    }
    if (fire)
    {
      catapult.spin(reverse, 100, vex::velocityUnits::pct);
    }
    vex::task::sleep(20);
  }
  return 0;
}


void autonomous(void) {
  autonCata = false;
  
  PIDMove(2950);
  PIDTurn(-70);
  // PIDMove(900);
  leftDrive.spinFor(fwd, 1000, degrees, 100, vex::velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 1000, degrees, 100, vex::velocityUnits::pct, false);
  wings.set(true);
  wait(1000, msec);
  leftDrive.spinFor(reverse, 1000, degrees, 5, vex::velocityUnits::pct, false);
  rightDrive.spinFor(reverse, 1000, degrees, 5, vex::velocityUnits::pct);
  
}


double kTurn = 0.65;
double turnRemap(double turn){
  double denom = sin(M_PI/2 * kTurn);
  double firstRemap = sin(M_PI/2 * kTurn * turn)/denom;
  return sin(M_PI/2 * kTurn * firstRemap) / denom;
}

double fastStopAccum = 0.0;
double negInertiaAccum = 0.0;
void updateAccum(){
  if (negInertiaAccum>1){
    negInertiaAccum-=1;
  }
  else if (negInertiaAccum<-1) {
    negInertiaAccum+=1;
  }
  else{
    negInertiaAccum=0;
  }
}

double prevTurn = 0.0;
double prevThrottle = 0.0;

double drive_deadband=0.1;
double drive_slew = 0.02;
double kInertiaScalar =0.75;//0.5
double kSensitivty = 1;

void curvatureDrive(double throttle, double turn){
  bool pointTurn = false;
  double linear = throttle;
  if (fabs(throttle)<drive_deadband && fabs(turn)>drive_deadband)
  {
    linear = 0.0;
    pointTurn=true;
  }
  else if (throttle-prevThrottle>drive_slew) {
    linear = prevThrottle+drive_slew;
  }
  else if (throttle-prevThrottle<-(drive_slew*2)) {
    linear = prevThrottle-(drive_slew*2);
  }
  double remappedTurn = turnRemap(turn);
  double left, right;
  if (pointTurn){
    left = remappedTurn*fabs(remappedTurn);
    right = -remappedTurn*fabs(remappedTurn);
    // printf("%f\n", remappedTurn*fabs(remappedTurn));
  }
  else {
    double negInertiaPower = (turn-prevTurn)*kInertiaScalar;
    negInertiaAccum+=negInertiaPower;
    double angular = fabs(linear)*(remappedTurn+negInertiaAccum)*kSensitivty - fastStopAccum;

    right =linear; left = linear;
    //printf("%f\t%f\n", negInertiaAccum, angular);
    left += angular;
    right -=angular;
    updateAccum();
  }
  prevTurn=turn;
  prevThrottle=throttle;
  leftDrive.spin (fwd, left*100, vex::velocityUnits::pct);
  rightDrive.spin (fwd, right*100, vex::velocityUnits::pct);
}

double LeftPercent = 0;
double RightPercent = 0;
double lastLeftPercent = 0;
double lastRightPercent = 0;
void leftExpo (vex::directionType type, double percentage){
  if(fabs(percentage) < 1)
    percentage = 0;
  else if(percentage >= 1)
    percentage = 2*pow(1.05, percentage-42) + 1;
  else{
    percentage = -percentage;
    percentage = 2*pow(1.05, percentage-42) + 1;
    percentage = -percentage;
  }
  // if (percentage-lastLeftPercent>50)
  // {
  //   percentage=lastLeftPercent+50;
  // }
  // if (percentage-lastLeftPercent<-50)
  // {
  //   percentage=lastLeftPercent-50;
  // }
  LeftPercent=percentage;

  leftDrive.spin (type, percentage, vex::velocityUnits::pct);
  lastLeftPercent=percentage;
}

void rightExpo (vex::directionType type, double percentage){

  if(fabs(percentage) < 1)
    percentage = 0;
  else if(percentage >= 1)
    percentage = 2*pow(1.05, percentage-42) + 1;
  else{
    percentage = -percentage;
    percentage = 2*pow(1.05, percentage-42) + 1;
    percentage = -percentage;
  }

  //  if (percentage-lastRightPercent>50)
  // {
  //   percentage=lastRightPercent+50;
  // }
  // if (percentage-lastRightPercent<-50)
  // {
  //   percentage=lastRightPercent-50;
  // }


  RightPercent=percentage;
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
bool CataOn = false;
bool CataToggle =false;
bool reload = true;
bool elevationUpOn = false;
bool elevationUpToggle = false;
bool elevationOn = false;
bool elevationToggle = false;
double loadAngle = 250;
bool wingsOn = false;
bool wingsToggle = false;
bool blooperOn = false;
bool blooperToggle = false;
double maxSpeed = 127;
bool lifterOn = false;
bool lifterToggle = false;
bool sensorFireOn = false;
bool sensorFireToggle = false;
bool fullSpeedOn = false;
bool fullSpeedToggle = false;

void usercontrol(void) {
  intakeLifter.set(true);
  enableDrivePID=false;
  autonCata=false;
  // User control code here, inside the loop
  while (1) {
    driveBrake(coast);
    // rightExpo(forward, (Controller.Axis3.value() - Controller.Axis1.value()), maxSpeed);
    // leftExpo(forward, (Controller.Axis3.value() + Controller.Axis1.value()), maxSpeed);
    // rightExpo(forward, (Controller.Axis2.value()));
    // leftExpo(forward, (Controller.Axis3.value()));
    curvatureDrive(Controller.Axis3.value()/127.0, Controller.Axis1.value()/127.0);
    if (Controller.ButtonL1.pressing())
    {
      intake.spin(fwd, 600, rpm);
    }
    else if (Controller.ButtonL2.pressing())
    {
      intake.spin(reverse, 600, rpm);
    }
    else {
      intake.stop();
    }

    if (Controller.ButtonL1.pressing()&&Controller.ButtonL2.pressing())
    {
      if (!wingsOn)
      {
        wingsToggle=!wingsToggle;
        wingsOn=true;
      }
    }
    else {
      wingsOn=false;
    }
    if (wingsToggle)
    {
      wings.set(true);
    }
    else {
      wings.set(false);
    }

    if (Controller.ButtonUp.pressing())
    {
      if (!blooperOn)
      {
        blooperToggle=!blooperToggle;
        blooperOn=true;
      }
    }
    else {
      blooperOn=false;
    }
    if (blooperToggle)
    {
      blooper.set(true);
    }
    else {
      blooper.set(false);
    }

    if (Controller.ButtonDown.pressing())
    {
      if (!lifterOn)
      {
        lifterToggle=!lifterToggle;
        lifterOn=true;
      }
    }
    else {
      lifterOn=false;
    }
    if (lifterToggle)
    {
      blockerLifter.set(true);
    }
    else {
      blockerLifter.set(false);
    }

    if (Controller.ButtonX.pressing())
    {
      if (!elevationUpOn)
      {
        elevationUpToggle = !elevationUpToggle;
        elevationUpOn=true;
      }
    }
    else
    {
      elevationUpOn=false;
    }
    if (elevationUpToggle)
    {
      elevationLifter.set(true);
    }
    else
    {
      elevationLifter.set(false);
    }

    if (Controller.ButtonB.pressing())
    {
      if (!elevationOn)
      {
        elevationToggle = !elevationToggle;
        elevationOn=true;
      }
    }
    else
    {
      elevationOn=false;
    }
    if (elevationToggle)
    {
      elevationLifter.set(false);
      elevationUpToggle=false;
      elevation.set(true);
    }
    else
    {
      elevation.set(false);
    }

    if (Controller.ButtonR1.pressing())
    {
      reload=true;
      catapult.spin(reverse, 100, vex::velocityUnits::pct);
    }
    if (Controller.ButtonR2.pressing())
    {
      reload=false;
      catapult.spin(reverse, 100, vex::velocityUnits::pct);
    }
    if (reload && cataSense.angle(deg)>265)//93
    {
      catapult.stop(hold);
    }
    else if (!reload && cataSense.angle(deg)<242)
    {
      catapult.stop(coast);

      reload=true;
      catapult.spin(reverse, 100, vex::velocityUnits::pct);
    }
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
