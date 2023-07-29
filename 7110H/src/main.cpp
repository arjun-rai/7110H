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
  Inertial.resetRotation();
  Inertial.setHeading(0, degrees);
  Inertial.calibrate();
  cataSense.resetPosition();
  while (Inertial.isCalibrating()) {
  wait(100, msec);
  }
  
  leftDrive.resetRotation();
  rightDrive.resetRotation();
  // leftEncoder.resetRotation();
  // rightEncoder.resetRotation();
  driveBrake(coast);
  cata.setBrake(hold);
  cata2.setBrake(hold);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

double kP = 0; //steady minor oscillations, should stop close to the correct point
double kI = 0; //compensate for undershoot
double kD = 0; //until steady

double turnkP = 0;
double turnkI = 0;
double turnkD = 0;
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
    if (fabs(turnError)<4)
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
    
    leftDrive.spin(fwd, curveLeftVar*(lateralMotorPower - turnMotorPower), pct);
    rightDrive.spin(fwd, curveRightVar*(lateralMotorPower + turnMotorPower), pct);
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(10);
  }

  leftDrive.stop(vex::brakeType::brake);
  rightDrive.stop(vex::brakeType::brake);
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
    vex::task::sleep(20);
  }
  return 0;
}



void autonomous(void) {
 
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
    percentage = 2*pow(1.05, percentage+42) + 1;
    percentage = -percentage;
  }

  // if(percentage >= 0){
  //   percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
  // }else{
  //   percentage = -percentage;
  //   percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
  //   percentage = -percentage;
  // }
  // if (percentage-lastLeftPercent>0 && percentage!=0 && lastLeftPercent<0)
  // {
  //   percentage=lastLeftPercent+5;
  // }
  LeftPercent=percentage;
  // if (LeftPercent>=70&&RightPercent>=70)
  // {
  //   percentage=70;
  // }
  // if (percentage<-4&&rightDrive.velocity(pct)>0&&leftDrive.velocity(pct)>0&&fabs(LeftPercent-RightPercent)<5)
  // {
  //   percentage=lastLeftPercent-0.8;
  // }
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
    percentage = 2*pow(1.05, percentage+42) + 1;
    percentage = -percentage;
  }


  // if(percentage >= 0){
  //   percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
  // }else{
  //   percentage = -percentage;
  //   percentage = 1.2*pow(1.043, percentage) + 0.2*percentage - 1.2;
  //   percentage = -percentage;
  // }
  RightPercent=percentage;
  // if (LeftPercent>=70&&RightPercent>=70)
  // {
  //   percentage=70;
  // }
  // if (percentage<-4&&rightDrive.velocity(pct)>0&&leftDrive.velocity(pct)>0&&fabs(LeftPercent-RightPercent)<5)
  // {
  //   percentage=lastRightPercent-0.8;
  // }
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
bool clawOn = false;
bool clawToggle = false;
bool lifterOn = false;
bool lifterToggle = false;
bool balanceOn = false;
bool balanceToggle = false;
bool wedgeOn = false;
bool wedgeToggle = false;


void usercontrol(void) {
   enableDrivePID=false;
  // User control code here, inside the loop
  while (1) {
    driveBrake(coast);
    // leftDrive.spin(vex::directionType::fwd, driveSpeed*(Controller.Axis3.value() + turnSpeed*(Controller.Axis1.value())), vex::velocityUnits::pct);
    // rightDrive.spin(vex::directionType::fwd,  driveSpeed*(Controller.Axis3.value() - turnSpeed*(Controller.Axis1.value())), vex::velocityUnits::pct);
    leftExpo(vex::directionType::fwd, (Controller.Axis3.value() + Controller.Axis1.value()));
    rightExpo(vex::directionType::fwd, (Controller.Axis3.value() - Controller.Axis1.value()));

    if (Controller.ButtonL1.pressing())
    {
      if (!clawOn)
      {
        clawToggle = !clawToggle;
        clawOn=true;
      }
    }
    else
    {
      clawOn=false;
    }
    if (clawToggle)
    {
      claw.set(true);
      claw2.set(true);
    }
    else
    {
      claw.set(false);
      claw2.set(false);
    }

    if (Controller.ButtonL2.pressing())
    {
      if (!lifterOn)
      {
        lifterToggle = !lifterToggle;
        lifterOn=true;
      }
    }
    else
    {
      lifterOn=false;
    }
    if (lifterToggle)
    {
      lifter.set(true);
    }
    else
    {
      lifter.set(false);
    }

    // if (Controller.ButtonUp.pressing())
    // {
    //   if (!balanceOn)
    //   {
    //     balanceToggle = !balanceToggle;
    //     balanceOn=true;
    //   }
    // }
    // else
    // {
    //   balanceOn=false;
    // }
    // if (balanceToggle)
    // {
    //   balance.set(true);
    // }
    // else
    // {
    //   balance.set(false);
    // }

    if (Controller.ButtonX.pressing())
    {
      if (!wedgeOn)
      {
        wedgeToggle = !wedgeToggle;
        wedgeOn=true;
      }
    }
    else
    {
      wedgeOn=false;
    }
    if (wedgeToggle)
    {
      wedge.set(true);
    }
    else
    {
      wedge.set(false);
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
    if (reload && cataSense.angle(deg)>225)
    {
       catapult.spin(reverse, 10, vex::velocityUnits::pct);
    }
    if (reload && cataSense.angle(deg)>257)//93
    {
      catapult.stop(hold);
    }
    else if (!reload && cataSense.angle(deg)<185)
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
