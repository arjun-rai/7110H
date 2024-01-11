#include "vex.h"
#include "pid.h"
#include "pure-pursuit.h"

double turnkP = 0.915; //0.057
double turnkI = 0; //0.0035
double turnkD = 4.3;
double turnkF = 0;

double pivotP = 0.21;
double pivotI =0.08;
//Autonomous Settings
double desiredTurnValue = 0;
double startingTurnValue =0;

double turnError; //SensorValue - DesiredValue : Position 
double turnPrevError = 0; //Position 2- milleseconds ago
double turnDerivative; // error - prevError : Speed
double turnTotalError=0; //totalError = totalError + error;

double curveLeftVar= 1;
double curveRightVar  =1;

int integralBound =90;
bool resetDriveSensors = false;
double maxTurningPower = 12;
timer t;
//Variables modified for use
bool enableDrivePID = true;
double timeLimit = 3;
bool pivot = false;
bool rightStop = true;
bool leftStop = true;

int turnPID(){
  t = timer();
  while(enableDrivePID)
  {
    if (t.time(seconds)>timeLimit)
    {
      break;
    }
    if (resetDriveSensors)
    {
      resetDriveSensors = false;
     // Inertial.setRotation(0, degrees);
      startingTurnValue=(Inertial.rotation());
      
    }
    //////////////////////////////////////////////////////////
    //Turning Movement PID
    /////////////////////////////////////////////////////////
    //Potential
    turnError =desiredTurnValue-((Inertial.rotation()));
    if ((fabs(turnError)<2 && fabs(turnDerivative)<1))
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
    double turnMotorPower;
    if (!pivot)
    {
      turnMotorPower = turnError*turnkP + turnDerivative*turnkD+turnTotalError*turnkI + turnkF*(desiredTurnValue-startingTurnValue);
    }// x
    else {
      turnMotorPower = pivotP*turnError+pivotI*turnTotalError;
    }
    
    if(turnMotorPower>maxTurningPower && !pivot)
    {
      turnMotorPower=maxTurningPower;
    }
    if(turnMotorPower<-maxTurningPower && !pivot)
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
    if (rightStop && pivot)
    {
        leftDrive.spin(fwd, (turnMotorPower), volt);   
        rightDrive.stop(hold);   
    }
    if (leftStop && pivot)
    {
        rightDrive.spin(fwd, (turnMotorPower), volt);   
        leftDrive.stop(hold);   
    }
    else if (!pivot){
      leftDrive.spin(fwd, curveLeftVar*(turnMotorPower), volt);
      rightDrive.spin(fwd, curveRightVar*(-turnMotorPower), volt);
    }
    turnPrevError = turnError;
    // printf("%f\n", Inertial.rotation());
    // getCurrLoc();
    wait(10, msec);
  }
  leftDrive.stop(vex::brakeType::hold);
  rightDrive.stop(vex::brakeType::hold);
  return 1;
}


double moveError;
double movePrevError;
double moveDerivative;

double KpMove=1;
double KdMove=10;


double lastMovePct =0;

double maxMoveVoltage =10; //8

double desiredLength = 0;

bool enableDist = true;
int dist(double timeout, brakeType chooseBrakeType)
{
  double timeout_loop = (timeout*1000.0);
  double timeout_time =0;
  double startingDist = (parallelEncoder.position(deg)/360.0)*M_PI*2.75;
  while (enableDist&&timeout_time<timeout_loop)
  {
    moveError = desiredLength-(((parallelEncoder.position(deg)/360.0)*M_PI*2.75)-startingDist);
    moveDerivative = moveError-movePrevError;

    double moveVolt = (moveError*KpMove+moveDerivative*KdMove);

    moveVolt = clamp(moveVolt, -maxMoveVoltage, maxMoveVoltage);
    // if(straightPct-lastStraightPct>5)
    // {
    //   straightPct=lastStraightPct+5;
    // }
    //printf("%f\n", timeout_time);
    
    leftDrive.spin(fwd, moveVolt, voltageUnits::volt);
    rightDrive.spin(fwd, moveVolt, voltageUnits::volt);


    if (fabs(moveError)<3)
    {
      break;
    }

    movePrevError=moveError;
    lastMovePct=moveVolt;
    timeout_time+=20;
    // getCurrLoc();
    wait(20, msec);
  }
  leftDrive.stop(chooseBrakeType);
  rightDrive.stop(chooseBrakeType);
  return 1;
}

void PIDTurn(double angle)
{
  resetDriveSensors=true;
  desiredTurnValue=angle;
  turnPID();
}
void PIDMove (double length, double timeout, vex::brakeType chooseBrakeType)
{
  desiredLength=length;
  dist(timeout, chooseBrakeType);
}