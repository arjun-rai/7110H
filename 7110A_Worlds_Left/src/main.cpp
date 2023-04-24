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

//LEFTTT
std::vector<std::vector<pathPoint>> pathMain = {
  {point(0, 0), point(0,15)},
  {point(8, 24), point(17, 3)},
  {point(17,3), point(-12, 36), point(-14,41), point(-24,50)},
  {point(-24,50), point(-42, 44), point(-38,24)},
  {point(-38,24), point(-7,36)}
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
  hEncoder.resetPosition();
  fEncoder.resetPosition();
  while (Inertial.isCalibrating()) {
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
    local_X=(2*sin(orientation_delta/2.0))*((hEncoder_delta/orientation_delta)+2.5);
    local_Y=(2*sin(orientation_delta/2.0))*((fEncoder_delta/orientation_delta)+0.75);
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
    if (fabs(turnError)<4)
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
    // Controller.Screen.setCursor(0,0);
    // Controller.Screen.clearLine();
    // Controller.Screen.print((Inertial.rotation()));//(Inertial.rotation()+Inertial2.rotation())/2.0
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
  double startingDist = (fEncoder.position(deg)/360.0)*M_PI*2.75;
  while (enableDist&&timeout_time<timeout_loop)
  {
    moveError = desiredLength-(((fEncoder.position(deg)/360.0)*M_PI*2.75)-startingDist);
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
    wait(20, msec);
  }
  leftDrive.stop(chooseBrakeType);
  rightDrive.stop(chooseBrakeType);
  return 1;
}

double track_width = 11;
//double dt = 0.005;
double maxVelChange=6; //3
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
      curv = curvature(path, pos, look, radians((Inertial.rotation())));
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
      rightDrive.spin(fwd, (-wheels[0]*4*60)/(3.25*M_PI*5), vex::velocityUnits::rpm);
      leftDrive.spin(fwd, (-wheels[1]*4*60)/(3.25*M_PI*5), vex::velocityUnits::rpm);
    }
    else
    {
      rightDrive.spin(fwd, (wheels[0]*4*60)/(3.25*M_PI*5), vex::velocityUnits::rpm);
      leftDrive.spin(fwd, (wheels[1]*4*60)/(3.25*M_PI*5), vex::velocityUnits::rpm);
    }
    //double avg = (rightDrive.velocity(vex::velocityUnits::rpm)+leftDrive.velocity(vex::velocityUnits::rpm))/2.0;
    //printf("%d\t%f\n", close, vel);
    //printf("%f\n", look[2]);
    wait(10, msec);
  }
  rightDrive.stop(hold);
  leftDrive.stop(hold);
  return true;
}



void PIDTurn(double x, double y, bool reverse, bool left)
{
  resetDriveSensors=true;
  double ang = degree(atan2(x-pos[0], y-pos[1]));
  desiredTurnValue=ang;
  if (reverse&&left)
  {
    desiredTurnValue-=180;
  }
  else if (reverse)
  {
    desiredTurnValue+=180;
  }
  drivePID();
  // resetDriveSensors=true;
  // desiredValue=((distanceP(pos[0], pos[1], x, y)*4*360)/(M_PI*3.25*5));
  // drivePID();  
}

void PIDTurn(double angle)
{
  resetDriveSensors=true;
  desiredTurnValue=angle;
  drivePID();
}
void PIDMove (double length, double timeout=5, brakeType chooseBrakeType=hold)
{
  desiredLength=length;
  dist(timeout, chooseBrakeType);
}
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





bool enableOdom=true;
int odom()
{
  while (enableOdom)
  {
    getCurrLoc();
    // Controller.Screen.setCursor(0, 0);
    // Controller.Screen.clearLine();
    // Controller.Screen.print("%d %d %d", (int)pos[0], (int)pos[1], (int)curveError);
    //printf("%d\t%d\n", (int)pos[0], (int)pos[1]);
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
double shootPoint[] = {0,0};
double shootDist = 10;
double shootTime = 0;
bool singlePiston = false;
int loadCata()
{
  while (autonCata)
  {
    if (load)
    {
      catapult.spin(reverse, 100, vex::velocityUnits::pct);
    }
    // if (cataSense.angle(deg)<103&&load&&loader)
    // {
    //   catapult.stop(hold);
    //   load=!load;
    // }
    if (cataSense.angle(deg)>100&&load)
    {
      catapult.spin(reverse, 20, vex::velocityUnits::pct);
    }
    if (cataSense.angle(deg)>117&&load)
    {
      catapult.stop(hold);
      load=!load;
    }
    if (fire&&intakeSense.objectDistance(mm)>170)
    {
      if (shootTime!=0)
      {
        wait(shootTime,msec);
      }
      if ((shootPoint[0]==0 && shootPoint[1]==0) || (shootPoint[0]!=0 && shootPoint[1]!=0&&distanceP(pos[0], pos[1], shootPoint[0], shootPoint[1])<shootDist))
      {
        catapult.spin(reverse, 100, vex::velocityUnits::pct);
        // wait(70, msec);
        // if (singlePiston)
        // {
        //   cataBoost.set(true);
        // }
        // else 
        // {
        //   cataBoost.set(true);
        //   cataBoost2.set(true);
        // }
        }
      }
    if (cataSense.angle()>119)
    {
       if (singlePiston)
        {
          cataBoost.set(true);
        }
        else 
        {
          cataBoost.set(true);
          cataBoost2.set(true);
        }
    }
    if (cataSense.angle(deg)<50&&fire)
    {
      catapult.stop(coast);
      cataBoost.set(false);
      cataBoost2.set(false);
      fire=!fire;
      load=true;
    }
    vex::task::sleep(20);
  }
  return 0;
}



void autonomous(void) {
  // desiredPos[0]=48;desiredPos[1]=48;
  vex::task odometry(odom);
  vex::task autonCatapult(loadCata);
  // PID(48, 48);Z
  // desiredLength=48;
  // dist();


  //////////////////////////LEFT/////////////////////////


  // intake.spin(forward, 600, rpm);
  // intake.spinFor(forward, 1200, degrees, 600, rpm);

  singlePiston=false;
  cataReduce=false;
  load=true;
  PIDMove(-4);
  intake.spinFor(forward, 250, degrees, 600, rpm);
  PIDMove(10);
  PIDTurn(-28, 100, false, true);
  fire=true;
  wait(200, msec);
  PIDTurn(16, 19, false, false);
  // PIDMove(15);
  // PIDTurn(-17, 100, false, true);
  // PIDMove(-4);
  PIDMove(18);
  wait(20, msec);
  
  // PIDMove(-5);
  // PIDTurn(5, 15, true, false);
  PIDTurn(148);
  intakeLifter.set(true);
  intake.spin(reverse, 600, rpm);
  wait(150, msec);
  PIDMove(-12.5);
  //wait(100, msec);
  intakeLifter.set(false);
  wait(650, msec);
  PIDMove(6); //5
  PIDTurn(-28, 100, false, true);
  if (intakeSense.objectDistance(mm)>170)
    fire=true;
  wait(175, msec);
  // PIDTurn(16, 16, true, true);
  //printf("%f\n", Inertial.rotation());
  PIDTurn(-145);
  intakeLifter.set(true);
  wait(150, msec);
  PIDMove(-14);
  //wait(500, msec);
  intakeLifter.set(false);
  wait(300, msec);
  PIDMove(-15);
  PIDMove(-15);
  wait(400, msec);
  // PIDMove(5);
  PIDTurn(-13, 120, false, true);
  // shootTime=200;
  // leftDrive.setVelocity(600, rpm);
  // rightDrive.setVelocity(600, rpm);
  // wait(800, msec);
  PIDMove(9);
  if (intakeSense.objectDistance(mm)>170)
    fire=true;
  // rightDrive.stop();
  // leftDrive.stop();
  
  
  
  wait(15000,msec);




  // singlePiston=false;
  // cataReduce=false;
  // load=true;
  // PIDMove(-4);
  // intake.spinFor(forward, 250, degrees, 600, rpm);
  // PIDMove(10);
  // PIDTurn(16, 19, false, false);
  // PIDMove(15);
  // PIDTurn(-17, 100, false, true);
  // PIDMove(-4);
  // wait(20, msec);
  // fire=true;
  // wait(175, msec);
  // // PIDMove(-5);
  // // PIDTurn(5, 15, true, false);
  // PIDTurn(148);
  // intakeLifter.set(true);
  // intake.spin(reverse, 600, rpm);
  // wait(150, msec);
  // PIDMove(-12.5);
  // //wait(100, msec);
  // intakeLifter.set(false);
  // wait(650, msec);
  // PIDMove(5);
  // PIDTurn(-20, 100, false, true);
  // if (intakeSense.objectDistance(mm)>170)
  //   fire=true;
  // wait(175, msec);
  // // PIDTurn(16, 16, true, true);
  // //printf("%f\n", Inertial.rotation());
  // PIDTurn(-145);
  // intakeLifter.set(true);
  // wait(150, msec);
  // PIDMove(-19);
  // //wait(500, msec);
  // intakeLifter.set(false);
  // wait(150, msec);
  // PIDMove(-19);
  // wait(150, msec);
  // PIDMove(5);
  // PIDTurn(-8, 120, false, true);
  // // PIDMove(12);
  // if (intakeSense.objectDistance(mm)>170)
  //   fire=true;

  // wait(15000, msec);
  // // PIDTurn();

  // PIDTurn(-2, 15, true, false);
  // intakeLifter.set(true);
  // intake.spin(reverse, 600, rpm);
  // PIDMove(-10);
  // wait(100, msec);
  // intakeLifter.set(false);
  // // wait(100, msec);
  // // PIDMove(-4);
  // wait(200, msec);
  // // pathing(pathMain[0], false);


  // wait(15000, msec);


  /////////////////////////RIGHT////////////////////////////


  // PIDTurn(4,24,false);
  //

  // PIDTurn(4,24,false);

  // load=true;
  // shootPoint[0]=7;shootPoint[1]=24;
  // shootDist=10;
  // if (intakeSense.objectDistance(mm)>170)
  //   fire=true;
  // pathing(pathMain[0], false);
  
  // wait(100, msec);
  // //wait(200, msec);
  // // load=true;
  // //wait(350, msec);//RID
  // //pathing(pathMain[1], false);
  // PIDTurn(20, 9, true, true);
  // //PIDMove(-60);
  // pathing(pathMain[1], true);
  // PIDMove(-12.5,0.5, brake);
  // wait(350, msec);
  // intake.spinFor(forward, 500, degrees, 600, rpm);
  // PIDMove(9);
  // PIDTurn(-12,36, true, false);
  // intake.spin(reverse, 600, rpm);
  // pathing(pathMain[2], true);
  // PIDMove(-16);
  // //PIDMove(-8);
  // wait(100, msec);
  // PIDTurn(26,122, false, true);
  // shootPoint[0]=-33;shootPoint[1]=42;
  // shootDist=5;
  // fire=true;
  // PIDMove(12);
  // wait(100, msec);
  // // wait(200, msec);
  // // load=true;
  // PIDMove(-17);
  // PIDTurn(-43,0, true,false);
  // maxVelChange=2;
  // pathing(pathMain[3], true);
  // // wait(500, msec); //RID
  // // PIDTurn(-41,0, true, false);
  // //pathing(pathMain[3], true);
  // // wait(100, msec);
  // maxVelChange=6;
  // pathing(pathMain[4], false);
  // shootPoint[0]=-19;shootPoint[1]=30;
  // fire=true;
  // pathing(pathMain[1], true);
  
}


double LeftPercent = 0;
double RightPercent = 0;
double lastLeftPercent = 0;
double lastRightPercent = 0;
void leftExpo (vex::directionType type, double percentage){
  if(fabs(percentage) < 1)
    percentage = 0;
  else if(percentage >= 1)
    percentage = 2*pow(1.0359999, percentage) + 1;
  else{
    percentage = -percentage;
    percentage = 2*pow(1.0359999, percentage) + 1;
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
  if (percentage<-4&&rightDrive.velocity(pct)>0&&leftDrive.velocity(pct)>0&&fabs(LeftPercent-RightPercent)<5)
  {
    percentage=lastLeftPercent-0.8;
  }
  leftDrive.spin (type, percentage, vex::velocityUnits::pct);
  lastLeftPercent=percentage;
}

void rightExpo (vex::directionType type, double percentage){

  if(fabs(percentage) < 1)
    percentage = 0;
  else if(percentage >= 1)
    percentage = 2*pow(1.0359999, percentage) + 1;
  else{
    percentage = -percentage;
    percentage = 2*pow(1.0359999, percentage) + 1;
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

bool autonOn = false;
bool autonToggle = false;

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
  enableOdom=false;
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
        Controller.Screen.setCursor(0, 0);
        Controller.Screen.clearLine();
        Controller.Screen.print("B:%d, R:%d, A:%d", boostToggle, modeToggle, autonToggle);
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

    if (Controller.ButtonLeft.pressing())
    {
      if (!autonOn)
      {
        autonToggle = !autonToggle;
        autonOn=true;
        Controller.Screen.setCursor(0, 0);
        Controller.Screen.clearLine();
        Controller.Screen.print("B:%d, R:%d, A:%d", boostToggle, modeToggle, autonToggle);
      }
    }
    else  
    {
      autonOn=false;
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
        Controller.Screen.setCursor(0, 0);
        Controller.Screen.clearLine();
        Controller.Screen.print("B:%d, R:%d, A:%d", boostToggle, modeToggle, autonToggle);
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
    //printf("%f\n", cataSense.angle());
    if (Controller.ButtonR2.pressing()&& intakeSense.objectDistance(mm)>170)//&& intakeSense.objectDistance(mm)>170
    {
      intakeToggle=false;
      reload=false;
      catapult.spin(reverse, 100, vex::velocityUnits::pct);
      // wait (50, msec);
      
      // if (true)
      // {
      //   wait(140, msec);
      //   cataBoost.set(false);
      // }
    }
    if (cataSense.angle()>119)
    {
      if (boostToggle)
        cataBoost.set(true);
      if (autonToggle)
      {
        cataBoost.set(true);
        cataBoost2.set(true);
      }
    }
    // if (reload&&loaderToggle&&cataSense.angle(deg)>116.5)
    // {
    //   catapult.stop(hold);
      
    // }
    if (reload && cataSense.angle(deg)>100)
    {
       catapult.spin(reverse, 20, vex::velocityUnits::pct);
    }
    if (reload && cataSense.angle(deg)>117)//93
    {
      catapult.stop(hold);
    }
    else if (!reload && cataSense.angle(deg)<50)
    {
      intakeToggle=false;
      catapult.stop(coast);
      cataBoost.set(false);
      cataBoost2.set(false);

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
