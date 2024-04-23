#include "drive-controller.h"
#include "vex.h"


double kTurn = 0.65;
double turnRemap(double turn){
  double denom = sin(M_PI/2 * kTurn);
  double firstRemap = sin(M_PI/2 * kTurn * turn)/denom;
  return sin(M_PI/2 * kTurn * firstRemap) / denom;
}

double fastStopAccum = 0.0;
double negInertiaAccum = 0.05;
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
double kInertiaScalar =0;//0.5
double kSensitivty = 0.6;

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

  leftDrive.spin(fwd, left*12, volt);
  rightDrive.spin(fwd, right*12,volt);
  GearboxLeft.spin(reverse, left*12, volt);
  GearboxRight.spin(reverse, right*12, volt);
}


double single_prevTurn = 0.0;
double single_prevThrottle = 0.0;

double single_drive_deadband=0.1;
double single_drive_slew = 0.02;
double single_kInertiaScalar =0;//0.5
double single_kSensitivty = 0.8;

void curvatureSingleDrive(double throttle, double turn){
  bool pointTurn = false;
  double linear = throttle;
  if (fabs(throttle)<single_drive_deadband && fabs(turn)>single_drive_deadband)
  {
    linear = 0.0;
    pointTurn=true;
  }
  else if (throttle-single_prevThrottle>single_drive_slew) {
    linear = single_prevThrottle+single_drive_slew;
  }
  else if (throttle-single_prevThrottle<-(single_drive_slew*2)) {
    linear = single_prevThrottle-(single_drive_slew*2);
  }
  double remappedTurn = turnRemap(turn);
  double left, right;
  if (pointTurn){
    left = remappedTurn*fabs(remappedTurn);
    right = -remappedTurn*fabs(remappedTurn);
    // printf("%f\n", remappedTurn*fabs(remappedTurn));
  }
  else {
    double negInertiaPower = (turn-single_prevTurn)*single_kInertiaScalar;
    negInertiaAccum+=negInertiaPower;
    double angular = fabs(linear)*(remappedTurn+negInertiaAccum)*single_kSensitivty - fastStopAccum;

    right =linear; left = linear;
    //printf("%f\t%f\n", negInertiaAccum, angular);
    left += angular;
    right -=angular;
    updateAccum();
  }
  single_prevTurn=turn;
  single_prevThrottle=throttle;

  leftDrive.spin(fwd, left*12, volt);
  rightDrive.spin(fwd, right*12,volt);
  // GearboxLeft.spin(reverse, left*12, volt);
  // GearboxRight.spin(reverse, right*12, volt);
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