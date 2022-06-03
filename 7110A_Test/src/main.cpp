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
class Button
{
  public:
    int x, y,  width, height;
    std::string text;
    std::string buttonColor, textColor;
    bool pressed;
    Button(int x, int y, int width, int height, std::string text, std::string buttonColor, std::string textColor)
    {
      this->x = x;
      this->y = y;
      this->width = width;
      this->height = height;
      this->text = text;
      this->buttonColor = buttonColor;
      this->textColor = textColor;
      this->pressed=false;
    }
    
  //displays button
  void show()
  {
    //Brain.Screen.clearScreen(vex::white);
    if (!pressed)
    {
      Brain.Screen.setFillColor(buttonColor.c_str());
      Brain.Screen.setPenColor(buttonColor.c_str());
    }
    else 
    {
      Brain.Screen.setFillColor("#2EFF00");
      Brain.Screen.setPenColor("#2EFF00");
    }
    Brain.Screen.drawRectangle(x, y, width, height);
    Brain.Screen.setPenColor(textColor.c_str());
    Brain.Screen.printAt(x+(width/2.0)- Brain.Screen.getStringWidth(text.c_str())/2.0, y+Brain.Screen.getStringHeight(text.c_str())/2.0 +height/2.0, false, text.c_str());
  }
  //checks if the button is pressed;
  bool isPressed()
  {
    if (Brain.Screen.pressing() && Brain.Screen.xPosition()>=x && Brain.Screen.xPosition()<=x+width 
    && Brain.Screen.yPosition() >=y && Brain.Screen.yPosition()<=y+height)
    {
      
        return true;
    }
    
    return false;
  }

  
};
int autonSelector()
{
  Button autonButtons[] = {
  Button(10, 10, 150, 50, "Auton Red 1", "#FF0000", "#FFFFFF"),
  Button(170, 10, 150, 50, "Auton Red 2", "#FF0000", "#FFFFFF"),
  Button(330, 10, 150, 50, "Auton Red 3", "#FF0000", "#FFFFFF"),
  Button(10, 70, 150, 50, "Auton Blue 1", "#0000FF", "#FFFFFF"),
  Button(170, 70, 150, 50, "Auton Blue 2", "#0000FF", "#FFFFFF"),
  Button(330, 70, 150, 50, "Auton Blue 3", "#0000FF", "#FFFFFF"),
  Button(10, 130, 480, 50, "Submit", "#2EFF00", "#FFFFFF")
  };
  bool chosenOn[] = {false, false, false, false, false, false, false};
  bool chosen[] = {false, false, false, false, false, false, false};
  bool submitted = false;
  int index = -1;
  int len = sizeof(autonButtons)/sizeof(autonButtons[0]);
  while (!submitted)
  {
    for (int i =0;i<len; i++)
    {
      Button b = autonButtons[i];
      if (autonButtons[i].isPressed() && (index==i || index==-1) && i!=len-1)
      {
        if (!chosenOn[i])
        {
          chosen[i] = !chosen[i];
          chosenOn[i]=true;
          if (chosen[i])
          {
            autonButtons[len-1] = Button(10, 130, 480, 50, "Submit " + b.text, "#2EFF00", "#FFFFFF");
            autonButtons[i].pressed=true;
            index =i;
          }
          else
          {
            autonButtons[len-1] = Button(10, 130, 480, 50,"Submit", "#2EFF00", "#FFFFFF");
            autonButtons[i].pressed=false;
            index =-1;
          }
        }
      }
      else
      {
        chosenOn[i] = false;
      }
      //once submit is pressed and there is an auton selected, break out of the loop
      if (i==len-1 && autonButtons[i].isPressed() && index!=-1)
      {
        return index;
        submitted = true;
        break;
      }
      b.show();
    }
    vex::wait(20,msec);
  }
  return -1;
}
//auton buttons with the text, and hex codes for colors

int autonNum =-1;
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
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




double loc =23; //inches  dist-1
bool enableProfile = true;
int profile()
{
  if (loc>=31)
  {
    double maxRpm = 200;
    double distForAccel = 16; //inches

    double maxV = maxRpm*((M_PI*3.25)/60.0); //inches per second
    double a = (maxV*maxV)/(2.0*distForAccel); //inch/s^2  
    double timeForConst = (loc-2*distForAccel)/maxV;
    double timeForAccel = ((loc - (timeForConst*maxV))/(0.5*maxV))/2.0;
    int size = (timeForAccel*1000)/10.0;
    double vel[size];
    //printf("%f %f %f\n", timeForAccel, timeForConst, maxV);
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
      wait(7, msec);
    }

    wait(timeForConst, sec);

    for (int i =size-1;i>=0; i--)
    {
      double rpm = ((vel[i])/(M_PI*3.25))*60;
      rightDrive.spin(fwd, rpm, vex::velocityUnits::rpm);
      leftDrive.spin(fwd, rpm, vex::velocityUnits::rpm);
      wait(7, msec);
    }
    rightDrive.stop();
    leftDrive.stop();
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
    //printf("%f %f %f\n", timeForAccel, timeForConst, maxV);
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

    for (int i =size/2.0-1;i>=0; i--)
    {
      double rpmVal = ((vel[i])/(M_PI*3.25))*60;
      //double actual =(rightDrive.velocity(vex::velocityUnits::rpm)+leftDrive.velocity(vex::velocityUnits::rpm))/2.0;
      rightDrive.spin(fwd, rpmVal, vex::velocityUnits::rpm);
      leftDrive.spin(fwd, rpmVal, vex::velocityUnits::rpm);
      wait(10, msec);
    }
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

void autonomous(void) {
  vex::task prof(profile);
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
