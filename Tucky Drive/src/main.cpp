/*--------------------------------------------------------------------------------------------*/
/*                                                                                            */
/*    Module:       main.cpp                                                                  */
/*    Author:       Tucky J "The Rock" "The Johnson" "The Johnson Rock/Rock Johnson" Johnson  */
/*    Created:      22 Sept 2021                                                              */
/*    Description:  Family recipe for a classic Johnson Dubuccine Alfredo                     */
/*                                                                                            */
/*--------------------------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
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

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//custom functions
 
//end of custom functions

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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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
  // User control code here, inside the loop
  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);
  Roller.setStopping(brake);
  Roller.setVelocity(100, percent);
  Moby.setStopping(hold);
  Moby.setVelocity(100, percent);
  Wheel.setStopping(coast);
  Wheel.setVelocity(100, percent);
  bool ClampToggle = false;
  bool LiftToggle1 = false;
  bool LiftToggle2 = true;
  bool WheelToggle = false;
  bool DriveToggle = false;
  bool ClampLatch = false;
  bool LiftLatch1 = false;
  bool LiftLatch2 = false;
  bool WheelLatch = false;
  bool WheelRevLatch = false;
  bool WheelVelLatch = false;
  bool TurnLatch = false;
  bool DriveLatch = false;
  float WheelVel = 100;
  float WheelVel2 = 100;
  float TurnVel = 60;

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    
    // Deadband stops the motors when Axis values are close to zero.
    float deadband = 10;

    while (true) {

    // Get the velocity percentage of the left motor. (Axis3)
    int LeftSpeed = Controller.Axis3.position() * 1.0;
    // Get the velocity percentage of the right motor. (Axis2)
    int RightSpeed = Controller.Axis3.position() * 1.0;

    int LeftTurnSpeed1 = RightSpeed + (abs(Controller.Axis1.position())*(TurnVel/100));
    int RightTurnSpeed1 = LeftSpeed - (abs(Controller.Axis1.position())*(TurnVel/100));
    int LeftTurnSpeed2 = LeftSpeed + (abs(Controller.Axis1.position())*(TurnVel/100));
    int RightTurnSpeed2 = RightSpeed - (abs(Controller.Axis1.position())*(TurnVel/100));

    // Set the speed of the left motor. If the value is less than the deadband,
    // set it to zero.
    
    if (abs(LeftSpeed) < deadband&&Controller.Axis1.position() <! deadband&&Controller.Axis1.position() > deadband) {
      // Set the speed to zero.
      LeftDriveSmart.setVelocity(0, percent);
    } else if(Controller.Axis1.position() <! deadband&&Controller.Axis1.position() > deadband){
      // Set the speed to leftMotorSpeed
      LeftDriveSmart.setVelocity(LeftSpeed, percent);
    }

    // Set the speed of the right motor. If the value is less than the deadband,
    // set it to zero.
    if (abs(RightSpeed) < deadband&&Controller.Axis1.position() <! deadband&&Controller.Axis1.position() > deadband) {
      // Set the speed to zero
      RightDriveSmart.setVelocity(0, percent);
    } else if(Controller.Axis1.position() <! deadband&&Controller.Axis1.position() > deadband){
      // Set the speed to rightMotorSpeed
      RightDriveSmart.setVelocity(RightSpeed, percent);
    }

    if(Controller.Axis1.position() < deadband){
      RightDriveSmart.setVelocity(LeftTurnSpeed1, percent);
      LeftDriveSmart.setVelocity(RightTurnSpeed1, percent);
    }

    if(Controller.Axis1.position() > deadband){
      LeftDriveSmart.setVelocity(LeftTurnSpeed2, percent);
      RightDriveSmart.setVelocity(RightTurnSpeed2, percent);
    }

    // Spin both motors in the forward direction.
    // if(LeftSpeed < 0 && RightSpeed < 0){
    //   LeftDriveSmart.setVelocity(LeftSpeed * 0.5, percent);
    //   RightDriveSmart.setVelocity(RightSpeed * 0.5, percent);
    // }
    // if(LeftSpeed != 0 || RightSpeed != 0 || LeftTurnSpeed1 != 0 || RightTurnSpeed1 != 0 || LeftTurnSpeed2 != 0 || RightTurnSpeed2 != 0)
    //   Controller.rumble("-");
    LeftDriveSmart.spin(forward);
    RightDriveSmart.spin(forward);
    
    Wheel.setVelocity(WheelVel, percent);
    
    if(Controller.ButtonL2.pressing()){
      Roller.spin(reverse);
    }else if(Controller.ButtonR2.pressing()){
      Roller.spin(forward);
    }else{
      Roller.stop();
    }

    // if(Controller.ButtonX.pressing()){
    //   Moby.spin(reverse);
    // }else if(Controller.ButtonB.pressing()){
    //   Moby.spin(forward);
    // }else{
    //   Moby.stop();
    // }

    if(Controller.ButtonR1.pressing()){
      if(ClampLatch == false){
        ClampToggle = !ClampToggle;
        ClampLatch = true;
      }
    }else{
      ClampLatch = false;
    }
    Clamp1.set(ClampToggle);
    Clamp2.set(ClampToggle);

    if(Controller.ButtonUp.pressing()){
      if(LiftLatch1 == false){
        LiftToggle1 = !LiftToggle1;
        LiftLatch1 = true;
      }
    }else{
      LiftLatch1 = false;
    }
    Lift1.set(LiftToggle1);

    if(Controller.ButtonL1.pressing()){
      if(LiftLatch2 == false){
        LiftToggle2 = !LiftToggle2;
        LiftLatch2 = true;
      }
    }else{
      LiftLatch2 = false;
    }
    Lift2.set(LiftToggle2);
    Lift3.set(LiftToggle2);

    if(Controller.ButtonX.pressing()){
      if(WheelLatch == false){
        WheelToggle = !WheelToggle;
        WheelLatch = true;
        Controller.rumble(".");
      }
    }else{
      WheelLatch = false;
    }
    if(WheelToggle == true){
      Wheel.spin(forward);
    }else{
      Wheel.stop();
    }

    if(Controller.ButtonA.pressing()){
      if(WheelRevLatch == false){
        if(WheelVel == 70)
          WheelVel = WheelVel2;
        else if(WheelVel == WheelVel2)
          WheelVel = 70;
        WheelRevLatch = true;
      }
    }else{
      WheelRevLatch = false;
    }  

    if(Controller.ButtonY.pressing()){
      if(WheelVelLatch == false){
        if(WheelVel == 100){
          WheelVel = 20;
          WheelVel2 = 20;
        }else{
          WheelVel = 100;
          WheelVel2 = 100;
        }
        WheelVelLatch = true;
        Controller.rumble(".");
      }
    }else{
      WheelVelLatch = false;
    }

    if(Controller.ButtonB.pressing()){
      if(TurnLatch == false){
        if(TurnVel == 60){
          TurnVel = 30;
        }else{
          TurnVel = 60;
        }
        TurnLatch = true;
        Controller.rumble(".");
      }
    }else{
      TurnLatch = false;
    }

    if(Controller.ButtonDown.pressing()){
      if(DriveLatch == false){
        DriveToggle = !DriveToggle;
        DriveLatch = true;
        Controller.rumble(".");
      }
    }else{
      DriveLatch = false;
    }
    if(DriveToggle == false){
      LeftDriveSmart.setStopping(brake);
      RightDriveSmart.setStopping(brake);
    }else{
      LeftDriveSmart.setStopping(coast);
      RightDriveSmart.setStopping(coast);
    }

    wait(5, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
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
