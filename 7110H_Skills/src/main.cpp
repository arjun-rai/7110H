#include "vex.h"
#include "drive-controller.h"
#include "pure-pursuit.h"
#include "pid.h"
#include "paths.h"
using namespace vex;
competition Competition;
extern std::vector<std::vector<pathPoint>> pathMain;
extern std::vector<double> finSpeed;
extern std::vector<double> startSpeed;
extern double pos[];
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
  for (int i =0;i<pathMain.size(); i++)
  {
    // pathMain[i] = inject(pathMain[i]);
    // pathMain[i] = smooth(pathMain[i]);
    pathMain[i].pop_back();
    curv_func(pathMain[i]);
    // std::cout << pathMain[i][0].x;
    speed_func(pathMain[i], startSpeed[i], finSpeed[i]);
  }

 
  

  //shows button, allows user to select button and then stops once submit is pressed
  //autonNum = autonSelector();
  Brain.Screen.clearScreen(vex::black);
  //Inertial.calibrate(2000);
  Inertial.calibrate();
  while (Inertial.isCalibrating()) {
  wait(100, msec);
  }
  
  Inertial.resetRotation();
  Inertial.setRotation(-90, degrees);
  
  parallelEncoder.resetPosition();
  perpendicularEncoder.resetPosition();
  driveBrake(coast);
  printf("yes!\n");
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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




void autonomous(void) {
  vex::task odometry(odom);
  intake.spin(reverse, 200, rpm);
  pathing(pathMain[0], true, true, 15*2.54, 1800);
  PIDMove(12);
  PIDTurn(-66); //-77
  PIDMove(-7, 0.6);
  wingsBackLeft.set(true);
  double angle = Inertial.rotation();
  timer t = timer();
  motor1.spin(reverse, 150, rpm);
  motor2.spin(reverse, 150, rpm);
  waitUntil(t.time(sec)>20);
  // wait(27, sec);
  motor1.stop();
  motor2.stop();
  wait(250, msec);
  Inertial.calibrate();
  waitUntil(!Inertial.isCalibrating());
  Inertial.setRotation(angle, degrees);


  wingsBackLeft.set(false);
  pathing(pathMain[1], false, false, 18*2.54, 3000); //18
  wings.set(true);
  intake.spin(fwd, 200, rpm);
  pathing(pathMain[2], false, true, 40*2.54, -3500);
  // PIDMove(54);
  PIDMove(-7);
  wings.set(false);
  PIDTurn(70, true);
  pathing(pathMain[3], false, true, 16*2.54, 2600);
  wings.set(true);
  pathing(pathMain[4], false, true, 20*2.54, 3500);
  wings.set(false);
  PIDMove(-10);
  PIDTurn(-340);
  PIDMove(-30, 1);
  pos[0]=-143.0;
  pos[1]=72.0;
  // printf("%f %f \n", pos[0], pos[1]);
  intake.stop();
  PIDMove(5);
  PIDTurn(-270);
  // PIDMove(-45);
  intake.spin(reverse, 200, rpm);
  pathing(pathMain[5], false, true, 25*2.54, 3000);
  intake.spin(fwd, 200, rpm);
  PIDTurn(30); 
  // wings.set(true);
  wingsBackLeft.set(true);
  wingsBackRight.set(true);
  pathing(pathMain[6], true, true, 20*2.54, 2000);
  wingsBackLeft.set(false);
  wingsBackRight.set(false);
  // wings.set(false);
  pathing(pathMain[7], false, true, 22*2.54, 2000);
  PIDTurn(-360+360);
  PIDMove(-95/2.54);
  // pathing(pathMain[8], true, true, 50*2.54, 2000);
  // wings.set(true);
  PIDTurn(-270+360);
  wingsBackLeft.set(true);
  wingsBackRight.set(true);
  pathing(pathMain[9], true, true, 30*2.54, 2000);
  wingsBackLeft.set(false);
  wingsBackRight.set(false);
  pathing(pathMain[10], false, true, 25*2.54, 3500);
  PIDTurn(-180+360);
  // wingsBackLeft.set(true);
  // wingsBackRight.set(true);
  wings.set(true);
  intake.spin(fwd, 200, rpm);
  pathing(pathMain[11], false, true, 22*2.54, 3000);
  wings.set(false);
  PIDMove(-25);
  // wings.set(true);
  PIDTurn(-30+360);
  leftDrive.spinFor(fwd, 7, rev, 600, rpm, false);
  rightDrive.spinFor(fwd, 7, rev, 600, rpm, false);
  wait(1000, msec);
  leftDrive.stop();
  rightDrive.stop();
  wings.set(false);
  // PIDMove(20, 1.5);
  PIDMove(-15);

  // printf("%f %f\n", pos[0], pos[1]);
  
}





bool wingsOn = false;
bool wingsToggle = false;
bool modeOn = false;
bool modeToggle = false;
bool ptoOn = false;
bool ptoToggle = false;
bool backWingsOn = false;
bool backWingsToggle = false;
bool ratchetToggle=false;
bool ratchetOn=false;
timer tMatch = timer();
void usercontrol(void) {
  vex::task odometry(odom);
  intake.spin(reverse, 200, rpm);
  pathing(pathMain[0], true, true, 15*2.54, 1800);
  PIDMove(12);
  PIDTurn(-66); //-77
  PIDMove(-7, 0.6);
  wingsBackLeft.set(true);
  double angle = Inertial.rotation();
  timer t = timer();
  motor1.spin(reverse, 150, rpm);
  motor2.spin(reverse, 150, rpm);
  waitUntil(t.time(sec)>20);
  // wait(27, sec);
  motor1.stop();
  motor2.stop();
  wait(250, msec);
  Inertial.calibrate();
  waitUntil(!Inertial.isCalibrating());
  Inertial.setRotation(angle, degrees);


  wingsBackLeft.set(false);
  pathing(pathMain[1], false, false, 18*2.54, 3000); //18
  wings.set(true);
  intake.spin(fwd, 200, rpm);
  pathing(pathMain[2], false, true, 40*2.54, -3500);
  // PIDMove(54);
  PIDMove(-7);
  wings.set(false);
  PIDTurn(70, true);
  pathing(pathMain[3], false, true, 16*2.54, 2600);
  wings.set(true);
  pathing(pathMain[4], false, true, 20*2.54, 3500);
  wings.set(false);
  PIDMove(-10);
  PIDTurn(-340);
  PIDMove(-30, 1);
  // pos[0]=-143.0;
  // pos[1]=72.0;
  // // printf("%f %f \n", pos[0], pos[1]);
  // intake.stop();
  // PIDMove(5);
  // PIDTurn(-270);
  // // PIDMove(-45);
  // intake.spin(reverse, 200, rpm);
  // pathing(pathMain[5], false, true, 25*2.54, 3000);
  // intake.spin(fwd, 200, rpm);
  // PIDTurn(30); 
  // // wings.set(true);
  // wingsBackLeft.set(true);
  // wingsBackRight.set(true);
  // pathing(pathMain[6], true, true, 20*2.54, 2000);
  // wingsBackLeft.set(false);
  // wingsBackRight.set(false);
  // // wings.set(false);
  // pathing(pathMain[7], false, true, 22*2.54, 2000);
  // PIDTurn(-360+360);
  // PIDMove(-95/2.54);
  // // pathing(pathMain[8], true, true, 50*2.54, 2000);
  // // wings.set(true);
  // PIDTurn(-270+360);
  // wingsBackLeft.set(true);
  // wingsBackRight.set(true);
  // pathing(pathMain[9], true, true, 30*2.54, 2000);
  // wingsBackLeft.set(false);
  // wingsBackRight.set(false);
  // pathing(pathMain[10], false, true, 25*2.54, 3500);
  // PIDTurn(-180+360);
  // // wingsBackLeft.set(true);
  // // wingsBackRight.set(true);
  // wings.set(true);
  // intake.spin(fwd, 200, rpm);
  // pathing(pathMain[11], false, true, 22*2.54, 3000);
  // wings.set(false);
  // PIDMove(-25);
  // // wings.set(true);
  // PIDTurn(-30+360);
  // leftDrive.spinFor(fwd, 7, rev, 600, rpm, false);
  // rightDrive.spinFor(fwd, 7, rev, 600, rpm, false);
  // wait(1000, msec);
  // leftDrive.stop();
  // rightDrive.stop();
  // wings.set(false);
  // // PIDMove(20, 1.5);
  // PIDMove(-15);

  // pos[0]=-143.0;
  // pos[1]=72.0;
  // // printf("%f %f \n", pos[0], pos[1]);
  // intake.stop();
  // PIDMove(5);
  // PIDTurn(-270);
  // // PIDMove(-45);
  // intake.spin(reverse, 200, rpm);
  // pathing(pathMain[5], false, true, 25*2.54, 3000);
  // intake.spin(fwd, 200, rpm);
  // PIDTurn(0); 
  // User control code here, inside the loop
  motor1.setBrake(hold);
  motor2.setBrake(hold);
  while (1) {
    // if (tMatch.time(sec)>103.5)
    // {
    //   ratchet.set(true);
    // }
    driveBrake(coast);
    // rightExpo(forward, (Controller.Axis3.value() - Controller.Axis1.value()), maxSpeed);
    // leftExpo(forward, (Controller.Axis3.value() + Controller.Axis1.value()), maxSpeed);
    // rightExpo(forward, (Controller.Axis2.value()));
    // leftExpo(forward, (Controller.Axis3.value()));
    curvatureDrive(Controller.Axis3.value()/127.0, Controller.Axis1.value()/127.0);
    if (Controller.ButtonL1.pressing())
    {
      if (modeToggle){
        // ratchetToggle=true;
        // motor1.spin(fwd, 200, rpm);
        // motor2.spin(fwd, 200, rpm);
      }
      else {
        intake.spin(fwd, 200, rpm);
      }
    }
    else if (Controller.ButtonL2.pressing())
    {
      if (modeToggle)
      {
      motor1.spin(reverse, 150, rpm);
      motor2.spin(reverse, 150, rpm);
      }
      else {
        intake.spin(reverse, 200, rpm);
      }
    }
    else {
      // motor1.stop();
      // motor2.stop();
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

    if (Controller.ButtonR1.pressing())
    {
      if (!modeOn)
      {
        modeToggle=!modeToggle;
        modeOn=true;
      }
    }
    else {
      modeOn=false;
    }


     if (Controller.ButtonR2.pressing())
    {
      if (!backWingsOn)
      {
        backWingsToggle=!backWingsToggle;
        backWingsOn=true;
      }
    }
    else {
      backWingsOn=false;
    }
    if (backWingsToggle)
    {
      wingsBackRight.set(true);
      wingsBackLeft.set(true);
    }
    else {
      wingsBackRight.set(false);
      wingsBackLeft.set(false);
    }


    if (Controller.ButtonX.pressing())
    {
      if (!ratchetOn)
      {
        ratchetToggle=!ratchetToggle;
        ratchetOn=true;
      }
    }
    else {
      ratchetOn=false;
    }
    if(ratchetToggle)
    {
      ratchet.set(true);
    }
    else {
     ratchet.set(false);
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
