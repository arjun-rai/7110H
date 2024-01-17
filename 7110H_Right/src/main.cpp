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
  //  for (int i=0;i<pathMain[0].size(); i++)
  // {
  //   printf("%f\t%f\n", pathMain[0][i].x, pathMain[0][i].y);
  //   wait(20, msec);
  // }
  // for (int i=0;i<pathMain[0].size(); i++)
  // {
  //   printf("%f\n", pathMain[0][i].finVel);
  //   wait(20, msec);
  // }
  // std::cout << pathMain[0].size();
  // motor1.spin(reverse, 100, rpm);
  // motor2.spin(reverse, 100, rpm);
  // printf("%f %f\n", pathMain[0][pathMain[0].size()].x, pathMain[0][pathMain[0].size()].y);
  vex::task odometry(odom);
  motor1.spin(reverse, 40, rpm);
  motor2.spin(reverse, 40, rpm);
  wait(700, msec);
  PIDMove(6);
  // wait(500, msec);
  pathing(pathMain[0], true, false);
  motor1.stop();
  motor2.stop();
  wingsBackLeft.set(true);
  wait(500, msec);
  pathing(pathMain[1], true, true);
  wingsBackLeft.set(false);
  PIDMove(9);
 
  PIDTurn(20);
  // motor1.spin(fwd, 100, rpm);
  // motor2.spin(fwd, 100, rpm);
  // wings.set(true);
  PIDMove(23, 0.6);
  // wings.set(false);
  PIDMove(-12);

  PIDTurn(-67);
  motor1.spin(reverse, 50, rpm);
  motor2.spin(reverse, 50, rpm);
  PIDMove(49);
  PIDTurn(-10);
  motor1.stop();
  motor2.stop();
  // pathing(pathMain[2], false, false);
  PIDMove(24);
  PIDTurn(90);
   wings.set(true);
  PIDMove(36, 0.7);
   wings.set(false);
  // pathing(pathMain[3], false, true);
  PIDMove(-14);
  PIDTurn(-148);
  PIDMove(43, 3, coast);

  // pathing(pathMain[2], false, true);
  // wingsBackLeft.set(false);
  // wait(5000, msec);
  // pathing(pathMain[2], true, true);
  // while (true)
  // {
  //   getCurrLoc();
  // }
}





bool wingsOn = false;
bool wingsToggle = false;
bool blooperOn = false;
bool blooperToggle = false;
double maxSpeed = 127;
bool ptoOn = false;
bool ptoToggle = false;
bool backWingsOn = false;
bool backWingsToggle = false;

void usercontrol(void) {
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
      if (ptoToggle){
        motor1.spin(fwd, 100, rpm);
        motor2.spin(fwd, 100, rpm);
      }
      else {
      motor1.spin(fwd, 56, rpm);
      motor2.spin(fwd, 56, rpm);
      }
    }
    else if (Controller.ButtonL2.pressing())
    {
      motor1.spin(reverse, 100, rpm);
      motor2.spin(reverse, 100, rpm);
    }
    else {
      motor1.stop();
      motor2.stop();
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
      if (!ptoOn)
      {
        ptoToggle=!ptoToggle;
        ptoOn=true;
      }
    }
    else {
      ptoOn=false;
    }
    if (ptoToggle)
    {
      pto.set(true);
      motor1.setBrake(hold);
      motor2.setBrake(hold);
    }
    else {
      pto.set(false);
      motor1.setBrake(coast);
      motor2.setBrake(coast);
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
