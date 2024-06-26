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
  GearboxRight.setBrake(b);
  GearboxLeft.setBrake(b);
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
  
  // parallelEncoder.resetPosition();
  leftDrive.resetPosition();
  rightDrive.resetPosition();
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
  intake.spin(fwd, 600, rpm);
  wait(200, msec);
  intake.spin(reverse, 400, rpm);
  wait(400, msec);
  pathing(pathMain[0], true, true, 17*2.54);
  pathing(pathMain[1], true, true, 17*2.54);
  intake.stop();
  PIDTurn(-330);
  backWing.set(true);
  wait(50, msec);
  PIDTurn(-360);
  backWing.set(false);
  intake.spin(fwd, 600, rpm);
  // wings.set(false);
  wings.set(true);
  pathing(pathMain[2], false, true, 21*2.54, 1200);
  wings.set(false);
  PIDMove(-40);
  // wings.set(true);
  PIDTurn(-333);
  pathing(pathMain[2], false, true, 21*2.54, 1200);
  // wings.set(false);
  pathing(pathMain[3], true, true, 17*2.54, 2500);
  PIDTurn(-419);
  intake.spin(reverse, 200, rpm);
  PIDMove(113);
  intake.stop();
  PIDTurn(-290);
  intake.spin(fwd, 600, rpm);
  wait(350, msec);
  PIDTurn(-390);
  intake.spin(reverse, 300, rpm);
  PIDMove(50);
  wait(300, msec);
  PIDTurn(-270);
  wings.set(true);
  intake.spin(fwd, 600, rpm);
  PIDMove(100, 0.9);
  wings.set(false);
  PIDMove(-30);
  PIDTurn(-139);
  // PIDMove(102, 3, coast);

  
  
 
  // PIDMove(-40);
  // PIDMove(40);
  // printf("%f %f \n",pos[0], pos[1]);

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
bool releaseOn = false;
bool releaseToggle = false;
bool backWingOn = false;
bool backWingToggle = false;

timer tMatch = timer();
void usercontrol(void) {
  // User control code here, inside the loop
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
    curvatureSingleDrive(Controller.Axis3.value()/127.0, Controller.Axis4.value()/127.0);
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

    if (Controller.ButtonR2.pressing())
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

    if (Controller.ButtonA.pressing())
    {
      if (!backWingOn)
      {
        backWingToggle=!backWingToggle;
        backWingOn=true;
      }
    }
    else {
      backWingOn=false;
    }
    if (backWingToggle)
    {
      backWing.set(true);
    }
    else {
      backWing.set(false);
    }


    if (Controller.ButtonX.pressing()&&Controller.ButtonY.pressing())
    {
      if (!releaseOn)
      {
        releaseToggle=!releaseToggle;
        releaseOn=true;
      }
    }
    else {
      releaseOn=false;
    }
    if (releaseToggle)
    {
      release.set(true);
    }
    else {
      release.set(false);
    }
    // Controller.Screen.clearLine();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Dri: %.0f\n Int: %.0f\n" , FrontLeft.temperature(temperatureUnits::fahrenheit), intake.temperature(temperatureUnits::fahrenheit));
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
