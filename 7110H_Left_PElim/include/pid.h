#ifndef PID_H
#define PID_H
int turnPID();
int dist(double timeout, vex::brakeType chooseBrakeType);
void PIDTurn(double ang, bool stopSet=true, bool rightStp=false);
void PIDMove (double length, double timeout=3, vex::brakeType chooseBrakeType=vex::brakeType::hold);
void PIDTurn(double pos[], double x, double y, bool reverse, bool left, bool stopSet);
#endif