#ifndef PID_H
#define PID_H
int turnPID();
int dist(double timeout, vex::brakeType chooseBrakeType);
void PIDTurn(double angle);
void PIDMove (double length, double timeout=5, vex::brakeType chooseBrakeType=vex::brakeType::hold);

#endif