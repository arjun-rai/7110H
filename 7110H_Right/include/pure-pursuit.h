#include <vector>
#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

//stores data for calculations
struct pathPoint{
  double x;
  double y;
  double w = 0;
  double curve = 0;
  double dist =0;
  double vel =0;
  double finVel=0;
};

struct vect{
  double mag;
  double angle;
};

pathPoint point(double x1, double y1);
float clamp(float input, float min, float max);
//distance between two points using coordinates
double distanceP(double x1, double y1, double x2, double y2);
double distanceP(pathPoint p1, pathPoint p2);
double radians(double degrees);
double degree(double radians);
vect vect_(double x1, double y1, double x2, double y2);
pathPoint add(pathPoint p, vect v, int t);
std::vector<pathPoint> inject(std::vector<pathPoint> p);
std::vector<pathPoint> smooth(std::vector<pathPoint> p);
double area_of_triangle(pathPoint a, pathPoint b, pathPoint c);
void curv_func(std::vector<pathPoint>& p);
void speed_func(std::vector<pathPoint>& p, double finSpeed);
int closest(double pos[], std::vector<pathPoint> p);
void lookahead(double pos[], std::vector<pathPoint> path, double ret[]);
double sign(double x);
double curvature(std::vector<pathPoint> path, double pos[], double lookahead[], double angle);
void turn(double curv, double vel, double width, double ret[]);
double constrain(double input, double lastInput, double min, double max);
bool pathing(std::vector<pathPoint> path, bool backwards, bool stop=true);

#endif



