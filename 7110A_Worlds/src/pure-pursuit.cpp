#include "vex.h"

////////////////////////////////////////////////////////////////////////////
// PATH GENERATION
///////////////////////////////////////////////////////////////////////////

pathPoint point(double x1, double y1)
{
  pathPoint p;
  p.x = x1;
  p.y = y1; 
  return p;
}

float clamp(float input, float min, float max){
  if( input > max ){ return(max); }
  if(input < min){ return(min); }
  return(input);
}


//distance between two points using coordinates
double distanceP(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x2-x1, 2)+pow(y2-y1, 2));
}

double distanceP(pathPoint p1, pathPoint p2)
{
  return distanceP(p1.x, p1.y, p2.x, p2.y);
}

double radians(double degrees){
  return degrees * M_PI/180.0;
}

double degree(double radians){
  return radians * 180.0/M_PI;
}


vect vect_(double x1, double y1, double x2, double y2)
{
  vect v;
  v.mag = distanceP(x1, y1, x2, y2);
  if (x2-x1==0)
  {
    if (y2>y1)
      v.angle=90;
    else
      v.angle=-90;
  }
  else if (y2-y1==0)
  {
    if (x2>x1)
      v.angle =0;
    else 
      v.angle = 180;
  }
  else 
    v.angle = degree(atan2(y2-y1, x2-x1));
  return v;
}

pathPoint add(pathPoint p, vect v, int t){
  double x_temp = p.x + (v.mag*cos(radians(v.angle)))*t;
  double y_temp = p.y + (v.mag*sin(radians(v.angle)))*t;
  return point(x_temp, y_temp);
}

double spacing = 0.5;
std::vector<pathPoint> inject(std::vector<pathPoint> p)
{
  std::vector<pathPoint> ret;
  for (int i =0; i<p.size()-1; i++)
  {
    vect v = vect_(p[i].x, p[i].y, p[i+1].x, p[i+1].y);
    int num = ceil(v.mag/spacing);
    v.mag = spacing;
    for (int j=0;j<num;j++)
    {
      ret.push_back(add(p[i],v,j));
    }
  }
  ret.push_back(p.at(p.size()-1));
  return ret;
}

double b = 0.95; //weight_smooth b = 0.75~0.98
double a = 1-b; //weight data: a = 1-b
double tol = 0.001;
std::vector<pathPoint> smooth(std::vector<pathPoint> p)
{
  std::vector<pathPoint> p_new = p;
  double change = tol;
  while (change>=tol)
  {
    change = 0.0;
    for (int i =1;i<p.size()-1;i++)
    {
      double aux = p_new[i].x;
      p_new[i].x += a* (p[i].x-p_new[i].x) + b*(p_new[i-1].x + p_new[i+1].x - (2.0*p_new[i].x));
      change+= fabs(aux-p_new[i].x);
      
      aux = p_new[i].y;
      p_new[i].y += a* (p[i].y-p_new[i].y) + b*(p_new[i-1].y + p_new[i+1].y - (2.0*p_new[i].y));
      change+= fabs(aux-p_new[i].y);
    }
  }
  return p_new;
}
//twice the signed area of a triangle
double area_of_triangle(pathPoint a, pathPoint b, pathPoint c)
{
  return fabs((b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x));
}

void curv_func(std::vector<pathPoint>& p)
{
  p[0].curve = 0.0001; //so no /0 error
  p[p.size()-1].curve = 0.0001;
  for (int i =1;i<p.size()-1; i++) //Menger curvature
  {
    p[i].x+=0.0001;
    p[i].y+=0.0001;
    pathPoint p1 = p[i-1];
    pathPoint p2 = p[i];
    pathPoint p3 = p[i+1];
    double temp = (4*fabs(area_of_triangle(p1, p2, p3)/2.0));
    p[i].curve = temp/(distanceP(p1,p2)*distanceP(p2, p3)*distanceP(p3, p1));
    // if (p[i].curve)
    if(p[i].curve==0) p[i].curve+=0.00001;
  }
}

double max_vel = (600*4*3.25*M_PI)/(5*60); //rpm to in/s
double turning_const = 1; //changes how fast it goes around turns
double max_accel = 150; //in/s^3  used to be 6
double starting_vel = (200*4*3.25*M_PI)/(5*60); //rpm to in/s
void speed_func(std::vector<pathPoint>& p)
{
  for (int i =0;i<p.size(); i++)
  {
    p[i].vel = fmin(max_vel, turning_const/p[i].curve);
  }
  //  for (int i =0;i<p.size(); i++)
  // {
  //   printf("%d\t%f\n", i, p[i].vel);
  //   wait(50, msec);
  // }

  p[p.size()-1].finVel =0; //end velocity
  for (int i=p.size()-2; i>=0;i--)
  {
    double d = distanceP(p[i+1], p[i]);
    p[i].finVel = fmin(p[i].vel, sqrt(pow(p[i+1].finVel, 2)+2*max_accel*d));
  }

  p[0].finVel = starting_vel;
  for (int i=1;i<p.size(); i++)
  {
    double d = distanceP(p[i-1], p[i]);
    double test = sqrt(pow(p[i-1].finVel, 2)+2*max_accel*d);
    if (test<p[i].finVel)
      p[i].finVel=test;
    else 
      break;
  }
}

////////////////////////////////////////////////////////////////////////////
// PATH FOLLOWING
////////////////////////////////////////////////////////////////////////////

int closest(double pos[], std::vector<pathPoint> p)
{
  double minDist[] = {0, distanceP(pos[0], pos[1], p[0].x, p[0].y)};
  for (int i =0;i<p.size(); i++)
  {
    double dist = distanceP(p[i].x, p[i].y,pos[0], pos[1]);
    if (dist<minDist[1])
    {
      minDist[0]=i;
      minDist[1]=dist;
    }
  }
  return minDist[0];
}

double l = 20;
//double angle = 0; //radians
int t_i=0;
void lookahead(double pos[], std::vector<pathPoint> path, double ret[])
{
  for (int i =0; i<path.size()-1; i++)
  {
    int i_ = path.size()-2-i;
    pathPoint p = path[path.size()-2-i];
    double d[] = {path[i_+1].x-p.x, path[i_+1].y-p.y};
    double f[] = {p.x-pos[0], p.y-pos[1]};
    double a = 0;
    double b =0;
    double c = -(l*l);
    for (int j = 0; j < 2; j++) {
        a += d[j] * d[j];
        b+=d[j]*f[j];
        c+=f[j]*f[j];
    }
    b*=2;
    double disc = b*b-4*a*c;
    if (disc>=0)
    {
      double t1 = (-b+sqrt(disc))/(2*a);
      double t2 = (-b-sqrt(disc))/(2*a);
      if (0<=t1 && t1<=1)
      {
        double t = t1;
        t_i = i_;
        ret[0] = p.x+t*d[0];
        ret[1] = p.y+t*d[1];
        ret[2] = t_i;
        return;
      }
      if (0<=t2 && t2<=1)
      {
        double t = t2;
        t_i = i_;
        ret[0] = p.x+t*d[0];
        ret[1] = p.y+t*d[1];
        ret[2] = t_i;
        return;
      }
    }
  }
  t_i=0;
  ret[0] = path[closest(pos, path)].x;
  ret[1] = path[closest(pos, path)].y;
}

double sign(double x) 
{
  if ((int)x==0)
  {
    return 1;
  }
  return copysign(1, x);
}

double curvature(std::vector<pathPoint> path, double pos[], double lookahead[], double angle)
{
  double a = -tan(M_PI/2-angle);
  double b = 1;
  double c = tan(M_PI/2-angle)*pos[0]-pos[1];
  double x = fabs(a*lookahead[0]+b*lookahead[1] + c)/sqrt(a*a+b*b); //point line dist
  double side = sign(sin(M_PI/2 - angle)*(lookahead[0]-pos[0]) - cos(M_PI/2-angle)*(lookahead[1]-pos[1]));
  //^ needed to figure out which side the robot is on
  double curv = (2*x)/(l*l);
  curv*=side;
  return curv;
}

void turn(double curv, double vel, double width, double ret[])
{
  ret[0] = vel*(2+curv*width)/2.0;
  ret[1] = vel*(2-curv*width)/2.0;
}

//used for rate limiting
double constrain(double input, double lastInput, double min, double max)
{
  if (input-lastInput<min)
    return min;
  if (input-lastInput>max)
    return max;
  return input-lastInput;
}





