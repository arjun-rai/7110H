#include "vex.h"
#include "paths.h"
#include "pure-pursuit.h"
std::vector<pathPoint> convert_to_path(std::string str)
{
    std::vector<double> vect;
    std::stringstream ss(str);

    for (int i; ss >> i;) {
        vect.push_back(i);    
        if (ss.peek() == ',')
            ss.ignore();
    }
    std::vector<pathPoint> path;
    for (int i =0;i<vect.size();i+=3)
    {
      pathPoint temp = point(vect[i], vect[i+1]);
      path.push_back(temp);
    }
    return path;
}
std::vector<std::vector<pathPoint>> pathMain = {
  {
    convert_to_path(std::string "")
  }
};