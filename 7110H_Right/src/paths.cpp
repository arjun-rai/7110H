#include "vex.h"
#include "paths.h"
#include "pure-pursuit.h"
std::vector<std::vector<pathPoint>> convert_to_path(std::string str)
{
    std::istringstream stream(str);
    std::string line;
    std::vector<std::vector<pathPoint>> paths;

    while (std::getline(stream, line)) {
        if (line.find("#PATH-POINTS-START Path") != std::string::npos) {
            // Start a new path for each "#PATH-POINTS-START Path"
            paths.push_back(std::vector<pathPoint>());
            continue;
        }

        std::istringstream lineStream(line);
        std::string token;

        // Skip the first two values and only consider the first two for x, y
        std::getline(lineStream, token, ','); // x
        double x = atof(token.c_str());

        std::getline(lineStream, token, ','); // y
        double y = atof(token.c_str());

        paths.back().push_back(point(x,y));
    }
    return paths;
}
const char * path_str = R"(#PATH-POINTS-START Path
96.267,-160.238,120,0
97.113,-150.274,120
96.838,-140.281,120
95.252,-130.414,120
92.432,-120.83,120
88.53,-111.634,120
83.761,-102.852,120
78.364,-94.437,120
72.561,-86.294,120
66.552,-78.301,120
60.551,-70.302,120
54.724,-62.176,120
49.23,-53.824,120
44.226,-45.171,120
39.864,-36.18,120
36.272,-26.856,120
33.543,-17.245,120
31.712,-7.425,120
30.433,8.385,120,0
30.433,8.385,0,0)";
std::vector<std::vector<pathPoint>> pathMain = convert_to_path(path_str);
std::vector<double> finSpeed = {0};
std::vector<double> startSpeed = {300};
//

