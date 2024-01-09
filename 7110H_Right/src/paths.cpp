#include "vex.h"
#include "paths.h"
#include "pure-pursuit.h"
std::vector<std::vector<pathPoint>> convert_to_path(std::string str)
{
    std::istringstream stream(str);
    std::string line;
    std::vector<std::vector<pathPoint> > paths;

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
const char * test = R"()";
std::vector<std::vector<pathPoint>> pathMain = convert_to_path(test);

