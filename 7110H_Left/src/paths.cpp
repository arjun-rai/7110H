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
-113.968,-147.196,120,18.000000000000004
-111.997,-147.532,120
-110.017,-147.814,120
-108.031,-148.052,120
-106.041,-148.252,120
-104.048,-148.421,120
-102.053,-148.564,120
-100.057,-148.683,120
-98.059,-148.784,120
-96.061,-148.866,120
-94.062,-148.934,120
-92.063,-148.988,120
-90.063,-149.03,120
-88.064,-149.061,120
-86.064,-149.083,120
-84.064,-149.097,120
-82.064,-149.104,120
-80.064,-149.104,120
-78.064,-149.098,120
-76.064,-149.088,120
-74.064,-149.073,120
-72.064,-149.055,120
-70.064,-149.033,120
-68.064,-149.01,120
-66.064,-148.984,120
-64.065,-148.957,120
-62.065,-148.93,120
-60.065,-148.903,120
-58.065,-148.875,120
-56.065,-148.849,120
-54.065,-148.825,120
-52.066,-148.802,120
-50.066,-148.783,120
-48.066,-148.766,120
-46.066,-148.754,120
-44.066,-148.747,120
-42.066,-148.745,120
-40.066,-148.75,120
-38.066,-148.763,120
-36.066,-148.785,120
-34.066,-148.816,120
-32.067,-148.859,120
-30.067,-148.915,120
-28.069,-148.986,120
-26.396,-149.059,120,0
-26.396,-149.059,0,0)";
std::vector<std::vector<pathPoint>> pathMain = convert_to_path(path_str);
std::vector<double> finSpeed = {0};
std::vector<double> startSpeed = {50};
//