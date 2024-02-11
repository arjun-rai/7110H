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
-82.604,-139.743,120,0
-82.389,-137.754,120
-82.166,-135.767,120
-81.934,-133.78,120
-81.693,-131.795,120
-81.443,-129.811,120
-81.183,-127.828,120
-80.913,-125.846,120a
-80.634,-123.865,120
-80.345,-121.886,120
-80.047,-119.909,120
-79.74,-117.932,120
-79.425,-115.957,120
-79.102,-113.984,120
-78.773,-112.011,120
-78.438,-110.039,120
-78.1,-108.068,120
-77.759,-106.097,120
-77.417,-104.127,120
-77.077,-102.156,120
-76.74,-100.184,120
-76.407,-98.212,120
-76.082,-96.239,120
-75.764,-94.264,120
-75.456,-92.288,120
-75.158,-90.311,120
-74.872,-88.331,120
-74.598,-86.35,120
-74.336,-84.367,120
-74.086,-82.383,120
-73.849,-80.397,120
-73.625,-78.41,120
-73.412,-76.421,120
-73.212,-74.431,120
-73.022,-72.44,120
-72.844,-70.448,120
-72.676,-68.455,120
-72.519,-66.461,120
-72.371,-64.467,120
-72.232,-62.472,120
-72.102,-60.476,120
-71.98,-58.48,120
-71.867,-56.483,120
-71.761,-54.486,120
-71.662,-52.488,120
-71.57,-50.49,120
-71.484,-48.492,120
-71.404,-46.494,120
-71.33,-44.495,120
-71.263,-42.496,120
-71.2,-40.497,120
-71.142,-38.498,120
-71.09,-36.499,120
-71.041,-34.499,120
-70.998,-32.5,120
-70.958,-30.5,120
-70.923,-28.5,120
-70.891,-26.501,120
-70.863,-24.501,120
-70.839,-22.501,120
-70.803,-18.943,120,0
-70.803,-18.943,0,0
#PATH-POINTS-START Path
-70.182,-20.185,120,0
-70.611,-22.138,120
-71.038,-24.092,120
-71.462,-26.047,120
-71.883,-28.002,120
-72.303,-29.957,120
-72.722,-31.913,120
-73.141,-33.869,120
-73.56,-35.824,120
-73.981,-37.78,120
-74.405,-39.734,120
-74.833,-41.688,120
-75.266,-43.64,120
-75.706,-45.591,120
-76.155,-47.54,120
-76.616,-49.486,120
-77.089,-51.43,120
-77.578,-53.369,120
-78.086,-55.303,120
-78.615,-57.232,120
-79.17,-59.154,120
-79.753,-61.067,120
-80.368,-62.97,120
-81.019,-64.861,120
-81.712,-66.737,120
-82.448,-68.596,120
-83.233,-70.436,120
-84.069,-72.253,120
-84.959,-74.043,120
-85.907,-75.804,120
-86.914,-77.532,120
-87.98,-79.225,120
-89.103,-80.879,120
-90.283,-82.494,120
-91.518,-84.067,120
-92.805,-85.598,120
-94.14,-87.087,120
-95.52,-88.534,120
-96.942,-89.941,120
-98.401,-91.309,120
-99.893,-92.64,120
-101.417,-93.936,120
-102.967,-95.199,120
-104.541,-96.432,120
-106.139,-97.635,120
-107.755,-98.813,120
-109.39,-99.966,120
-111.04,-101.096,120
-112.704,-102.205,120
-114.381,-103.295,120
-116.07,-104.367,120
-117.768,-105.423,120
-119.476,-106.463,120
-121.193,-107.49,120
-122.916,-108.504,120
-124.648,-109.505,120
-126.385,-110.496,120
-128.128,-111.477,120
-129.876,-112.449,120
-131.629,-113.412,120
-133.222,-114.279,120,0
-133.222,-114.279,0,0
#PATH-POINTS-START Path
-10.869,-147.817,120,0
-12.869,-147.815,120
-14.869,-147.811,120
-16.869,-147.803,120
-18.869,-147.791,120
-20.869,-147.774,120
-22.869,-147.753,120
-24.868,-147.726,120
-26.868,-147.694,120
-28.868,-147.655,120
-30.867,-147.61,120
-32.867,-147.558,120
-34.866,-147.498,120
-36.865,-147.43,120
-38.863,-147.353,120
-40.861,-147.268,120
-42.859,-147.174,120
-44.856,-147.07,120
-46.853,-146.955,120
-48.849,-146.83,120
-50.845,-146.694,120
-52.839,-146.546,120
-54.833,-146.387,120
-56.825,-146.215,120
-58.817,-146.03,120
-60.807,-145.831,120
-62.796,-145.62,120
-64.783,-145.393,120
-66.768,-145.152,120
-68.752,-144.896,120
-70.733,-144.625,120
-72.712,-144.336,120
-74.689,-144.032,120
-76.663,-143.712,120
-78.634,-143.372,120
-80.602,-143.016,120
-82.567,-142.642,120
-84.527,-142.248,120
-86.484,-141.835,120
-88.437,-141.403,120
-90.385,-140.951,120
-92.329,-140.478,120
-94.267,-139.984,120
-96.2,-139.471,120
-98.126,-138.933,120
-100.047,-138.375,120
-101.961,-137.796,120
-103.868,-137.192,120
-105.767,-136.565,120
-107.659,-135.917,120
-109.542,-135.245,120
-111.417,-134.547,120
-113.282,-133.826,120
-115.139,-133.082,120
-116.985,-132.312,120
-118.82,-131.518,120
-120.645,-130.699,120
-122.458,-129.856,120
-124.259,-128.986,120
-126.048,-128.092,120
-127.825,-127.174,120
-129.588,-126.23,120
-131.337,-125.26,120
-133.073,-124.266,120
-134.794,-123.248,120
-136.5,-122.204,120
-138.19,-121.135,120
-139.865,-120.041,120
-141.524,-118.925,120
-143.167,-117.784,120
-144.791,-116.617,120
-146.399,-115.428,120
-147.99,-114.216,120
-149.563,-112.981,120
-151.118,-111.723,120
-152.654,-110.442,120
-154.171,-109.14,120
-155.671,-107.816,120
-157.444,-106.205,120,0
-157.444,-106.205,0,0)";
std::vector<std::vector<pathPoint>> pathMain = convert_to_path(path_str);
std::vector<double> finSpeed = {0, 0, 0};
std::vector<double> startSpeed = {200, 200, 100};
//