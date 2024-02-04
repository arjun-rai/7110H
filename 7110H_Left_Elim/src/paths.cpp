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
-81.444,-129.811,120
-81.184,-127.827,120
-80.916,-125.846,120
-80.638,-123.865,120
-80.352,-121.886,120
-80.056,-119.907,120
-79.753,-117.931,120
-79.441,-115.955,120
-79.122,-113.981,120
-78.798,-112.007,120
-78.468,-110.035,120
-78.135,-108.062,120
-77.799,-106.091,120
-77.462,-104.119,120
-77.126,-102.148,120
-76.792,-100.176,120
-76.462,-98.203,120
-76.138,-96.23,120
-75.82,-94.255,120
-75.51,-92.279,120
-75.209,-90.302,120
-74.918,-88.324,120
-74.637,-86.343,120
-74.368,-84.362,120
-74.109,-82.378,120
-73.863,-80.394,120
-73.628,-78.407,120
-73.404,-76.42,120
-73.191,-74.431,120
-72.99,-72.441,120
-72.799,-70.451,120
-72.619,-68.459,120
-72.449,-66.466,120
-72.288,-64.472,120
-72.138,-62.478,120
-71.995,-60.483,120
-71.862,-58.488,120
-71.736,-56.492,120
-71.619,-54.495,120
-71.509,-52.498,120
-71.406,-50.501,120
-71.309,-48.503,120
-71.22,-46.505,120
-71.137,-44.507,120
-71.06,-42.508,120
-70.988,-40.51,120
-70.922,-38.511,120
-70.861,-36.512,120
-70.805,-34.512,120
-70.754,-32.513,120
-70.707,-30.514,120
-70.665,-28.514,120
-70.627,-26.514,120
-70.594,-24.515,120
-70.564,-22.515,120
-70.537,-20.515,120
-70.515,-18.515,120
-70.492,-16.148,120,0
-70.492,-16.148,0,0
#PATH-POINTS-START Path
-69.25,-16.148,120,0
-69.68,-18.101,120
-70.106,-20.055,120
-70.53,-22.01,120
-70.952,-23.965,120
-71.372,-25.92,120
-71.792,-27.876,120
-72.211,-29.831,120
-72.631,-31.787,120
-73.052,-33.742,120
-73.476,-35.697,120
-73.903,-37.65,120
-74.336,-39.603,120
-74.776,-41.554,120
-75.223,-43.503,120
-75.68,-45.45,120
-76.15,-47.395,120
-76.633,-49.335,120
-77.131,-51.272,120
-77.649,-53.204,120
-78.189,-55.13,120
-78.753,-57.049,120
-79.343,-58.959,120
-79.964,-60.86,120
-80.619,-62.75,120
-81.311,-64.627,120
-82.042,-66.488,120
-82.818,-68.332,120
-83.639,-70.155,120
-84.509,-71.956,120
-85.429,-73.732,120
-86.402,-75.479,120
-87.427,-77.197,120
-88.504,-78.881,120
-89.634,-80.532,120
-90.815,-82.146,120
-92.044,-83.723,120
-93.321,-85.262,120
-94.642,-86.764,120
-96.005,-88.228,120
-97.407,-89.654,120
-98.845,-91.044,120
-100.315,-92.399,120
-101.815,-93.722,120
-103.344,-95.012,120
-104.897,-96.271,120
-106.473,-97.503,120
-108.07,-98.706,120
-109.686,-99.886,120
-111.319,-101.04,120
-112.967,-102.173,120
-114.63,-103.284,120
-116.305,-104.376,120
-117.993,-105.449,120
-119.691,-106.506,120
-121.399,-107.547,120
-123.116,-108.573,120
-124.841,-109.584,120
-126.574,-110.583,120
-128.313,-111.57,120
-130.059,-112.546,120
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