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
-76.393,-140.053,120,0
-76.622,-138.067,120
-76.849,-136.079,120
-77.072,-134.092,120
-77.289,-132.104,120
-77.5,-130.115,120
-77.701,-128.125,120
-77.891,-126.134,120
-78.066,-124.142,120
-78.221,-122.148,120
-78.354,-120.152,120
-78.457,-118.155,120
-78.528,-116.156,120
-78.563,-114.157,120
-78.561,-112.157,120
-78.524,-110.157,120
-78.458,-108.158,120
-78.367,-106.16,120
-78.258,-104.163,120
-78.135,-102.167,120
-78.004,-100.171,120
-77.867,-98.176,120
-77.727,-96.181,120
-77.585,-94.186,120
-77.444,-92.191,120
-77.303,-90.196,120
-77.164,-88.201,120
-77.014,-86.019,120,0
-77.014,-86.019,0,0
#PATH-POINTS-START Path
-77.014,-87.262,120,0
-76.512,-85.326,120
-76.01,-83.39,120
-75.508,-81.454,120
-75.006,-79.518,120
-74.504,-77.582,120
-74.002,-75.646,120
-73.5,-73.71,120
-72.998,-71.774,120
-72.497,-69.838,120
-71.995,-67.902,120
-71.493,-65.966,120
-70.991,-64.03,120
-70.489,-62.094,120
-69.987,-60.158,120
-69.485,-58.222,120
-68.983,-56.286,120
-68.481,-54.35,120
-67.979,-52.414,120
-67.477,-50.478,120
-66.975,-48.542,120
-66.473,-46.606,120
-65.971,-44.67,120
-65.47,-42.734,120
-64.968,-40.798,120
-64.466,-38.862,120
-63.964,-36.926,120
-63.462,-34.99,120
-62.96,-33.054,120
-62.458,-31.118,120
-61.956,-29.182,120
-61.454,-27.246,120
-60.952,-25.31,120
-60.45,-23.374,120
-59.948,-21.438,120
-59.624,-20.185,120,0
-59.624,-20.185,0,0
#PATH-POINTS-START Path
-60.866,-21.427,120,0
-61.258,-23.388,120
-61.665,-25.347,120
-62.087,-27.302,120
-62.524,-29.253,120
-62.977,-31.201,120
-63.447,-33.145,120
-63.933,-35.085,120
-64.436,-37.021,120
-64.957,-38.952,120
-65.496,-40.878,120
-66.056,-42.798,120
-66.635,-44.712,120
-67.233,-46.621,120
-67.851,-48.523,120
-68.492,-50.417,120
-69.155,-52.304,120
-69.839,-54.183,120
-70.546,-56.054,120
-71.279,-57.915,120
-72.033,-59.767,120
-72.813,-61.609,120
-73.619,-63.44,120
-74.448,-65.259,120
-75.307,-67.066,120
-76.19,-68.86,120
-77.101,-70.641,120
-78.039,-72.407,120
-79.005,-74.158,120
-79.999,-75.894,120
-81.021,-77.613,120
-82.07,-79.315,120
-83.15,-80.999,120
-84.257,-82.665,120
-85.392,-84.311,120
-86.556,-85.938,120
-87.746,-87.545,120
-88.965,-89.13,120
-90.21,-90.696,120
-91.481,-92.24,120
-92.779,-93.762,120
-94.101,-95.262,120
-95.447,-96.742,120
-96.816,-98.199,120
-98.208,-99.635,120
-99.621,-101.051,120
-101.054,-102.446,120
-102.507,-103.82,120
-103.979,-105.174,120
-105.467,-106.51,120
-106.971,-107.829,120
-108.491,-109.129,120
-110.024,-110.413,120
-111.57,-111.681,120
-113.128,-112.936,120
-114.697,-114.177,120
-116.275,-115.405,120
-117.862,-116.622,120
-119.456,-117.829,120
-121.058,-119.027,120
-122.665,-120.217,120
-124.278,-121.401,120
-125.894,-122.579,120
-127.514,-123.752,120
-129.136,-124.922,120
-130.76,-126.089,120
-132.911,-127.632,120,17.6
-132.911,-127.632,0,17.6
#PATH-POINTS-START Path
-132.911,-127.011,120,0
-132.154,-128.861,120
-131.261,-130.65,120
-130.261,-132.382,120
-129.177,-134.063,120
-128.02,-135.694,120
-126.8,-137.278,120
-125.524,-138.818,120
-124.196,-140.314,120
-122.82,-141.765,120
-121.399,-143.173,120
-119.934,-144.534,120
-118.427,-145.849,120
-116.879,-147.115,120
-115.29,-148.329,120
-113.66,-149.487,120
-111.989,-150.587,120
-110.278,-151.622,120
-108.526,-152.587,120
-106.734,-153.474,120
-106.205,-153.717,120,13.899999999999999
-106.205,-153.717,0,13.899999999999999
#PATH-POINTS-START Path
-106.826,-154.338,120,0
-104.828,-154.232,120
-102.831,-154.133,120
-100.833,-154.042,120
-98.835,-153.958,120
-96.836,-153.882,120
-94.837,-153.813,120
-92.838,-153.752,120
-90.839,-153.696,120
-88.84,-153.649,120
-86.84,-153.607,120
-84.84,-153.572,120
-82.841,-153.542,120
-80.841,-153.519,120
-78.841,-153.502,120
-76.841,-153.491,120
-74.841,-153.485,120
-72.841,-153.485,120
-70.841,-153.49,120
-68.841,-153.501,120
-66.841,-153.516,120
-64.841,-153.537,120
-62.841,-153.563,120
-60.841,-153.593,120
-58.842,-153.628,120
-56.842,-153.667,120
-54.843,-153.711,120
-52.843,-153.759,120
-50.844,-153.812,120
-48.845,-153.868,120
-46.846,-153.928,120
-44.847,-153.992,120
-42.848,-154.06,120
-40.849,-154.131,120
-38.85,-154.206,120
-36.852,-154.283,120
-34.854,-154.364,120
-32.855,-154.447,120
-30.857,-154.533,120
-28.859,-154.621,120
-26.861,-154.711,120
-24.863,-154.802,120
-22.865,-154.895,120
-20.868,-154.989,120
-18.87,-155.082,120
-16.872,-155.175,120
-14.874,-155.265,120
-12.876,-155.352,120
-10.877,-155.433,120
-8.879,-155.504,120
-6.879,-155.558,120
-4.969,-155.58,120,0
-4.969,-155.58,0,0
#PATH-POINTS-START Path
-4.969,-153.096,120,0
-6.938,-153.446,120
-8.904,-153.81,120
-10.868,-154.189,120
-12.829,-154.58,120
-14.788,-154.984,120
-16.745,-155.397,120
-18.7,-155.819,120
-20.653,-156.249,120
-22.605,-156.684,120
-24.557,-157.123,120
-26.507,-157.565,120
-28.458,-158.006,120
-30.409,-158.444,120
-32.362,-158.878,120
-34.316,-159.306,120
-36.272,-159.723,120
-38.23,-160.129,120
-40.191,-160.52,120
-42.156,-160.892,120
-44.125,-161.244,120
-46.098,-161.574,120
-48.075,-161.876,120
-50.056,-162.148,120
-52.042,-162.388,120
-54.031,-162.593,120
-56.024,-162.759,120
-58.02,-162.884,120
-60.018,-162.967,120
-62.018,-163.006,120
-64.018,-162.994,120
-66.017,-162.934,120
-68.014,-162.827,120
-70.007,-162.667,120
-71.996,-162.455,120
-73.979,-162.195,120
-75.954,-161.88,120
-77.921,-161.517,120
-79.878,-161.105,120
-81.824,-160.642,120
-83.758,-160.136,120
-85.68,-159.581,120
-87.589,-158.985,120
-89.483,-158.344,120
-91.364,-157.664,120
-93.231,-156.947,120
-95.082,-156.19,120
-96.92,-155.401,120
-98.743,-154.579,120
-100.551,-153.723,120
-102.344,-152.838,120
-104.124,-151.927,120
-105.89,-150.988,120
-107.642,-150.022,120
-109.38,-149.033,120
-111.105,-148.021,120
-112.818,-146.989,120
-114.518,-145.935,120
-116.206,-144.863,120
-117.882,-143.772,120
-119.547,-142.664,120
-121.201,-141.539,120
-122.844,-140.399,120
-124.476,-139.243,120
-126.099,-138.073,120
-127.711,-136.89,120
-129.314,-135.694,120
-130.908,-134.486,120
-132.493,-133.266,120
-134.069,-132.036,120
-135.638,-130.795,120,0
-137.259,-129.495,120
-137.259,-129.495,0,0)";

std::vector<std::vector<pathPoint>> pathMain = convert_to_path(path_str);
std::vector<double> finSpeed = {400, 300, 0, 0, 0, 0};
std::vector<double> startSpeed = {600, 400, 600, 400, 300, 400};
//