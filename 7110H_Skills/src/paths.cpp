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
100.201,-135.604,120,0
102.2,-135.666,120
104.2,-135.671,120
106.199,-135.615,120
108.195,-135.488,120
110.185,-135.292,120
112.166,-135.022,120
114.136,-134.674,120
116.089,-134.244,120
118.021,-133.729,120
119.928,-133.125,120
121.803,-132.431,120
123.642,-131.646,120
125.439,-130.768,120
127.188,-129.8,120
128.885,-128.742,120
130.524,-127.595,120
132.098,-126.363,120
133.608,-125.051,120
135.05,-123.665,120
136.419,-122.208,120
137.717,-120.687,120
138.944,-119.107,120
140.1,-117.476,120
141.184,-115.795,120
142.199,-114.072,120
143.149,-112.312,120
144.035,-110.519,120
144.858,-108.697,120
145.621,-106.848,120
146.328,-104.977,120
146.981,-103.087,120
147.583,-101.18,120
148.135,-99.257,120
148.64,-97.322,120
149.099,-95.376,120
149.517,-93.42,120
149.894,-91.456,120
150.233,-89.485,120
150.535,-87.508,120
150.802,-85.526,120
151.036,-83.54,120
151.238,-81.55,120
151.408,-79.557,120
151.549,-77.562,120
151.662,-75.565,120
151.748,-73.567,120
151.807,-71.568,120
151.832,-70.334,120,0
151.832,-70.334,0,0
#PATH-POINTS-START Path
155.391,-107.186,120,0
153.447,-106.717,120
151.499,-106.266,120
149.547,-105.83,120
147.592,-105.41,120
145.633,-105.005,120
143.672,-104.613,120
141.708,-104.232,120
139.743,-103.861,120
137.776,-103.5,120
135.807,-103.147,120
133.838,-102.801,120
131.867,-102.461,120
129.895,-102.126,120
127.922,-101.795,120
125.949,-101.468,120
123.976,-101.142,120
122.003,-100.817,120
120.029,-100.492,120
118.056,-100.165,120
116.083,-99.837,120
114.111,-99.506,120
112.139,-99.17,120
110.168,-98.83,120
108.199,-98.483,120
106.23,-98.128,120
104.264,-97.766,120
102.299,-97.394,120
100.336,-97.011,120
98.375,-96.614,120
96.418,-96.205,120
94.463,-95.782,120
92.512,-95.342,120
90.565,-94.884,120
88.623,-94.406,120
86.686,-93.909,120
84.755,-93.387,120
82.831,-92.84,120
80.915,-92.269,120
79.007,-91.667,120
77.11,-91.034,120
75.224,-90.369,120
73.352,-89.667,120
71.494,-88.927,120
69.652,-88.147,120
67.831,-87.321,120
66.029,-86.452,120
64.255,-85.529,120
62.507,-84.557,120
60.793,-83.527,120
59.113,-82.442,120
57.476,-81.293,120
55.882,-80.086,120
54.343,-78.809,120
52.856,-77.472,120
51.436,-76.063,120
50.081,-74.592,120
48.802,-73.056,120
47.601,-71.457,120
46.481,-69.8,120
45.452,-68.086,120
44.506,-66.324,120
43.658,-64.513,120
42.893,-62.665,120
42.227,-60.78,120
41.644,-58.867,120
41.148,-56.929,120
40.739,-54.972,120
40.404,-53.001,120
40.149,-51.017,120
39.968,-49.025,120
39.852,-47.029,120
39.799,-45.03,120
39.807,-43.03,120
39.875,-41.031,120
40.093,-37.766,120,0
40.093,-37.766,0,0
#PATH-POINTS-START Path
40.093,-37.162,120,0
40.13,-35.163,120
40.166,-33.163,120
40.203,-31.163,120
40.239,-29.164,120
40.276,-27.164,120
40.313,-25.164,120
40.349,-23.165,120
40.386,-21.165,120
40.422,-19.165,120
40.459,-17.166,120
40.495,-15.166,120
40.532,-13.166,120
40.568,-11.167,120
40.605,-9.167,120
40.641,-7.167,120
40.678,-5.168,120
40.714,-3.168,120
40.751,-1.168,120
40.787,0.831,120
40.824,2.831,120
40.86,4.831,120
40.897,6.83,120
40.933,8.83,120
40.97,10.83,120
41.006,12.829,120
41.043,14.829,120
41.08,16.829,120
41.116,18.828,120
41.153,20.828,120
41.189,22.828,120
41.226,24.827,120
41.262,26.827,120
41.299,28.827,120
41.335,30.826,120
41.372,32.826,120
41.408,34.826,120
41.445,36.825,120
41.481,38.825,120
41.518,40.825,120
41.554,42.824,120
41.591,44.824,120
41.627,46.824,120
41.664,48.823,120
41.7,50.823,120
41.737,52.823,120
41.773,54.822,120
41.81,56.822,120
41.847,58.822,120
41.883,60.821,120
41.92,62.821,120
41.956,64.821,120
41.993,66.82,120
42.029,68.82,120
42.066,70.82,120
42.102,72.819,120
42.139,74.819,120
42.175,76.819,120
42.212,78.818,120
42.248,80.818,120
42.285,82.818,120
42.321,84.817,120
42.358,86.817,120
42.394,88.817,120
42.431,90.816,120
42.467,92.816,120
42.508,95.038,120,0
42.508,95.038,0,0)";
std::vector<std::vector<pathPoint>> pathMain = convert_to_path(path_str);
std::vector<double> finSpeed = {500, 600, 100};
std::vector<double> startSpeed = {600, 600, 600};
//