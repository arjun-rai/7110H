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
81.736,-137.567,120,0
81.919,-135.575,120
82.064,-133.58,120
82.169,-131.583,120
82.237,-129.584,120
82.265,-127.585,120
82.25,-125.585,120
82.198,-123.586,120
82.109,-121.588,120
81.983,-119.592,120
81.817,-117.599,120
81.613,-115.609,120
81.375,-113.623,120
81.102,-111.642,120
80.792,-109.666,120
80.446,-107.696,120
80.068,-105.732,120
79.658,-103.775,120
79.214,-101.825,120
78.737,-99.883,120
78.23,-97.948,120
77.694,-96.021,120
77.125,-94.104,120
76.528,-92.195,120
75.904,-90.295,120
75.252,-88.404,120
74.571,-86.524,120
73.865,-84.653,120
73.134,-82.791,120
72.376,-80.94,120
71.594,-79.1,120
70.79,-77.269,120
69.96,-75.448,120
69.108,-73.639,120
68.236,-71.839,120
67.341,-70.051,120
66.424,-68.273,120
65.489,-66.506,120
64.532,-64.749,120
63.556,-63.004,120
62.562,-61.268,120
61.548,-59.544,120
60.517,-57.831,120
59.468,-56.128,120
58.402,-54.436,120
57.32,-52.753,120
56.22,-51.083,120
55.106,-49.422,120
53.975,-47.773,120
52.829,-46.134,120
51.668,-44.505,120
50.493,-42.887,120
49.303,-41.279,120
48.1,-39.681,120
46.882,-38.095,120
45.652,-36.518,120
44.408,-34.952,120
43.151,-33.396,120
41.882,-31.85,120
40.601,-30.315,120
39.308,-28.789,120
38.002,-27.274,120
36.685,-25.769,120
35.356,-24.274,120
34.017,-22.789,120
32.666,-21.314,120
31.305,-19.849,120
29.933,-18.393,120
28.424,-16.813,120,0
28.424,-16.813,0,0
#PATH-POINTS-START Path
16.863,-56.636,120,0
18.275,-55.221,120
19.731,-53.849,120
21.239,-52.535,120
22.794,-51.278,120
24.405,-50.093,120
26.066,-48.981,120
27.781,-47.951,120
29.55,-47.018,120
31.368,-46.187,120
33.233,-45.466,120
35.14,-44.865,120
37.083,-44.392,120
39.053,-44.053,120
41.043,-43.851,120
43.041,-43.787,120
45.039,-43.859,120
47.028,-44.068,120
48.999,-44.406,120
50.946,-44.863,120
52.863,-45.432,120
54.746,-46.105,120
56.592,-46.872,120
58.4,-47.728,120
60.167,-48.664,120
61.895,-49.671,120
63.584,-50.742,120
65.233,-51.873,120
66.845,-53.056,120
68.421,-54.288,120
69.96,-55.565,120
71.465,-56.882,120
72.938,-58.235,120
74.378,-59.623,120
75.786,-61.043,120
77.165,-62.491,120
78.516,-63.967,120
79.839,-65.467,120
81.135,-66.989,120
82.405,-68.535,120
83.649,-70.1,120
84.87,-71.685,120
86.065,-73.288,120
87.238,-74.908,120
88.387,-76.545,120
89.512,-78.199,120
90.614,-79.868,120
91.37,-81.044,120,0
91.37,-81.044,0,0
#PATH-POINTS-START Path
92.013,-81.044,120,0
93.521,-82.356,120
94.994,-83.71,120
96.428,-85.103,120
97.824,-86.536,120
99.179,-88.007,120
100.491,-89.516,120
101.76,-91.061,120
102.983,-92.644,120
104.158,-94.262,120
105.282,-95.917,120
106.35,-97.607,120
107.36,-99.334,120
108.308,-101.095,120
109.19,-102.889,120
109.998,-104.719,120
110.727,-106.581,120
111.372,-108.474,120
111.919,-110.397,120
112.367,-112.346,120
112.695,-114.319,120
112.901,-116.308,120
112.967,-118.306,120
112.88,-120.304,120
112.63,-122.287,120
112.207,-124.241,120
111.603,-126.147,120
110.813,-127.983,120
109.842,-129.731,120
108.702,-131.373,120
107.409,-132.897,120
105.978,-134.293,120
104.431,-135.56,120
102.787,-136.698,120
101.066,-137.716,120
99.279,-138.614,120
97.442,-139.404,120
95.566,-140.094,120
93.658,-140.694,120
91.726,-141.209,120
89.774,-141.648,120
87.808,-142.013,120
85.831,-142.316,120
83.846,-142.56,120
82.378,-142.705,120,0
82.378,-142.705,0,0
#PATH-POINTS-START Path
19.432,-149.128,120,0
21.417,-148.884,120
23.405,-148.666,120
25.396,-148.476,120
27.389,-148.313,120
29.384,-148.176,120
31.381,-148.063,120
33.379,-147.974,120
35.378,-147.906,120
37.377,-147.86,120
39.377,-147.833,120
41.377,-147.823,120
43.377,-147.83,120
45.377,-147.85,120
47.377,-147.881,120
49.376,-147.923,120
51.376,-147.972,120
53.375,-148.026,120
55.374,-148.082,120
57.374,-148.138,120
59.373,-148.19,120
61.372,-148.235,120
63.372,-148.271,120
65.372,-148.293,120
67.372,-148.297,120
69.372,-148.28,120
71.371,-148.236,120
73.37,-148.161,120
75.367,-148.051,120
77.361,-147.9,120
79.351,-147.702,120
81.336,-147.453,120
83.312,-147.147,120
85.277,-146.778,120
87.229,-146.342,120
89.163,-145.832,120
91.075,-145.247,120
92.962,-144.583,120
94.816,-143.834,120
96.636,-143.005,120
98.415,-142.093,120
100.15,-141.098,120
101.838,-140.025,120
103.477,-138.879,120
105.062,-137.66,120
106.594,-136.374,120
108.073,-135.028,120
109.498,-133.626,120
110.639,-132.428,120,0
110.639,-132.428,0,0
#PATH-POINTS-START Path
109.997,-133.07,120,0
111.357,-131.604,120
112.657,-130.085,120
113.892,-128.512,120
115.057,-126.886,120
116.146,-125.209,120
117.157,-123.484,120
118.088,-121.713,120
118.936,-119.903,120
119.703,-118.055,120
120.387,-116.176,120
120.993,-114.27,120
121.525,-112.343,120
121.988,-110.397,120
122.387,-108.437,120
122.729,-106.467,120
123.019,-104.488,120
123.266,-102.503,120
123.477,-100.515,120
123.659,-98.523,120
123.818,-96.529,120
123.961,-94.534,120
124.096,-92.539,120
124.228,-90.543,120
124.364,-88.548,120
124.51,-86.553,120
124.673,-84.56,120
124.857,-82.568,120
125.068,-80.58,120
125.312,-78.595,120
125.594,-76.615,120
125.917,-74.641,120
126.288,-72.676,120
126.697,-70.767,120,0
126.697,-70.767,0,0)";



std::vector<std::vector<pathPoint>> pathMain = convert_to_path(path_str);
std::vector<double> finSpeed = {100, 150, 0, 100, 100};
std::vector<double> startSpeed = {300,300, 150, 100, 100};
//

