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
29.444,-150.048,120,0
31.444,-150.053,120
33.444,-150.066,120
35.444,-150.085,120
37.444,-150.106,120
39.444,-150.127,120
41.444,-150.145,120
43.444,-150.159,120
45.444,-150.165,120
47.444,-150.162,120
49.443,-150.147,120
51.443,-150.119,120
53.443,-150.074,120
55.442,-150.011,120
57.44,-149.928,120
59.437,-149.823,120
61.433,-149.693,120
63.427,-149.537,120
65.418,-149.353,120
67.407,-149.138,120
69.391,-148.89,120
71.372,-148.609,120
73.346,-148.293,120
75.315,-147.938,120
77.275,-147.544,120
79.227,-147.109,120
81.17,-146.633,120
83.101,-146.112,120
85.019,-145.547,120
86.924,-144.936,120
88.813,-144.279,120
90.684,-143.574,120
92.537,-142.822,120
94.37,-142.022,120
96.181,-141.174,120
97.97,-140.279,120
99.311,-139.568,120,0
99.311,-139.568,0,0
#PATH-POINTS-START Path
98.313,-140.566,120,0
99.921,-139.376,120
101.527,-138.185,120
103.131,-136.99,120
104.73,-135.789,120
106.322,-134.578,120
107.905,-133.356,120
109.474,-132.116,120
111.028,-130.856,120
112.56,-129.571,120
114.067,-128.256,120
115.542,-126.906,120
116.979,-125.515,120
118.371,-124.079,120
119.71,-122.593,120
120.987,-121.054,120
122.194,-119.46,120
123.323,-117.809,120
124.367,-116.104,120
125.323,-114.347,120
126.187,-112.544,120
126.958,-110.699,120
127.64,-108.819,120
128.234,-106.909,120
128.745,-104.976,120
129.18,-103.024,120
129.544,-101.057,120
129.843,-99.08,120
130.083,-97.094,120
130.27,-95.103,120
130.409,-93.108,120
130.504,-91.11,120
130.559,-89.111,120
130.579,-87.111,120
130.566,-85.111,120
130.524,-83.112,120
130.456,-81.113,120
130.364,-79.115,120
130.25,-77.118,120
130.115,-75.123,120
129.962,-73.129,120
129.753,-70.699,120,0
129.753,-70.699,0,0
#PATH-POINTS-START Path
19.962,-81.179,120,0
20.462,-79.242,120
20.934,-77.299,120
21.378,-75.349,120
21.793,-73.393,120
22.18,-71.43,120
22.536,-69.462,120
22.864,-67.489,120
23.163,-65.512,120
23.434,-63.53,120
23.678,-61.545,120
23.897,-59.557,120
24.094,-57.567,120
24.273,-55.575,120
24.437,-53.582,120
24.594,-51.588,120
24.751,-49.594,120
24.919,-47.601,120
25.109,-45.61,120
25.336,-43.623,120
25.618,-41.643,120
25.977,-39.676,120
26.432,-37.729,120
27.005,-35.813,120
27.712,-33.943,120
28.565,-32.134,120
29.561,-30.401,120
30.693,-28.752,120
31.945,-27.193,120
33.299,-25.722,120
34.74,-24.335,120
35.932,-23.289,120,0
35.932,-23.289,0,0
#PATH-POINTS-START Path
35.433,-23.289,120,0
37.429,-23.162,120
39.425,-23.036,120
41.421,-22.909,120
43.417,-22.782,120
45.413,-22.655,120
47.409,-22.529,120
49.405,-22.402,120
51.401,-22.275,120
53.396,-22.149,120
55.392,-22.022,120
57.388,-21.895,120
59.384,-21.768,120
61.38,-21.642,120
63.376,-21.515,120
65.372,-21.388,120
67.368,-21.261,120
69.364,-21.135,120
71.36,-21.008,120
73.356,-20.881,120
75.352,-20.754,120
77.348,-20.628,120
79.344,-20.501,120
81.34,-20.374,120
83.336,-20.248,120
85.332,-20.121,120
87.328,-19.994,120
89.324,-19.867,120
91.32,-19.741,120
93.316,-19.614,120
95.312,-19.487,120
97.308,-19.36,120
99.304,-19.234,120
101.3,-19.107,120
103.296,-18.98,120
105.292,-18.854,120
107.288,-18.727,120
109.284,-18.6,120
111.28,-18.473,120
113.276,-18.347,120
115.272,-18.22,120
117.268,-18.093,120
119.264,-17.966,120
121.26,-17.84,120
123.256,-17.713,120
125.252,-17.586,120
127.248,-17.46,120
129.753,-17.3,120,0
129.753,-17.3,0,0)";
std::vector<std::vector<pathPoint>> pathMain = convert_to_path(path_str);
std::vector<double> finSpeed = {50, 100, 100, 100};
std::vector<double> startSpeed = {300,75,50, 100};
//

