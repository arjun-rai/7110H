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
21.117,-149.68,120,0
23.112,-149.818,120
25.11,-149.892,120
27.11,-149.912,120
29.11,-149.889,120
31.109,-149.83,120
33.107,-149.739,120
35.104,-149.622,120
37.099,-149.481,120
39.092,-149.319,120
41.084,-149.14,120
43.074,-148.944,120
45.063,-148.734,120
47.051,-148.512,120
49.037,-148.279,120
51.022,-148.036,120
53.007,-147.784,120
54.99,-147.524,120
56.972,-147.258,120
58.953,-146.985,120
60.934,-146.708,120
62.914,-146.426,120
64.894,-146.142,120
66.873,-145.854,120
68.852,-145.565,120
70.831,-145.275,120
72.809,-144.985,120
74.788,-144.695,120
76.768,-144.407,120
78.747,-144.122,120
80.727,-143.841,120
82.708,-143.564,120
84.69,-143.294,120
86.672,-143.032,120
88.656,-142.78,120
90.642,-142.539,120
92.629,-142.315,120
94.619,-142.109,120
96.61,-141.927,120
98.605,-141.776,120
100.601,-141.665,120
103.099,-141.606,120,0
103.099,-141.606,0,0
#PATH-POINTS-START Path
104.341,-142.227,120,0
106.222,-141.545,120
108.102,-140.864,120
109.982,-140.182,120
111.862,-139.5,120
113.743,-138.819,120
115.623,-138.137,120
117.503,-137.455,120
119.383,-136.774,120
121.263,-136.092,120
123.144,-135.41,120
125.024,-134.729,120
126.904,-134.047,120
128.407,-133.502,120,0
128.407,-133.502,0,0
#PATH-POINTS-START Path
129.707,-134.151,120,0
131.1,-132.716,120
132.492,-131.281,120
133.885,-129.846,120
135.278,-128.411,120
136.671,-126.976,120
138.064,-125.541,120
139.457,-124.105,120
140.85,-122.67,120
142.243,-121.235,120
143.636,-119.8,120
145.029,-118.365,120
146.422,-116.93,120
147.815,-115.494,120
149.208,-114.059,120
150.601,-112.624,120
151.159,-110.844,120,0
151.183,-108.844,120
151.208,-106.845,120
151.233,-104.845,120
151.258,-102.845,120
151.282,-100.845,120
151.307,-98.845,120
151.332,-96.845,120
151.356,-94.845,120
151.381,-92.846,120
151.406,-90.846,120
151.43,-88.846,120
151.455,-86.846,120
151.48,-84.846,120
151.504,-82.846,120
151.529,-80.847,120
151.554,-78.847,120
151.578,-76.847,120
151.603,-74.847,120
151.628,-72.847,120
151.653,-70.847,120
151.677,-68.847,120
151.702,-66.848,120
151.727,-64.848,120
151.751,-62.848,120
151.776,-60.848,120
151.793,-59.446,120,0
151.793,-59.446,0,0
#PATH-POINTS-START Path
150.301,-62.108,120,0
150.037,-64.09,120
149.773,-66.073,120
149.508,-68.055,120
149.244,-70.038,120
148.98,-72.02,120
148.715,-74.003,120
148.451,-75.985,120
148.187,-77.968,120
147.922,-79.95,120
147.658,-81.932,120
147.394,-83.915,120
147.129,-85.897,120
146.865,-87.88,120
146.601,-89.862,120
146.336,-91.845,120
146.072,-93.827,120
145.808,-95.81,120
145.543,-97.792,120
145.333,-99.373,120,0
145.333,-99.373,0,0)";

std::vector<std::vector<pathPoint>> pathMain = convert_to_path(path_str);
std::vector<double> finSpeed = {300, 0, 600, 0};
std::vector<double> startSpeed = {400, 300, 400, 300};
//